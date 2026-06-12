/** @file
 *  @brief BLE CPS Service created based on Zephyr's HRS sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/check.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include "cps.h"

/* CPS UUIDs - Cycling Power Service specification */
#define BT_UUID_CPS_VAL 0x1818
#define BT_UUID_CPS BT_UUID_DECLARE_16(BT_UUID_CPS_VAL)
#define BT_UUID_CPS_MEASUREMENT_VAL 0x2A63
#define BT_UUID_CPS_MEASUREMENT BT_UUID_DECLARE_16(BT_UUID_CPS_MEASUREMENT_VAL)
#define BT_UUID_CPS_CONTROL_POINT_VAL 0x2A76
#define BT_UUID_CPS_CONTROL_POINT BT_UUID_DECLARE_16(BT_UUID_CPS_CONTROL_POINT_VAL)
#define BT_UUID_CPS_RESPONSE_VAL 0x2A91
#define BT_UUID_CPS_RESPONSE BT_UUID_DECLARE_16(BT_UUID_CPS_RESPONSE_VAL)
/* BT_UUID_SENSOR_LOCATION is already defined in Zephyr BLE headers */

#ifndef CONFIG_BT_CPS_LOG_LEVEL
#define CONFIG_BT_CPS_LOG_LEVEL 3
#endif

#define LOG_LEVEL CONFIG_BT_CPS_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cps);

#define GATT_PERM_READ_MASK (BT_GATT_PERM_READ |         \
							 BT_GATT_PERM_READ_ENCRYPT | \
							 BT_GATT_PERM_READ_AUTHEN)
#define GATT_PERM_WRITE_MASK (BT_GATT_PERM_WRITE |         \
							  BT_GATT_PERM_WRITE_ENCRYPT | \
							  BT_GATT_PERM_WRITE_AUTHEN)

#ifndef CONFIG_BT_CPS_DEFAULT_PERM_RW_AUTHEN
#define CONFIG_BT_CPS_DEFAULT_PERM_RW_AUTHEN 0
#endif
#ifndef CONFIG_BT_CPS_DEFAULT_PERM_RW_ENCRYPT
#define CONFIG_BT_CPS_DEFAULT_PERM_RW_ENCRYPT 0
#endif
#ifndef CONFIG_BT_CPS_DEFAULT_PERM_RW
#define CONFIG_BT_CPS_DEFAULT_PERM_RW 0
#endif

#define CPS_GATT_PERM_DEFAULT (                                                                                                                                                                      \
	CONFIG_BT_CPS_DEFAULT_PERM_RW_AUTHEN ? (BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN) : CONFIG_BT_CPS_DEFAULT_PERM_RW_ENCRYPT ? (BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT) \
																																		  : (BT_GATT_PERM_READ | BT_GATT_PERM_WRITE))

static uint8_t cps_blsc;				   /* Sensor location: 0x01 = crank */
static uint16_t cps_crank_length_mm = 170; /* Default crank length in mm */
static uint8_t cps_response[5];			   /* Control Point Response buffer: OpCode + ReqOpCode + Result + Data (2 bytes) */
static uint8_t cps_response_len = 0;	   /* Current response length */
static sys_slist_t cps_cbs = SYS_SLIST_STATIC_INIT(&cps_cbs);

static void cps_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	struct bt_cps_cb *listener;

	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	LOG_INF("CPS notifications %s", notif_enabled ? "enabled" : "disabled");

	SYS_SLIST_FOR_EACH_CONTAINER(&cps_cbs, listener, _node)
	{
		if (listener->ntf_changed)
		{
			listener->ntf_changed(notif_enabled);
		}
	}
}

static ssize_t read_blsc(struct bt_conn *conn, const struct bt_gatt_attr *attr,
						 void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &cps_blsc,
							 sizeof(cps_blsc));
}

static ssize_t read_cps_response(struct bt_conn *conn, const struct bt_gatt_attr *attr,
								 void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &cps_response,
							 cps_response_len);
}

static ssize_t ctrl_point_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
								const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	int err = -ENOTSUP;
	struct bt_cps_cb *listener;
	uint8_t opcode = *((uint8_t *)buf);

	LOG_INF("CPS CTRL Point Written (opcode=0x%02x, len=%d)", opcode, len);

	/* Handle crank length adjustment (op 0x05) locally */
	if (opcode == BT_CPS_CONTROL_POINT_SET_CRANK_LENGTH_REQ)
	{
		if (len >= 3)
		{
			cps_crank_length_mm = sys_get_le16((uint8_t *)buf + 1);
			LOG_INF("Crank length set to %u mm", cps_crank_length_mm);

			/* Build response: OpCode(0x20) + ReqOpCode(0x05) + Result(0x01=success) + Data(crank_length) */
			cps_response[0] = 0x20; /* Response OpCode */
			cps_response[1] = 0x05; /* Request OpCode */
			cps_response[2] = 0x01; /* Result Code: Success */
			sys_put_le16(cps_crank_length_mm, &cps_response[3]);
			cps_response_len = 5;

			return len;
		}
		else
		{
			LOG_ERR("Invalid crank length request length: %d", len);
			/* Build error response */
			cps_response[0] = 0x20; /* Response OpCode */
			cps_response[1] = 0x05; /* Request OpCode */
			cps_response[2] = 0x03; /* Result Code: Invalid parameter */
			cps_response_len = 3;
			return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
		}
	}

	SYS_SLIST_FOR_EACH_CONTAINER(&cps_cbs, listener, _node)
	{
		if (listener->ctrl_point_write)
		{
			err = listener->ctrl_point_write(opcode, (uint8_t *)buf, len);
			/* If we get an error other than ENOTSUP then immediately
			 * break the loop and return a generic gatt error, assuming this
			 * listener supports this request code, but failed to serve it
			 */
			if ((err != 0) && (err != -ENOTSUP))
			{
				return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
			}
		}
	}
	if (err)
	{
		return BT_GATT_ERR(BT_CPS_ATT_ERR_CONTROL_POINT_NOT_SUPPORTED);
	}
	else
	{
		return len;
	}
}

/* Cycling Power Service Declaration */
BT_GATT_SERVICE_DEFINE(cps_svc,
					   BT_GATT_PRIMARY_SERVICE(BT_UUID_CPS),
					   BT_GATT_CHARACTERISTIC(BT_UUID_CPS_MEASUREMENT, BT_GATT_CHRC_NOTIFY,
											  BT_GATT_PERM_NONE, NULL, NULL, NULL),
					   BT_GATT_CCC(cps_ccc_cfg_changed,
								   CPS_GATT_PERM_DEFAULT),
					   BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_LOCATION, BT_GATT_CHRC_READ,
											  CPS_GATT_PERM_DEFAULT &GATT_PERM_READ_MASK,
											  read_blsc, NULL, NULL),
					   BT_GATT_CHARACTERISTIC(BT_UUID_CPS_CONTROL_POINT, BT_GATT_CHRC_WRITE,
											  CPS_GATT_PERM_DEFAULT &GATT_PERM_WRITE_MASK,
											  NULL, ctrl_point_write, NULL),
					   BT_GATT_CHARACTERISTIC(BT_UUID_CPS_RESPONSE, BT_GATT_CHRC_READ,
											  CPS_GATT_PERM_DEFAULT &GATT_PERM_READ_MASK,
											  read_cps_response, NULL, NULL), );
static int cps_init(void)
{
	cps_blsc = 0x01;

	return 0;
}

int bt_cps_cb_register(struct bt_cps_cb *cb)
{
	CHECKIF(cb == NULL)
	{
		return -EINVAL;
	}

	sys_slist_append(&cps_cbs, &cb->_node);

	return 0;
}

int bt_cps_cb_unregister(struct bt_cps_cb *cb)
{
	CHECKIF(cb == NULL)
	{
		return -EINVAL;
	}

	if (!sys_slist_find_and_remove(&cps_cbs, &cb->_node))
	{
		return -ENOENT;
	}

	return 0;
}

int bt_cps_notify(int16_t power_watts, uint16_t crank_revolutions, uint16_t crank_event_time)
{
	// int rc;
	// static uint8_t cps_meas[9]; /* Flags (2) + Power (2) + Crank Rev (2) + Crank Time (2) + Pedal Balance (1) */
	// uint8_t *pos = cps_meas;

	// /* Flags (16-bit, little-endian): 0x0021 = Pedal Power Balance + Crank Revolution Data Present */
	// sys_put_le16(0x0021, pos);
	// pos += 2;

	// /* Instantaneous Power (16-bit, little-endian) */
	// sys_put_le16(power_watts, pos);
	// pos += 2;

	// /* Crank Revolutions (16-bit, little-endian) */
	// sys_put_le16(crank_revolutions, pos);
	// pos += 2;

	// /* Crank Event Time (16-bit, little-endian, units of 1/1024 second) */
	// sys_put_le16(crank_event_time, pos);
	// pos += 2;

	// /* Pedal Power Balance (8-bit): 128 = 50/50 split between left/right */
	// *pos++ = 128;

	// rc = bt_gatt_notify(NULL, &cps_svc.attrs[1], &cps_meas, sizeof(cps_meas));

	// return rc == -ENOTCONN ? 0 : rc;

	int rc;
	uint8_t cps_meas[8]; /* Flags (2) + Power (2) + Crank Rev (2) + Crank Time (2) */
	uint8_t *pos = cps_meas;

	/* Flags: Crank Revolution Data Present */
	sys_put_le16(0x0020, pos);
	pos += sizeof(uint16_t);

	/* Instantaneous Power: sint16, watts */
	sys_put_le16((uint16_t)power_watts, pos);
	pos += sizeof(int16_t);

	// *pos++ = 100; /* 50.0% if using 0.5% units */

	/* Cumulative Crank Revolutions: uint16 */
	sys_put_le16(crank_revolutions, pos);
	pos += sizeof(uint16_t);

	/* Cumulative Crank Revolution Time: uint16_t */
	sys_put_le16((uint16_t)crank_event_time, pos);

	rc = bt_gatt_notify(NULL, &cps_svc.attrs[2], cps_meas, sizeof(cps_meas));

	return rc == -ENOTCONN ? 0 : rc;
}

SYS_INIT(cps_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
