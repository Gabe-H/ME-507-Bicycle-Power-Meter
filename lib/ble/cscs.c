/** @file
 *  @brief CSCS Service implementation
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
#include "cscs.h"

/* CSCS UUIDs - Cycling Speed and Cadence Service specification */
#define BT_UUID_CSCS_VAL 0x1816
#define BT_UUID_CSCS BT_UUID_DECLARE_16(BT_UUID_CSCS_VAL)
#define BT_UUID_CSCS_MEASUREMENT_VAL 0x2A5B
#define BT_UUID_CSCS_MEASUREMENT BT_UUID_DECLARE_16(BT_UUID_CSCS_MEASUREMENT_VAL)
#define BT_UUID_CSCS_FEATURE_VAL 0x2A5C
#define BT_UUID_CSCS_FEATURE BT_UUID_DECLARE_16(BT_UUID_CSCS_FEATURE_VAL)
/* BT_UUID_SENSOR_LOCATION is already defined in Zephyr BLE headers */

#ifndef CONFIG_BT_CSCS_LOG_LEVEL
#define CONFIG_BT_CSCS_LOG_LEVEL 3
#endif

#define LOG_LEVEL CONFIG_BT_CSCS_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cscs);

#define GATT_PERM_READ_MASK (BT_GATT_PERM_READ |         \
                             BT_GATT_PERM_READ_ENCRYPT | \
                             BT_GATT_PERM_READ_AUTHEN)

#ifndef CONFIG_BT_CSCS_DEFAULT_PERM_RW_AUTHEN
#define CONFIG_BT_CSCS_DEFAULT_PERM_RW_AUTHEN 0
#endif
#ifndef CONFIG_BT_CSCS_DEFAULT_PERM_RW_ENCRYPT
#define CONFIG_BT_CSCS_DEFAULT_PERM_RW_ENCRYPT 0
#endif
#ifndef CONFIG_BT_CSCS_DEFAULT_PERM_RW
#define CONFIG_BT_CSCS_DEFAULT_PERM_RW 0
#endif

#define CSCS_GATT_PERM_DEFAULT ( \
	CONFIG_BT_CSCS_DEFAULT_PERM_RW_AUTHEN ? (BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN) : \
	CONFIG_BT_CSCS_DEFAULT_PERM_RW_ENCRYPT ? (BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT) : \
	(BT_GATT_PERM_READ | BT_GATT_PERM_WRITE))

static uint8_t cscs_sensor_location = 0x01; /* Sensor location: 0x01 = crank */
static uint16_t cscs_feature = 0x0002;      /* Feature flags: crank revolution data supported */
static sys_slist_t cscs_cbs = SYS_SLIST_STATIC_INIT(&cscs_cbs);

static void cscs_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);

    struct bt_cscs_cb *listener;

    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

    LOG_INF("CSCS notifications %s", notif_enabled ? "enabled" : "disabled");

    SYS_SLIST_FOR_EACH_CONTAINER(&cscs_cbs, listener, _node)
    {
        if (listener->ntf_changed)
        {
            listener->ntf_changed(notif_enabled);
        }
    }
}

static ssize_t read_cscs_feature(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &cscs_feature,
                             sizeof(cscs_feature));
}

static ssize_t read_cscs_sensor_location(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                         void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &cscs_sensor_location,
                             sizeof(cscs_sensor_location));
}

/* Cycling Speed and Cadence Service Declaration */
BT_GATT_SERVICE_DEFINE(cscs_svc,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_CSCS),
                       BT_GATT_CHARACTERISTIC(BT_UUID_CSCS_MEASUREMENT, BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_NONE, NULL, NULL, NULL),
                       BT_GATT_CCC(cscs_ccc_cfg_changed,
                                   CSCS_GATT_PERM_DEFAULT),
                       BT_GATT_CHARACTERISTIC(BT_UUID_CSCS_FEATURE, BT_GATT_CHRC_READ,
                                              CSCS_GATT_PERM_DEFAULT &GATT_PERM_READ_MASK,
                                              read_cscs_feature, NULL, NULL),
                       BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_LOCATION, BT_GATT_CHRC_READ,
                                              CSCS_GATT_PERM_DEFAULT &GATT_PERM_READ_MASK,
                                              read_cscs_sensor_location, NULL, NULL), );

static int cscs_init(void)
{
    cscs_sensor_location = 0x01; /* Crank */
    cscs_feature = 0x0002;       /* Crank revolution data supported */

    return 0;
}

int bt_cscs_cb_register(struct bt_cscs_cb *cb)
{
    CHECKIF(cb == NULL)
    {
        return -EINVAL;
    }

    sys_slist_append(&cscs_cbs, &cb->_node);

    return 0;
}

int bt_cscs_cb_unregister(struct bt_cscs_cb *cb)
{
    CHECKIF(cb == NULL)
    {
        return -EINVAL;
    }

    if (!sys_slist_find_and_remove(&cscs_cbs, &cb->_node))
    {
        return -ENOENT;
    }

    return 0;
}

int bt_cscs_notify(uint16_t crank_revolutions, uint16_t crank_event_time)
{
    int rc;
    static uint8_t cscs_meas[5]; /* Flags (1) + Crank Rev (2) + Crank Time (2) */
    uint8_t *pos = cscs_meas;

    /* Flags (8-bit): 0x02 = Crank Revolution Data Present */
    *pos++ = 0x02;

    /* Crank Revolutions (16-bit, little-endian) */
    sys_put_le16(crank_revolutions, pos);
    pos += 2;

    /* Crank Event Time (16-bit, little-endian, units of 1/1024 second) */
    sys_put_le16(crank_event_time, pos);
    pos += 2;

    rc = bt_gatt_notify(NULL, &cscs_svc.attrs[1], &cscs_meas, sizeof(cscs_meas));

    return rc == -ENOTCONN ? 0 : rc;
}

SYS_INIT(cscs_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
