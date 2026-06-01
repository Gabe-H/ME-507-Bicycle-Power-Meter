#ifndef BLE_TASK_H
#define BLE_TASK_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

#include "cps.h"
#include "cscs.h"
#include "app_ipc.h"

// #define RUN_STATUS_LED DK_LED1
// #define PERIPHERAL_CONN_STATUS_LED DK_LED3

// #define RUN_LED_BLINK_INTERVAL 1000
#define CPS_NOTIFY_INTERVAL K_MSEC(1100) /* ~1 Hz notification rate */

/* CPS Service UUIDs */
#define BT_UUID_CPS_VAL 0x1818
#define BT_UUID_CPS BT_UUID_DECLARE_16(BT_UUID_CPS_VAL)

/* CSCS Service UUID (Cycling Speed and Cadence Service) */
#define BT_UUID_CSCS_VAL 0x1816

/* Peripheral role: advertise CSCS (cadence) and CPS (power) for full cycling sensor */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
                  (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
                  (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                  BT_UUID_16_ENCODE(BT_UUID_CSCS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_CPS_VAL)),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME),
};

static struct k_work adv_work;
static bool peripheral_connected;

static void adv_work_handler(struct k_work *work);

static void advertising_start(void);

static void connected(struct bt_conn *conn, uint8_t conn_err);

static void disconnected(struct bt_conn *conn, uint8_t reason);

static void recycled_cb(void);

static void cps_notify_thread(void);

#endif /* BLE_TASK_H */