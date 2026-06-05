#include "ble_thread.h"

// LOG_MODULE_REGISTER(ble, LOG_LEVEL_DBG);
LOG_MODULE_REGISTER(ble, LOG_LEVEL_INF);

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .recycled = recycled_cb,
};

K_MEM_SLAB_DEFINE(ble_data_slab,
                  sizeof(struct ble_data_t),
                  BLE_DATA_SLAB_NUM_BLOCKS,
                  BLE_DATA_SLAB_ALIGNMENT);

static void adv_work_handler(struct k_work *work)
{
    int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

    if (err)
    {
        LOG_ERR("Advertising failed to start (err %d)\n", err);
        return;
    }

    LOG_INF("Advertising successfully started\n");
}

static void advertising_start(void)
{
    k_work_submit(&adv_work);
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn_err)
    {
        LOG_ERR("Failed to connect to %s, 0x%02x %s\n", addr, conn_err,
                bt_hci_err_to_str(conn_err));
        return;
    }

    peripheral_connected = true;
    // dk_set_led_on(PERIPHERAL_CONN_STATUS_LED);
    LOG_INF("Peripheral connected: %s\n", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    peripheral_connected = false;
    // dk_set_led_off(PERIPHERAL_CONN_STATUS_LED);
    LOG_INF("Disconnected: %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));
}

static void recycled_cb(void)
{
    LOG_DBG("Connection object recycled, restarting advertising\n");
    advertising_start();
}

static void cps_notify_thread(void)
{
    // uint16_t power_watts = 150U;     /* Start at 150W */
    // uint16_t power_direction = 1U;   /* 1 = increasing, 0 = decreasing */
    // uint16_t crank_revolutions = 0U; /* Cumulative crank revolutions */
    // uint16_t crank_event_time = 0U;  /* Time in 1/1024 second units */
    // uint32_t elapsed_ms = 0U;
    int err;

    err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)\n", err);
        return;
    }

    k_work_init(&adv_work, adv_work_handler);
    advertising_start();

    // Post READY bit to sync event share
    k_event_post(&thread_sync_event, BLE_THREAD_READY);

    LOG_DBG("Thread ready");

    // Continue to main loop after START bit received
    k_event_wait(&thread_sync_event, START_BIT, false, K_FOREVER);

    LOG_DBG("Thread started");

    while (1)
    {
        // Only send data if we're connected to a host
        if (peripheral_connected)
        {
            // /* Update power value: ramp from 150W to 350W over 60s cycle */
            // if (power_direction == 1U)
            // {
            //     power_watts += 10U; /* Increase by 10W per notification (~1.1s apart) */
            //     if (power_watts >= 350U)
            //     {
            //         power_watts = 350U;
            //         power_direction = 0U; /* Switch to decreasing */
            //     }
            // }
            // else
            // {
            //     power_watts -= 10U; /* Decrease by 10W per notification */
            //     if (power_watts <= 150U)
            //     {
            //         power_watts = 150U;
            //         power_direction = 1U; /* Switch to increasing */
            //     }
            // }

            // /* Update crank data: simulate 90 RPM cadence */
            // /* 90 RPM = 1.5 revolutions per second */
            // /* At 1.1s intervals: 1.5 * 1.1 = 1.65 revolutions per update */
            // crank_revolutions++;
            // crank_event_time += (1024 * 1100 / 1000); /* ~1.1s in 1/1024 second units */

            // Wait for data from power_meas thread
            struct ble_data_t *data = k_fifo_get(&ble_fifo, K_FOREVER);
            if (data)
            {
                LOG_INF("rcv: %u, %u, %u", data->power, data->crank_index, data->crank_event_time);

                /* Send power data via CPS */
                if (bt_cps_notify(data->power, data->crank_index, data->crank_event_time))
                {
                    LOG_WRN("CPS notify failed\n");
                }
                else
                {
                    LOG_DBG("CPS: Power=%uW, Crank_Rev=%u, Crank_Time=%u\n",
                            data->power, data->crank_index, data->crank_event_time);
                }

                /* Send cadence data via CSCS */
                if (bt_cscs_notify(data->crank_index, data->crank_event_time))
                {
                    LOG_WRN("CSCS notify failed\n");
                }

                // Free slab
                k_mem_slab_free(&ble_data_slab, (void *)data);
            }
            else
                continue; // Skip processing if no filter data ready
        }

        // k_sleep(CPS_NOTIFY_INTERVAL);
    }
}

K_THREAD_DEFINE(cps_notify_thread_id, STACKSIZE, cps_notify_thread,
                NULL, NULL, NULL, 7, 0, 0);