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

K_EVENT_DEFINE(device_connected_event);

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

    // peripheral_connected = true;
    k_event_set(&device_connected_event, DEV_CONNECTED);
    // dk_set_led_on(PERIPHERAL_CONN_STATUS_LED);
    LOG_INF("Peripheral connected: %s\n", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // peripheral_connected = false;
    k_event_clear(&device_connected_event, DEV_CONNECTED);
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

    // Continue to main loop after START bit received
    k_event_wait(&thread_sync_event, START_BIT, false, K_FOREVER);

    while (1)
    {
        // Only send data if host connected. Do not reset so that ble loop continues
        // while device is connected. Otherwise, wait forever.
        uint32_t events = k_event_wait(&device_connected_event, DEV_CONNECTED, false, K_FOREVER);
        if (events & DEV_CONNECTED)
        {
            // Wait for data from power_meas thread
            struct ble_data_t *data = k_fifo_get(&ble_fifo, K_FOREVER);
            if (data)
            {
                // LOG_INF("rcv: %u, %u, %u", data->power, data->crank_index, data->crank_event_time);

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

#if IS_ENABLED(CONFIG_NUS_DEBUG_BUILD)
                char buf[100];
                // int s = sprintf(buf, "CPS: Pwr=%03uW, Rev=%02u, Time=%06u", data->power, data->crank_index, data->crank_event_time);
                // int s = sprintf(buf, "%03u,%03u,%05u", data->power, data->crank_index, data->crank_event_time);
                int s = sprintf(buf, "%03u,%03u,%05u", data->power, data->crank_index, data->crank_event_time);

                int err = bt_nus_send(NULL, &buf, s);
                if (err < 0 && (err != -EAGAIN) && (err != -ENOTCONN))
                {
                    LOG_WRN("NUS failed!");
                    return;
                }
#endif

                // Free slab
                k_mem_slab_free(&ble_data_slab, (void *)data);
            }
            else
                continue; // Skip processing if no filter data ready
        }
    }
}

K_THREAD_DEFINE(cps_notify_thread_id, STACKSIZE, cps_notify_thread,
                NULL, NULL, NULL, 3, 0, 0); // Lowest priority in power calculation chain

#if IS_ENABLED(CONFIG_NUS_DEBUG_BUILD)
static void nus_thread(void)
{
    // char buf[] = "Hello, world!";
    // char buf[50];
    while (1)
    {
        // struct filter_data_t *data = k_fifo_peek_head(&filter_fifo);
        // if (data)
        // {
        //     sprintf(buf, "Omega: %03.2f, Theta: %03.2f", data->omega, data->theta);
        // }

        // int err = bt_nus_send(NULL, &buf, 29);
        // if (err < 0 && (err != -EAGAIN) && (err != -ENOTCONN))
        // {
        //     return;
        // }

        k_sleep(K_MSEC(50));
    }
}

K_THREAD_DEFINE(nus_thread_id, STACKSIZE, nus_thread,
                NULL, NULL, NULL, 0, 0, 0); // Very low priority - just logging

// static int cmd_pm_status(const struct shell *sh, size_t argc, char **argv)
// {
//     shell_print(sh, "NUS debug build is active");
//     return 0;
// }

// SHELL_CMD_REGISTER(pm_status, NULL, "Print power meter debug status", cmd_pm_status);
#endif