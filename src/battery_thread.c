#include "battery_thread.h"

LOG_MODULE_REGISTER(batt, LOG_LEVEL_INF);

static batt_mon_t batt_mon = I2C_DT_SPEC_GET(MAX17048_NODE);

/**
 * @brief Zephyr thread for battery monitoring
 *
 */
static void battery_monitor_thread(void)
{

    if (battery_monitor_init(&batt_mon)) // return 0 when properly configured
    {
        // Post READY bit to sync event share
        k_event_post(&thread_sync_event, BATTERY_THREAD_READY);

        return;
    }

    // Post READY bit to sync event share
    k_event_post(&thread_sync_event, BATTERY_THREAD_READY);

    // Continue to main loop after START bit received
    k_event_wait(&thread_sync_event, START_BIT, false, K_FOREVER);

    while (1)
    {
        // uint8_t pct = battery_monitor_read_soc(&batt_mon);

        // char *mem_ptr = k_malloc(32);
        // sprintf(mem_ptr, "Battery SOC: %d%%", pct);
        // k_fifo_put(&printk_fifo, mem_ptr);

        uint16_t volts = battery_monitor_read_voltage(&batt_mon); // Value is returned in millivolts

        char *mem_ptr = k_malloc(32);
        sprintf(mem_ptr, "Battery voltage: %dmV", volts);
        k_fifo_put(&printk_fifo, mem_ptr);

        k_sleep(K_SECONDS(5));
    }
}

K_THREAD_DEFINE(battery_thread_id, STACKSIZE, battery_monitor_thread, NULL, NULL, NULL, 7, 0, 0);