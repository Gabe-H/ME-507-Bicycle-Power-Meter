/**
 * @file main.c
 * @author Gabe Haarberg
 * @brief Main source file for Bicycle Power Meter project for ME-507 Spring 2026 at Cal Poly SLO.
 * @version 0.1
 * @date 2026-05-20
 *
 * @copyright Copyright (c) 2026
 *
 *
 * Written to be run on custom nRF52832-based PCB.
 */

/** BEGIN INCLUDES **/
// System
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

// Peripherals
#include "app_ipc.h"
#include "max17048.h"
#include "imu_task.h"

// #include <math.h>
/** END INCLUDES **/

/** BEGIN PERIPHERAL CONFIGURATION **/

#if !DT_NODE_EXISTS(MAX17048_NODE)
#error "No MAX17048 battery monitor node found in devicetree"
#endif
batt_mon_t batt_mon = I2C_DT_SPEC_GET(MAX17048_NODE);

/** END PERIPHERAL CONFIGURATION */

#define DPS_TO_RPM(x) (x * 0.1666)

/** Logger configuration **/
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/**
 * @brief RTOS Task for battery monitoring
 *
 */
void battery_monitor_task(void)
{

    if (battery_monitor_init(&batt_mon)) // return 0 when properly configured
    {
        return;
    }

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

/**
 * @brief RTOS Task for sending messages to RTT terminal
 *
 * Messages are sent as a char array pointer to the FIFO buffer
 */
void rtt_task(void)
{
    while (1)
    {
        char *rx_data = k_fifo_get(&printk_fifo, K_FOREVER);

        puts(rx_data);
        k_free(rx_data);
    }
}

/** Thread creation **/
K_THREAD_DEFINE(battery_task_id, STACKSIZE, battery_monitor_task, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(rtt_task_id, STACKSIZE, rtt_task, NULL, NULL, NULL, 7, 0, 0);

/** Register ADC axis callback **/