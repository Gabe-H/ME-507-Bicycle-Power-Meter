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

/** END INCLUDES **/

/** BEGIN PERIPHERAL CONFIGURATION **/

/** END PERIPHERAL CONFIGURATION */

#define DPS_TO_RPM(x) (x * 0.1666)

/** Logger configuration **/
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

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
K_THREAD_DEFINE(rtt_task_id, STACKSIZE, rtt_task, NULL, NULL, NULL, 7, 0, 0);

/** Register ADC axis callback **/