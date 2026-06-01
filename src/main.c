/**
 * @file main.c
 * @author Gabe Haarberg
 * @brief Bicycle Power Meter project for ME-507 Spring 2026 at Cal Poly SLO.
 * @version 0.1
 * @date 2026-05-20
 *
 * @copyright Copyright (c) 2026
 *
 *
 * Written to be run on custom nRF52832-based PCB. Individual tasks can be found in their src/ *_task.c files
 */

/** BEGIN INCLUDES **/
// System
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app_ipc.h"

/** END INCLUDES **/

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

K_THREAD_DEFINE(rtt_task_id, STACKSIZE, rtt_task, NULL, NULL, NULL, 7, 0, 0);