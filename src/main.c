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
 * Written to be run on custom nRF52832-based PCB. Individual thread functions can be found in their src/ *_thread.c files
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
 * @brief Zephyr thread for sending messages to RTT terminal
 *
 * Messages are sent as a char array pointer to the FIFO buffer
 */
void rtt_thread(void)
{
    // Post READY bit to sync event share
    k_event_post(&thread_sync_event, RTT_THREAD_READY);

    // Continue to main loop after START bit received
    k_event_wait(&thread_sync_event, START_BIT, false, K_FOREVER);
    while (1)
    {
        char *rx_data = k_fifo_get(&printk_fifo, K_FOREVER);

        puts(rx_data);
        k_free(rx_data);
    }
}

/**
 * @brief Main thread. Coordinates all thread setup before allowing them to enter their loops
 *
 * See coordination configuration in app_ipc.h
 *
 */
int main(void)
{
    // Wait for all READY bit flags to be set
    // k_event_wait_all(&thread_sync_event, ALL_READY, false, K_FOREVER);
    uint32_t events = k_event_wait_all(
        &thread_sync_event,
        ALL_READY,
        false,
        K_SECONDS(5));

    // Log error if not all threads started in time
    if ((events & ALL_READY) != ALL_READY)
    {
        LOG_ERR("Not all threads became ready. Got 0x%08X", events);
    }

    // Notify tasks to continue by setting START bit
    k_event_post(&thread_sync_event, START_BIT);

    LOG_DBG("Start bit sent to all threads");

    return 0; // Main thread no longer needed
}

K_THREAD_DEFINE(rtt_thread_id, STACKSIZE, rtt_thread, NULL, NULL, NULL, 7, 0, 0);