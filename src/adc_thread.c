/**
 * @file adc_thread.c
 * @author Gabe Haarberg
 * @brief Thread and helpers for reading from strain gauge ADC
 * @version 0.1
 * @date 2026-05-31
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "adc_thread.h"

#include <stdio.h>

LOG_MODULE_REGISTER(adc, LOG_LEVEL_INF);
// LOG_MODULE_REGISTER(adc, LOG_LEVEL_DBG);

K_MEM_SLAB_DEFINE(axis_data_slab,
                  sizeof(struct axis_data),
                  AXIS_DATA_SLAB_NUM_BLOCKS,
                  AXIS_DATA_SLAB_ALIGNMENT);

// TODO: DELTE THIS THREAD. ALL INPUTS WILL BE HANDLED BY BUILT IN AXIS THREAD(s)
/**
 * @brief Zephyr thread for reading values from the ADC
 *
 * Registers the analog-axis callback, then continuously reads mapped axis
 * values from the axis_fifo queue and forwards them to RTT via printk_fifo.
 */
// static void adc_thread(void)
// {
//     // Post READY bit to sync event share
//     k_event_post(&thread_sync_event, ADC_THREAD_READY);

//     LOG_DBG("Thread ready");

//     // Continue to main loop after START bit received
//     k_event_wait(&thread_sync_event, START_BIT, false, K_FOREVER);

//     LOG_DBG("Thread started");

//     return; // (thread no longer used. TODO: delete)

//     /* Main loop: read axis values from callback queue and print */
//     // while (1)
//     // {
//     // /* Get axis data from FIFO with timeout to keep responsiveness */
//     // struct axis_data *axis_msg = k_fifo_get(&axis_fifo, K_FOREVER);
//     // if (axis_msg)
//     // {
//     //     char *mem_ptr = k_malloc(64);
//     //     if (mem_ptr)
//     //     {
//     //         sprintf(mem_ptr, "Axis ch%d: %d", axis_msg->channel, (int)axis_msg->value);
//     //         k_fifo_put(&printk_fifo, mem_ptr);
//     //     }
//     //     k_mem_slab_free(&axis_data_slab, (void *)axis_msg);
//     // }
//     // }
// }

/**
 * @brief Input event callback (called by the Zephyr input subsystem)
 *
 * Receives fully-processed events from the analog-axis driver.
 * The driver already handles all scaling, deadzone, and clamping via DTS.
 * We just queue the mapped value for printing.
 */
static void input_evt_cb(struct input_event *evt, void *user_data)
{
    // ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    /* Filter for Y axis absolute position events */
    if (evt->type == INPUT_EV_ABS && evt->code == INPUT_ABS_Y)
    {
        LOG_DBG("Y-Axis value received: %d", evt->value);

        // struct axis_data *axis_msg;
        // if (k_mem_slab_alloc(&axis_data_slab, (void **)&axis_msg, K_NO_WAIT) == 0)
        // {
        //     axis_msg->channel = 0;
        //     axis_msg->value = evt->value; /* Already scaled/clamped by driver */
        //     k_fifo_put(&axis_fifo, axis_msg);
        // }
        // else
        // {
        //     LOG_WRN("Failed to allocate memory for axis slab item");
        // }
        k_mutex_lock(&axis_latest_lock, K_FOREVER);
        axis_latest_value = evt->value;
        k_mutex_unlock(&axis_latest_lock);
    }
    else
    {
        LOG_DBG("Received event type: %d code: %d", evt->type, evt->code);
    }
}

// K_THREAD_DEFINE(adc_thread_id, STACKSIZE, adc_thread, NULL, NULL, NULL, 7, 0, 0);

INPUT_CALLBACK_DEFINE(
    // DEVICE_DT_GET(DT_NODELABEL(anin0)),
    NULL,
    input_evt_cb,
    NULL);