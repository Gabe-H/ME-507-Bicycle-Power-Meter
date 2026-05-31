/**
 * @file adc_task.c
 * @author Gabe Haarberg
 * @brief Task and helpers for reading from strain gauge ADC
 * @version 0.1
 * @date 2026-05-31
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "adc_task.h"

/**
 * @brief RTOS Task for reading values from the ADC
 *
 * Registers the analog-axis callback, then continuously reads mapped axis
 * values from the axis_fifo queue and forwards them to RTT via printk_fifo.
 */
void adc_task(void)
{
    /* Main loop: read axis values from callback queue and print */
    while (1)
    {
        /* Get axis data from FIFO with timeout to keep responsiveness */
        struct axis_data *axis_msg = k_fifo_get(&axis_fifo, K_FOREVER);
        if (axis_msg)
        {
            char *mem_ptr = k_malloc(64);
            if (mem_ptr)
            {
                sprintf(mem_ptr, "Axis ch%d: %d", axis_msg->channel, (int)axis_msg->value);
                k_fifo_put(&printk_fifo, mem_ptr);
            }
            k_free(axis_msg);
        }
    }
}

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
        struct axis_data *axis_msg = k_malloc(sizeof(struct axis_data));
        if (axis_msg)
        {
            axis_msg->channel = 0;
            axis_msg->value = evt->value; /* Already scaled/clamped by driver */
            k_fifo_put(&axis_fifo, axis_msg);
        }
    }
}

K_THREAD_DEFINE(adc_task_id, STACKSIZE, adc_task, NULL, NULL, NULL, 7, 0, 0);

INPUT_CALLBACK_DEFINE(
    // DEVICE_DT_GET(DT_NODELABEL(anin0)),
    NULL,
    input_evt_cb,
    NULL);