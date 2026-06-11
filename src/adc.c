/**
 * @file adc.c
 * @author Gabe Haarberg
 * @brief Thread and helpers for reading from strain gauge ADC
 * @version 0.1
 * @date 2026-05-31
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "adc.h"

#include <stdio.h>

LOG_MODULE_REGISTER(adc, LOG_LEVEL_INF);
// LOG_MODULE_REGISTER(adc, LOG_LEVEL_DBG);

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

        k_mutex_lock(&axis_latest_lock, K_FOREVER);
        axis_latest_value = evt->value;
        k_mutex_unlock(&axis_latest_lock);
    }
    else
    {
        LOG_DBG("Received event type: %d code: %d", evt->type, evt->code);
    }
}

INPUT_CALLBACK_DEFINE(
    NULL,
    input_evt_cb,
    NULL);