#include "power_meas.h"

/** Logger configuration **/
// LOG_MODULE_REGISTER(pwr, LOG_LEVEL_DBG);
LOG_MODULE_REGISTER(pwr, LOG_LEVEL_INF);

/**
 * @brief Thread that handles the syncronization of Kalman-filtered angle/angular velocity and torque from the input axis to
 * create the power estimate.
 *
 * Steps:
 *  1) Wait for data to come in from filter (processed IMU data - timing driven by IMU thread)
 *  2) Get the latest axis value, accessed safetly with a mux. The axis thread only updates values
 *     upon a _change_, so we can't rely on this value to have an update immediately upon request.
 *  3) Scale the axis value to actual torque estimate using calibration data.
 *  4) Calculate power using T*omega
 *  5) (TODO) short filter average (0.5-1.0s period)
 *  7) Calculate crank_event_time for CPS
 *  6) Pack result into slab and FIFO for BLE task to pick up.
 */
void power_measure_thread(void)
{
    /**
     * Check if there is new axis data incoming. Axis FIFO only gets updated when the strain measured _changes_
     * Note that the IMU thread is what drives update timing, which then gets processed by the filter thread.
     * Here, we wait for the filter thread, then either instanta
     */

    float omega;          // Filtered angular velocity [rad/s]
    float theta;          // Filtered angle position [rad]
    int64_t ts;           // Uptime at input capture [ms]
    int32_t raw_axis_val; // Raw axis value

    // Post READY bit to sync event share
    k_event_post(&thread_sync_event, PWR_THREAD_READY);

    // Continue to main loop after START bit received
    k_event_wait(&thread_sync_event, START_BIT, false, K_FOREVER);

    while (1)
    {

        /* Wait for filter data to come in (wait forever until next packet comes in) */
        struct filter_data_t *filter_data = k_fifo_get(&filter_fifo, K_FOREVER);
        if (filter_data)
        {
            // Copy data before slab is erased
            omega = filter_data->omega;
            theta = filter_data->theta;
            ts = filter_data->ts;

            // Free slab
            k_mem_slab_free(&filter_data_slab, (void *)filter_data);
        }
        else
            continue; // Skip processing if no filter data ready

        /* Get latest axis value */
        k_mutex_lock(&axis_latest_lock, K_FOREVER); // We can wait 'forever' here because the publisher (adc_thread) only takes the mutex for a couple of clock cycles
        raw_axis_val = axis_latest_value;
        k_mutex_unlock(&axis_latest_lock); // Release mut for adc thread

        /* Scale axis value based on calibration data */

        // float torque = ((float)raw_axis_val * AXIS_SLOPE) + AXIS_INTERCEPT;
        float torque = (float)raw_axis_val;

        LOG_DBG("T: %.3f, O: %.2f, Th: %.2f", (double)torque, (double)omega, (double)theta);

        /* Pack result into slab*/
    }
}

K_THREAD_DEFINE(power_measure_thread_id, STACKSIZE, power_measure_thread, NULL, NULL, NULL, 5, 0, 0);