#include "power_meas.h"

/** Logger configuration **/
LOG_MODULE_REGISTER(pwr, LOG_LEVEL_DBG);
// LOG_MODULE_REGISTER(pwr, LOG_LEVEL_INF);

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

    float omega;               // Filtered angular velocity [rad/s]
    float theta;               // Filtered angle position [rad]
    uint16_t crank_index;      // Index of crank rotation (See `bt_cps_notify`)
    uint16_t crank_event_time; // Timing of crank rotation (See `bt_cps_notify`)
    int64_t ts;                // Uptime at input capture [ms]
    int32_t raw_axis_val;      // Raw axis value

    float omega_samples[NUM_SAMPLES] = {0};
    float theta_samples[NUM_SAMPLES] = {0};
    float torque_samples[NUM_SAMPLES] = {0};
    float omega_sum = 0.0f;
    float theta_sum = 0.0f;
    float torque_sum = 0.0f;
    uint32_t sample_count = 0;
    uint32_t sample_index = 0;
    uint32_t sample_total = 0;

    // Post READY bit to sync event share
    k_event_post(&thread_sync_event, PWR_THREAD_READY);
    LOG_DBG("Thread ready");

    // Continue to main loop after START bit received
    k_event_wait(&thread_sync_event, START_BIT, false, K_FOREVER);

    LOG_DBG("Starting thread");

    while (1)
    {
        /* Wait for filter data to come in (wait forever until next packet comes in) */
        struct filter_data_t *filter_data = k_fifo_get(&filter_fifo, K_FOREVER);
        if (filter_data)
        {
            // Copy data before slab is erased
            omega = filter_data->omega;
            theta = filter_data->theta;
            crank_index = filter_data->crank_index;
            crank_event_time = filter_data->crank_event_time;
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

        float torque = ((float)raw_axis_val * (float)AXIS_SLOPE) + (float)AXIS_INTERCEPT;
        // float torque = (float)raw_axis_val;

        omega_sum -= omega_samples[sample_index];
        theta_sum -= theta_samples[sample_index];
        torque_sum -= torque_samples[sample_index];

        omega_samples[sample_index] = omega;
        theta_samples[sample_index] = theta;
        torque_samples[sample_index] = torque;

        omega_sum += omega;
        theta_sum += theta;
        torque_sum += torque;

        sample_index = (sample_index + 1U) % NUM_SAMPLES;
        if (sample_count < NUM_SAMPLES)
        {
            sample_count++;
        }
        sample_total++;

        float omega_avg = omega_sum / (float)sample_count;
        float theta_avg = theta_sum / (float)sample_count;
        float torque_avg = torque_sum / (float)sample_count;

        if (sample_count == NUM_SAMPLES && (sample_total % NUM_SAMPLES) == 0U)
        {
            uint16_t power_avg = (uint16_t)(torque_avg * omega_avg);

            LOG_DBG("avg P:%u, T:%.3f, O:%.2f, T:%.2f | %u, %u",
                    power_avg,
                    (double)torque_avg,
                    (double)omega_avg,
                    (double)theta_avg,
                    crank_index,
                    crank_event_time);

            struct ble_data_t *data_out;

            if (k_mem_slab_alloc(&ble_data_slab, (void **)&data_out, K_MSEC(10)) == 0)
            {
                data_out->power = power_avg;
                data_out->crank_index = crank_index;
                data_out->crank_event_time = crank_event_time;

                k_fifo_put(&ble_fifo, data_out);
            }
            else
            {
                // LOG_WRN("Warning: Couldn't allocate space on slab");
            }
        }

        /* Pack result into slab*/
    }
}

K_THREAD_DEFINE(power_measure_thread_id, 768, power_measure_thread, NULL, NULL, NULL, 5, 0, 0);