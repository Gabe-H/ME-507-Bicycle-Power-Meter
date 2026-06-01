#include "filter_thread.h"

/**
 * @brief Angular velocity and angle position processing and filtering
 *
 */
void filter_thread(void)
{
    // Post READY bit to sync event share
    k_event_post(&thread_sync_event, FILTER_THREAD_READY);

    // Continue to main loop after START bit received
    k_event_wait(&thread_sync_event, START_BIT, false, K_FOREVER);

    while (1)
    {
        struct imu_data_t *data = k_fifo_get(&imu_fifo, K_FOREVER);
        float ax = data->accel_x;
        float ay = data->accel_y;

        k_mem_slab_free(&imu_data_slab, (void *)data);

        float angle = 0;
        int state = 0;

        // Check if atan denominator is 0 first
        if (ax == 0.0F)
        {
            if (ay < 0.0F)
                angle = 4.7124;
            else
                angle = 1.5708;
        }
        else
        {
            angle = atanf(ay / ax);

            // Arctan truth table.
            // Theta=0 when IMU x axis is facing upwards
            if (ax > 0.0F && ay > 0.0F)
            {
                state = 1;
                angle = 3.1415F - angle;
            }

            else if (ax > 0.0F && ay <= 0.0F)
            {
                state = 2;
                angle = 3.1415F - angle;
            }
            else if (ax < 0.0F && ay >= 0.0F)
            {
                state = 3;
                angle = -angle;
            }
            else if (ax < 0.0F && ay < 0.0F)
            {
                angle = 6.2831F - angle;
            }

            // angle += 1.5708; // Offset due to IMU placement
        }

        char *mem_ptr = k_malloc(50);

        // sprintf(mem_ptr, "Angle: %f", angle);
        sprintf(mem_ptr, "X: %.2f, Y: %.2f, theta: %.2f, [%d]", (double)data->accel_x, (double)data->accel_y, (double)angle, state);

        k_fifo_put(&printk_fifo, mem_ptr);
    }
}

K_THREAD_DEFINE(filter_thread_id, STACKSIZE, filter_thread, NULL, NULL, NULL, 6, 0, 0);