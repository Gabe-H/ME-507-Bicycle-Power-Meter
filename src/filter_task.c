#include "filter_task.h"

/**
 * @brief Angular velocity and angle position processing and filtering
 *
 */
void filter_task(void)
{
    while (1)
    {
        struct imu_data_t *data = k_fifo_get(&imu_fifo, K_FOREVER);

        k_mem_slab_free(&imu_data_slab, (void *)data);

        float angle = 0;

        if (data->accel_x == 0)
        {
            angle = 1.5708;
        }
        else
        {
            angle = atanf(data->accel_y / data->accel_x);

            // angle += 1.5708; // Offset due to IMU placement

            // Depending on signage of x component, add 180deg offset
            // to ensure output angle goes from 0-360 (rather than 0-180)
            // if (data->accel_x < 0)
            //     angle += 3.1415;
        }

        char *mem_ptr = k_malloc(50);

        // sprintf(mem_ptr, "Angle: %f", angle);
        sprintf(mem_ptr, "X: %.2f, Y: %.2f", (double)data->accel_x, (double)data->accel_y);

        k_fifo_put(&printk_fifo, mem_ptr);
    }
}

K_THREAD_DEFINE(angle_task_id, STACKSIZE, filter_task, NULL, NULL, NULL, 6, 0, 0);