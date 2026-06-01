#include "imu_task.h"

/** Logger configuration **/
LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

K_MEM_SLAB_DEFINE(imu_data_slab,
                  sizeof(struct imu_data_t),
                  IMU_DATA_SLAB_NUM_BLOCKS,
                  IMU_DATA_SLAB_ALIGNMENT);

/* Define global ICM device */
static icm_t icm = I2C_DT_SPEC_GET(ICM40609_NODE);

/**
 * @brief RTOS Task for the IMU
 *
 */
void imu_task(void)
{
    if (icm_init(&icm)) // return 0 when properly configured
    {
        return;
    }

    LOG_INF("IMU configured.");

    while (1)
    {
        static int ret;
        static float gx, gy, gz;
        static float ax, ay, az;

        ret = icm_read_gyro(&icm, &gx, &gy, &gz);
        ret = icm_read_accel(&icm, &ax, &ay, &az);

        if (ret)
        {
            LOG_WRN("Error reading gyro/accel values");
        }
        else
        {
            struct imu_data_t *data;

            if (k_mem_slab_alloc(&imu_data_slab, (void **)&data, K_MSEC(100)) == 0)
            {
                data->accel_x = ax;
                data->accel_y = ay;
                data->gyro_z = gz;
                data->ts = k_uptime_get();

                k_fifo_put(&imu_fifo, data);
            }
            else
            {
                LOG_WRN("Warning: Couldn't allocate space on slab");
            }
            // char *mem_ptr = k_malloc(100);
            // sprintf(mem_ptr, "Read values:  x: %0.2f, y: %0.2f, z: %0.2f dps | x: %0.2f, y: %0.2f, z: %0.2f g",
            //         (double)gx, (double)gy, (double)gz,
            //         (double)ax, (double)ay, (double)az);
            // k_fifo_put(&printk_fifo, mem_ptr);

            // LOG_INF("Read values:  x: %0.2f, y: %0.2f, z: %0.2f dps | x: %0.2f, y: %0.2f, z: %0.2f g", gx, gy, gz, ax, ay, az);
        }

        k_sleep(K_SECONDS(1));
    }
}

K_THREAD_DEFINE(imu_task_id, STACKSIZE, imu_task, NULL, NULL, NULL, 7, 0, 0);