#include "imu_thread.h"

/** Logger configuration **/
LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

K_MEM_SLAB_DEFINE(imu_data_slab,
                  sizeof(struct imu_data_t),
                  IMU_DATA_SLAB_NUM_BLOCKS,
                  IMU_DATA_SLAB_ALIGNMENT);

/* Define global ICM device */
static icm_t icm = I2C_DT_SPEC_GET(ICM40609_NODE);

/**
 * @brief Zephyr thread for the IMU
 *
 */
static void imu_thread(void)
{
    if (icm_init(&icm)) // return 0 when properly configured
    {
        // For now, just say task is ready even if it fails...
        k_event_post(&thread_sync_event, IMU_THREAD_READY);
        return;
    }

    LOG_INF("IMU configured.");

    // Post READY bit to sync event share
    k_event_post(&thread_sync_event, IMU_THREAD_READY);

    LOG_DBG("Thread ready");

    // Continue to main loop after START bit received
    k_event_wait(&thread_sync_event, START_BIT, false, K_FOREVER);

    LOG_DBG("Thread started");

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

            // Upon successful data read, send to memory slab (for filter to pick up)
            struct imu_data_t *data;

            if (k_mem_slab_alloc(&imu_data_slab, (void **)&data, K_MSEC(100)) == 0)
            {
                data->gyro_x = gx;
                data->gyro_y = gy;
                data->gyro_z = gz;
                data->accel_x = ax;
                data->accel_y = ay;
                data->accel_z = az;
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

        k_sleep(K_MSEC(IMU_PERIOD));
    }
}

K_THREAD_DEFINE(imu_thread_id, 512, imu_thread, NULL, NULL, NULL, 5, 0, 0); // Highest priority in power calc chain - dictates timing