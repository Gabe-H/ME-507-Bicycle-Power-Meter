#include "app_ipc.h"

K_FIFO_DEFINE(axis_fifo);

K_FIFO_DEFINE(imu_fifo);

K_FIFO_DEFINE(printk_fifo);

K_MEM_SLAB_DEFINE(imu_data_slab,
                  sizeof(struct imu_data_t),
                  IMU_DATA_SLAB_NUM_BLOCKS,
                  IMU_DATA_SLAB_ALIGNMENT);