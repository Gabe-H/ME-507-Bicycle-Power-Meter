#include "app_ipc.h"

K_EVENT_DEFINE(thread_sync_event);

K_FIFO_DEFINE(axis_fifo);

K_FIFO_DEFINE(imu_fifo);

K_FIFO_DEFINE(printk_fifo);

K_FIFO_DEFINE(filter_fifo);

K_FIFO_DEFINE(ble_fifo);

K_MUTEX_DEFINE(axis_latest_lock);
int32_t axis_latest_value = 0;

// K_SEM_DEFINE(
//     axis_latest_ready,
//     0,
//     1);