#include "app_ipc.h"

K_EVENT_DEFINE(thread_sync_event);

K_FIFO_DEFINE(axis_fifo);

K_FIFO_DEFINE(imu_fifo);

K_FIFO_DEFINE(printk_fifo);

K_FIFO_DEFINE(ble_fifo);