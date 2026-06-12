/**
 * @file app_ipc.c
 * @brief Inter-process communication setup
 * @version 0.1
 * @date 2026-06-11
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "app_ipc.h"

K_EVENT_DEFINE(thread_sync_event);

K_FIFO_DEFINE(axis_fifo);

K_FIFO_DEFINE(imu_fifo);

K_FIFO_DEFINE(printk_fifo);

K_FIFO_DEFINE(filter_fifo);

K_FIFO_DEFINE(ble_fifo);

K_MUTEX_DEFINE(axis_latest_lock);
int32_t axis_latest_value = 0;
