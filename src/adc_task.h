#ifndef ADC_TASK_H
#define ADC_TASK_H

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/input/input.h>
#include <stdio.h>
// #include <zephyr/device.h>
// #include <zephyr/devicetree.h>
#include "app_ipc.h"

static void adc_task(void);

static void input_evt_cb(struct input_event *evt, void *user_data);

#endif /* ADC_TASK_H */