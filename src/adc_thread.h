#ifndef ADC_THREAD_H
#define ADC_THREAD_H

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/input/input.h>
#include <stdio.h>
#include "app_ipc.h"

static void adc_thread(void);

static void input_evt_cb(struct input_event *evt, void *user_data);

#endif /* ADC_THREAD_H */