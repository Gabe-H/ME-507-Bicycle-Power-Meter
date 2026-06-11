#ifndef ADC_H
#define ADC_H

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/input/input.h>
#include "app_ipc.h"

static void input_evt_cb(struct input_event *evt, void *user_data);

#endif /* ADC_H */