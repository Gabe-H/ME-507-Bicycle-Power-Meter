#ifndef BATT_MON_H
#define BATT_MON_H

#include <stdio.h>
#include <zephyr/kernel.h>
#include "max17048.h"
#include "app_ipc.h"

#if !DT_NODE_EXISTS(MAX17048_NODE)
#error "No MAX17048 battery monitor node found in devicetree"
#endif

static void battery_monitor_task(void);

#endif /* BATT_MON_H */