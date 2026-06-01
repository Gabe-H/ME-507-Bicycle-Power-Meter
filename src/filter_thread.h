#ifndef FILTER_THREAD_H
#define FILTER_THREAD_H

#include <zephyr/kernel.h>
#include "app_ipc.h"

#include <eekf/eekf.h>
#include <math.h>
#include <stdio.h>

static void filter_thread(void);

/** Variables for Kalman */
static eekf_value dT = 0.1;  // Time step duration
static eekf_value s_w = 0.2; // Process noise standard deviation
static eekf_value s_z = 10;  // Measurement noise standard deviation

static eekf_return transition(eekf_mat *xp, eekf_mat *Jf, eekf_mat const *x,
                              eekf_mat const *u, void *userData);

static eekf_return measurement(eekf_mat *zp, eekf_mat *Jh, eekf_mat const *x,
                               void *userData);

#endif /* FILTER_THREAD_H */