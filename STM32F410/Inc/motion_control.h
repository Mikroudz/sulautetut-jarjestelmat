#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#include "main.h"
#include <stdio.h>
#include "bmx160.h"
#include <math.h>

#define KP 0.5
#define KI 0.5
#define KD 0.5

// Set pid loop etc whatever needed
void init_motion();

// Update pid with new values
// Probably stepper speed target, robot angle etc.. as a parameter
void update_motion_loop(uint16_t lm_speed);

// calc pitch from imu and gyro data with complementary filter
float complementary_filter(data_t *acc, data_t *gyro, float pitch);

#endif