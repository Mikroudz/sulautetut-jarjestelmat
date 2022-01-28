#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#include "main.h"
#include <stdio.h>
#include "bmx160.h"
#include <math.h>

#define FACTOR_REAL 1.0 
#define Q_NOISE 1.0
#define H_MEAS_REAL 1.0
#define ENV_NOISE 10.

typedef struct {
    // constant tuning
    float KP;
    float KI;
    float KD;
    const float max;
    const float min;
    // internal memory
    float last;
    float out_sum;
    // input values
    float new;
    float target;
} PID_TypeDef;


float kalmanfilter(float input);

// Update pid with new values
// Probably stepper speed target, robot angle etc.. as a parameter
void update_motion_loop(uint16_t lm_speed);

// calc pitch from imu and gyro data with complementary filter
float complementary_filter(data_t *acc, data_t *gyro, float pitch);

int pid(PID_TypeDef *input);
void pid_reset(PID_TypeDef *input);
void print_pid(PID_TypeDef *input);
float pid_simple(PID_TypeDef *pid);
float pid_steps(PID_TypeDef *input);


#endif