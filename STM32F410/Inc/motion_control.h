#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#include "main.h"
#include <stdio.h>
#include "bmx160.h"
#include <math.h>
#include "stepper.h"

#define FACTOR_REAL 1.0 
#define Q_NOISE 1.0
#define H_MEAS_REAL 1.0
#define ENV_NOISE 10.

typedef struct {
    // tuning parameters
    float KP;
    float KI;
    float KD;
    float max;
    float min;
    // internal memory
    float last;
    float out_sum;
    // input values
    float new;
    float target;
} PID_TypeDef;

typedef enum {
  MOVE_STOP = 0,  
  MOVE_FORWARD,
  MOVE_REVERSE,
  TURN_LEFT,
  TURN_RIGHT
} MoveDirection_t;

typedef enum {
    BL_IDLE = 0,
    BL_CALIB,
    BL_RUN,
    BL_ERROR,
    BL_START,
    BL_STOP
} BalanceStatus_t;

typedef enum {
    APP_READY = 0,
    APP_RUN,
    APP_STOP,
    LOW_BATTERY,
    INIT_FAIL
} MainState_t;

typedef struct {
    PID_TypeDef velocityPID;
    PID_TypeDef anglePID;
    int left_motor_offset;
    int right_motor_offset;
    float robot_angle;
    // program state
    MainState_t app_state;
    // Balance loop run status
    BalanceStatus_t state;
    // Robot moving direction
    MoveDirection_t direction;
    uint16_t setup;
    // stepper stuff
    Stepper_HandleTypeDef *step1;
    Stepper_HandleTypeDef *step2;
    uint32_t step_count;
    uint32_t loop_iter;

} Motion_TypeDef;


void motion_init(Motion_TypeDef *mot);
void motion_loop(Motion_TypeDef *mot);

float kalmanfilter(float input);

// calc pitch from imu and gyro data with complementary filter
float complementary_filter(data_t *acc, data_t *gyro, float pitch);

int pid(PID_TypeDef *input);
void pid_reset(PID_TypeDef *input);
void print_pid(PID_TypeDef *input);
float pid_simple(PID_TypeDef *pid);
float pid_steps(PID_TypeDef *input);

#endif