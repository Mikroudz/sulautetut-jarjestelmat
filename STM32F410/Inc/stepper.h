#ifndef __STEPPER_H
#define __STEPPER_H

#include "main.h"
#include <stdio.h>
#include <math.h>

#define STEPS_PER_ROUND 400
#define TIMER_FREQUENCY (SystemCoreClock / 19)

// constant calculations
#define ALPHA       (2 * M_PI / STEPS_PER_ROUND)
#define A_T_x100    (ALPHA * TIMER_FREQUENCY * 100)
#define T_FREQ_148  ((TIMER_FREQUENCY * 0.676) / 100)
#define A_SQ        (ALPHA * 2 * 10000000000)
#define A_x20000    (ALPHA * 20000)

typedef enum {
    STOP = 0,
    RUN,
    ACCEL,
    DECEL
} RunState;

typedef enum {
    FORWARD = 0,
    BACKWARD = 1
} Direction;

typedef struct {
    uint16_t ConstAcc;
    uint16_t Speed;
    GPIO_TypeDef *Step_Port;
    uint16_t Step_Pin;
    GPIO_TypeDef *Dir_Port;
    uint16_t Dir_Pin;
    TIM_HandleTypeDef *StepTimer;
    uint8_t Forward;
    uint8_t Backward;
} Stepper_InitTypeDef;

typedef struct {
    Stepper_InitTypeDef Init;
    //StepDir direction;
    RunState state;
    volatile Direction dir;
    volatile Direction target_dir;
    // calc variables
    uint16_t min_delay;
    uint32_t step_delay;
    int16_t decel_val;
    int16_t accel_count;
    uint16_t decel_start;
    uint16_t current_speed;
    uint16_t last_acc_steps;
    // int var
    int16_t last_accel_delay;
    uint16_t step_count;
    uint16_t rest;

} Stepper_HandleTypeDef;

// Init stepper parameters etc
void init_stepper(Stepper_HandleTypeDef *step, TIM_HandleTypeDef *timer,
    GPIO_TypeDef *step_port, uint16_t step_pin,
    GPIO_TypeDef *dir_port, uint16_t dir_pin,
    uint8_t side);

// call from step timer interrupt
void update_stepper(Stepper_HandleTypeDef *step);

//void stepper_check_directionchange(Stepper_HandleTypeDef *step);
void stepper_setangle(Stepper_HandleTypeDef *step, int16_t speed);

#endif