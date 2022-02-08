#ifndef __STEPPER_H
#define __STEPPER_H

#include "main.h"
#include <stdio.h>
#include <math.h>
#include "tmc2130.h"

#define STEPS_PER_ROUND 400 * 4
#define TIMER_FREQUENCY (SystemCoreClock / 50)

// constant calculations
#define MOTOR_RES   20
#define CLK_US      0.01041
#define MOTOR_COUNT (20 / (CLK_US * MOTOR_RES))

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
    tmc2130 Driver;
    uint16_t ConstAcc;
    GPIO_TypeDef *Step_Port;
    uint16_t Step_Pin;
    
    GPIO_TypeDef *Enable_Port;
    uint16_t Enable_Pin;

    TIM_HandleTypeDef *StepTimer;
    uint8_t Forward;
    uint8_t Backward;
} Stepper_InitTypeDef;

typedef struct {
    GPIO_TypeDef *Dir_Port;
    uint16_t Dir_Pin;
    Stepper_InitTypeDef Init;
    // directionparams
    volatile Direction dir;
    volatile Direction target_dir;
    // calc variables
    uint32_t step_delay;
    uint32_t target_delay;
    // step counting
    int step_count;
    uint8_t int_status;

} Stepper_HandleTypeDef;

// Init stepper parameters etc
void init_stepper(Stepper_HandleTypeDef *step, TIM_HandleTypeDef *timer,
    SPI_HandleTypeDef *spi,
    GPIO_TypeDef *step_port, uint16_t step_pin,
    GPIO_TypeDef *dir_port, uint16_t dir_pin,
    GPIO_TypeDef *enable_port, uint16_t enable_pin,
    GPIO_TypeDef *nss_port, uint16_t nss_pin);

// call from step timer interrupt
void update_stepper(Stepper_HandleTypeDef *step);

void stepper_setspeed(Stepper_HandleTypeDef *step, int16_t speed);

void enable_stepper(Stepper_HandleTypeDef *step);

void disable_stepper(Stepper_HandleTypeDef *step);
#endif