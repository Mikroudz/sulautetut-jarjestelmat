#include "stepper.h"

uint8_t init_stepper(Stepper_HandleTypeDef *step, TIM_HandleTypeDef *timer,
    SPI_HandleTypeDef *spi,
    GPIO_TypeDef *step_port, uint16_t step_pin,
    GPIO_TypeDef *dir_port, uint16_t dir_pin,
    GPIO_TypeDef *enable_port, uint16_t enable_pin,
    GPIO_TypeDef *nss_port, uint16_t nss_pin){
    
    assert_param(step && timer && spi);

    uint8_t ret = tmc2130_init(&step->Init.Driver, spi, enable_port, nss_port, enable_pin, nss_pin);

    if (ret != TMC2130_OK)
        return STEPPER_ERROR;

    step->Init.Step_Port = step_port;
    step->Init.Step_Pin = step_pin;
    step->Dir_Port = dir_port;
    step->Dir_Pin = dir_pin;
    step->Init.StepTimer = timer;
    step->Init.StepTimer->Instance->PSC = (uint16_t)50;
    step->dir = FORWARD;
    step->Init.StepTimer->Instance->ARR = 0xffff;
    step->step_delay = 0xffff;
    step->step_count = 0;
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
    return STEPPER_OK;
}

void disable_stepper(Stepper_HandleTypeDef *step){
    tmc2130_disable(&step->Init.Driver);
}

void enable_stepper(Stepper_HandleTypeDef *step){
    tmc2130_enable(&step->Init.Driver);
}

void stepper_setspeed(Stepper_HandleTypeDef *step, int16_t speed){
    uint16_t speed_target;
    // direction
    if(speed < 0){
        step->target_dir = BACKWARD;
        speed_target = (uint16_t)-speed;
        HAL_GPIO_WritePin(step->Dir_Port,step->Dir_Pin, GPIO_PIN_RESET);
    }else{
        speed_target = (uint16_t)speed;
        step->target_dir = FORWARD;
        HAL_GPIO_WritePin(step->Dir_Port,step->Dir_Pin, GPIO_PIN_SET);
    }
    // deadzone
    if (speed_target < 19){
        HAL_TIM_Base_Stop_IT(step->Init.StepTimer);
    }else{
        int step_speed = (60 * TIMER_FREQUENCY / STEPS_PER_ROUND) / speed_target - 1;
        step->step_delay = step_speed;
        HAL_TIM_Base_Start_IT(step->Init.StepTimer);
    }
}  

// call from timer interrupt handler
void update_stepper(Stepper_HandleTypeDef *step){
    // update timer if we have new value
    step->Init.StepTimer->Instance->ARR = (uint16_t)step->step_delay;
    // step step pin
    HAL_GPIO_TogglePin(step->Init.Step_Port, step->Init.Step_Pin);
    // count steps
    if(step->int_status)
        step->step_count += step->target_dir ? 1 : -1;
    step->int_status ^= 1;
}