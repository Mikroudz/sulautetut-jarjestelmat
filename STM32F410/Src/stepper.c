#include "stepper.h"

void init_stepper(Stepper_HandleTypeDef *step, TIM_HandleTypeDef *timer,
    GPIO_TypeDef *step_port, uint16_t step_pin,
    GPIO_TypeDef *dir_port, uint16_t dir_pin,
    uint8_t side){
    
    assert_param(step && timer);
    step->Init.ConstAcc = 20000;

    step->Init.Forward = side ? GPIO_PIN_SET : GPIO_PIN_RESET;
    step->Init.Backward = side ? GPIO_PIN_RESET : GPIO_PIN_SET;
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
    HAL_GPIO_WritePin(dir_port,dir_pin, step->Init.Forward);
}

void stepper_setangle(Stepper_HandleTypeDef *step, int16_t speed){
    //HAL_TIM_Base_Stop_IT(step->Init.StepTimer);
    uint16_t speed_target;

    if(speed < 0){
        step->target_dir = BACKWARD;
        speed_target = (uint16_t)-speed;
        HAL_GPIO_WritePin(step->Dir_Port,step->Dir_Pin, GPIO_PIN_RESET);
    }else{
        speed_target = (uint16_t)speed;
        step->target_dir = FORWARD;
        HAL_GPIO_WritePin(step->Dir_Port,step->Dir_Pin, GPIO_PIN_SET);
    }

    if (speed_target < 19){
        HAL_TIM_Base_Stop_IT(step->Init.StepTimer);
    }else{
        int step_speed = (60 * TIMER_FREQUENCY / STEPS_PER_ROUND) / speed_target - 1;
        step->step_delay = step_speed;
        HAL_TIM_Base_Start_IT(step->Init.StepTimer);
    }
}  

void step_stepper(Stepper_HandleTypeDef *step){
    HAL_GPIO_TogglePin(step->Init.Step_Port, step->Init.Step_Pin);

}

void update_stepper(Stepper_HandleTypeDef *step){

    step->Init.StepTimer->Instance->ARR = (uint16_t)step->step_delay;

    HAL_GPIO_TogglePin(step->Init.Step_Port, step->Init.Step_Pin);

    if(step->int_status)
        step->step_count += step->target_dir ? 1 : -1;
    step->int_status ^= 1;
}