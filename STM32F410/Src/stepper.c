#include "stepper.h"

void init_stepper(Stepper_HandleTypeDef *step, TIM_HandleTypeDef *timer,
    GPIO_TypeDef *step_port, uint16_t step_pin,
    GPIO_TypeDef *dir_port, uint16_t dir_pin){
    
    assert_param(step && timer);
    step->Init.ConstAcc = 15000;
    step->Init.Speed = 5000;

    step->Init.Step_Port = step_port;
    step->Init.Step_Pin = step_pin;
    step->Init.Dir_Port = dir_port;
    step->Init.Dir_Pin = dir_pin;
    step->Init.StepTimer = timer;
    step->Init.StepTimer->Instance->PSC = (uint16_t)20 - 1;

}

uint16_t stepper_getangle(Stepper_HandleTypeDef *step){
    return 1;//step->current_angle;
}

void stepper_setangle(Stepper_HandleTypeDef *step, int16_t steps){

    step->min_delay = A_T_x100 / step->Init.Speed;

    if(steps < 0){
        step->dir = BACKWARD;
        steps = -steps;
    }else{
        step->dir = FORWARD;
    }
    HAL_GPIO_WritePin(step->Init.Dir_Port,step->Init.Dir_Pin, step->dir);

    // FYI nää on aika nopeita laskuja kun prossu on niin nopea joten ihan sma mitä tässä tehään
    // En huomannut ongelmia performanssissa
    // Luultavasti kiihtyvyyttä ei tarvitse muuttaa normaalisti joten tunge 
    // tuo juuren lasku staattiseksi
    step->step_delay = (T_FREQ_148 * sqrt(A_SQ / step->Init.ConstAcc)) / 100;

    // limiittikin voidaan laskea etukäteen
    uint16_t max_speed_steps = (step->Init.Speed * step->Init.Speed) / ((A_x20000 * step->Init.ConstAcc) / 100); 

    uint16_t accel_limit = (steps * step->Init.ConstAcc) / (step->Init.ConstAcc + step->Init.ConstAcc);

    if (accel_limit == 0)
        accel_limit = 1;

    // laske kumpi tulee vastaan ensin; max nopeus vai steppien määrä?
    if (accel_limit <= max_speed_steps)
        step->decel_val = accel_limit - steps;
    else 
        step->decel_val = -(max_speed_steps * step->Init.ConstAcc) / step->Init.ConstAcc;

    if(step->state == STOP)
        step->Init.StepTimer->Instance->ARR = 10000;

    if (step->step_delay <= step->min_delay){
        step->step_delay = step->min_delay;
        step->state = RUN;
    }else
        step->state = ACCEL;

    step->decel_start = steps + step->decel_val;

    //printf("step delay: %d decel val: %d decel start: %d\n", step->step_delay, step->decel_val, step->decel_start);
    //printf("accel limit: %d max s limit: %d t freq: %ld\n", accel_limit, max_speed_steps, TIMER_FREQUENCY);
    HAL_TIM_Base_Start_IT(step->Init.StepTimer);
}  

void update_stepper(Stepper_HandleTypeDef *step){

    uint32_t new_step_delay;
    step->Init.StepTimer->Instance->ARR = (uint32_t)step->step_delay;

    // state machine for stepper (stop -> accel | run -> run -> decel -> stop)
    switch(step->state){
        case STOP:
            HAL_TIM_Base_Stop_IT(step->Init.StepTimer);
            step->rest = 0;
            step->step_count = 0;
            step->accel_count = 0;
        break;
        case RUN:
            HAL_GPIO_TogglePin(step->Init.Step_Port, step->Init.Step_Pin);
            step->step_count++;
            new_step_delay = step->min_delay;
            if (step->step_count >= step->decel_start){
                step->accel_count = step->decel_val;
                new_step_delay = step->last_accel_delay;

                step->state = DECEL;
            }
        break;
        case ACCEL:
            HAL_GPIO_TogglePin(step->Init.Step_Port, step->Init.Step_Pin);
            step->step_count++;
            step->accel_count++;
            new_step_delay = step->step_delay - (((2 * step->step_delay) + step->rest) / (4 * step->accel_count + 1));
            step->rest = ((2 * step->step_delay) + step->rest) % (4 * step->accel_count + 1);
            if (step->step_count >= step->decel_start){ // tarkistetaan hidastus
                step->accel_count = step->decel_val;
                step->state = DECEL;
            }else if (new_step_delay <= step->min_delay){ // siirrtyään ajoon staattisella nopeudella
                step->last_accel_delay = new_step_delay;
                new_step_delay = step->min_delay;
                step->rest = 0;
                step->state = RUN;
            }
        break;
        case DECEL:
            HAL_GPIO_TogglePin(step->Init.Step_Port, step->Init.Step_Pin);
            step->step_count++;
            step->accel_count++;
            new_step_delay = step->step_delay - ((int32_t)((2 * step->step_delay) + step->rest) / (int32_t)(4 * step->accel_count + 1));
            step->rest = (int32_t)((2 * step->step_delay) + step->rest) % (int32_t)(4 * step->accel_count + 1);
            if (step->accel_count >= 0)
                step->state = STOP;
        break;
    }
    step->step_delay = new_step_delay;
}