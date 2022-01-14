#include "stepper.h"

void init_stepper(Stepper_HandleTypeDef *step, TIM_HandleTypeDef *timer,
    GPIO_TypeDef *step_port, uint16_t step_pin,
    GPIO_TypeDef *dir_port, uint16_t dir_pin,
    uint8_t side){
    
    assert_param(step && timer);
    step->Init.ConstAcc = 20000;
    step->Init.Speed = 5000;

    step->Init.Forward = side ? GPIO_PIN_SET : GPIO_PIN_RESET;
    step->Init.Backward = side ? GPIO_PIN_RESET : GPIO_PIN_SET;
    step->Init.Step_Port = step_port;
    step->Init.Step_Pin = step_pin;
    step->Init.Dir_Port = dir_port;
    step->Init.Dir_Pin = dir_pin;
    step->Init.StepTimer = timer;
    step->Init.StepTimer->Instance->PSC = (uint16_t)20 - 1;
    step->dir = BACKWARD;
    //HAL_GPIO_WritePin(dir_port,dir_pin, GPIO_PIN_SET);
}

void stepper_setangle(Stepper_HandleTypeDef *step, int16_t speed){
    //HAL_TIM_Base_Stop_IT(step->Init.StepTimer);
    if(speed < 0){
        step->target_dir = BACKWARD;
        speed = -speed;
        HAL_GPIO_WritePin(step->Init.Dir_Port,step->Init.Dir_Pin, GPIO_PIN_RESET);
    }else{
        step->target_dir = FORWARD;
        HAL_GPIO_WritePin(step->Init.Dir_Port,step->Init.Dir_Pin, GPIO_PIN_SET);
    }
    

    step->min_delay = A_T_x100 / speed;
    step->step_delay = A_T_x100 / speed;


    // FYI nää on aika nopeita laskuja kun prossu on niin nopea joten ihan sma mitä tässä tehään
    // En huomannut ongelmia performanssissa
    // Luultavasti kiihtyvyyttä ei tarvitse muuttaa normaalisti joten tunge 
    // tuo juuren lasku staattiseksi
    uint16_t step_delay = (T_FREQ_148 * sqrt(A_SQ / step->Init.ConstAcc)) / 100;

    // Monta steppiä edelliseen nopeuteen
    uint16_t max_speed_steps = (step->current_speed * step->current_speed) / ((A_x20000 * step->Init.ConstAcc) / 100); 
    // monta steppiä hidastuksen aloittamiseen EI KÄYTÖSSÄ
    //uint16_t accel_limit = (steps * step->Init.ConstAcc) / (step->Init.ConstAcc + step->Init.ConstAcc);

    if(step->state == STOP){
        step->Init.StepTimer->Instance->ARR = 10000;
        step->dir = step->target_dir;
        HAL_GPIO_WritePin(step->Init.Dir_Port,step->Init.Dir_Pin, step->dir);
        if(step_delay <= step->min_delay){
            step->step_delay = step->min_delay;
            step->state = RUN;
        }else{
            step->state = ACCEL;
            step->step_delay = step_delay;
        }
    }else if(step->target_dir != step->dir){ // vaihdetaan suunta
        step->state = DECEL;
        // monta steppiä hidastetaan
        step->accel_count = -step->last_acc_steps;
        // DECEL interrupti hoitaa loput
    }else if(step->step_delay > step->min_delay){ // uusi nopeus hitaampi
        step->state = DECEL;
        //int16_t decel_steps = -(max_speed_steps * step->Init.ConstAcc) / step->Init.ConstAcc;

        step->accel_count = -(max_speed_steps - (step->last_acc_steps - 1));
        step->last_acc_steps = step->accel_count;
    }else if(step->step_delay < step->min_delay){ // uusi on nopeampi
        step->accel_count = step->last_acc_steps;
        step->state = ACCEL;
    }

    step->current_speed = speed;
    //printf("new step delay: %d step_delay: %d mind delay: %d\n", step_delay, step->step_delay, step->min_delay);
    //printf("state: %d dir: %d target dir: %d \n",  step->state, step->dir, step->target_dir);
    HAL_TIM_Base_Start_IT(step->Init.StepTimer);
}  

void update_stepper(Stepper_HandleTypeDef *step){

    uint32_t new_step_delay;
    step->Init.StepTimer->Instance->ARR = (uint32_t)step->step_delay;

    HAL_GPIO_TogglePin(step->Init.Step_Port, step->Init.Step_Pin);
    // state machine for stepper (stop -> accel | run -> run -> decel -> stop)
    /*switch(step->state){
        case STOP:
            HAL_TIM_Base_Stop_IT(step->Init.StepTimer);
            step->rest = 0;
            step->step_count = 0;
            step->accel_count = 0;
        break;
        case RUN: // Runissa ei tehdä muuta kun luikautellaan tuota pinniä
            // Poistuminen tapahtuu vain uuden nopeusarvon asettamalla
            HAL_GPIO_TogglePin(step->Init.Step_Port, step->Init.Step_Pin);
            new_step_delay = step->min_delay;

        break;
        case ACCEL:
            HAL_GPIO_TogglePin(step->Init.Step_Port, step->Init.Step_Pin);
            step->step_count++;
            step->accel_count++;
            new_step_delay = step->step_delay - (((2 * step->step_delay) + step->rest) / (4 * step->accel_count + 1));
            step->rest = ((2 * step->step_delay) + step->rest) % (4 * step->accel_count + 1);
            if (new_step_delay <= step->min_delay){ // siirrtyään ajoon staattisella nopeudella
                //step->last_accel_delay = new_step_delay;
                step->last_acc_steps = step->accel_count;
                new_step_delay = step->min_delay;
                step->rest = 0;
                step->state = RUN;
                printf("accel count: %d\n", step->accel_count);
            }
        break;
        case DECEL:
            HAL_GPIO_TogglePin(step->Init.Step_Port, step->Init.Step_Pin);
            step->step_count++;
            step->accel_count++;
            new_step_delay = step->step_delay - ((int32_t)((2 * step->step_delay) + step->rest) / (int32_t)(4 * step->accel_count + 1));
            step->rest = (int32_t)((2 * step->step_delay) + step->rest) % (int32_t)(4 * step->accel_count + 1);
            if(step->accel_count >= 0 && step->dir != step->target_dir){
                step->dir = step->target_dir;
                step->state = ACCEL;
                step->accel_count = 0;
                new_step_delay = 10000;
                HAL_GPIO_WritePin(step->Init.Dir_Port,step->Init.Dir_Pin, step->target_dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
                //printf("dir change\n");
            }else if (new_step_delay >= step->min_delay && step->dir == step->target_dir){ // siirrtyään ajoon staattisella nopeudella
                //step->last_accel_delay = new_step_delay;
                new_step_delay = step->min_delay;
                step->rest = 0;
                step->state = RUN;
                step->accel_count = 0;
                printf("decel count: %d\n", step->accel_count);
            }

        break;
    }*/
    //step->step_delay = new_step_delay;
}