#include "motion_control.h"


float pid_steps(PID_TypeDef *input){
    float error = input->target - input->new;

    float timestep = (((float)CALC_BALANCE + 1.) / 1000.);

    float d = -(input->new - input->last) / timestep;
    //integral
    input->out_sum += input->KI * error * timestep;

    float p = input->KP * error;

    float ret = (p + input->out_sum + input->KD * d);

    if (ret > input->max)
        ret = input->max;
    else if (ret < input->min)
        ret = input->min;

    input->last = input->new;

    return ret;
}

float pid_simple(PID_TypeDef *pid){
    float error = pid->target - pid->new;
    //integral
    pid->out_sum += error;

    float p = pid->KP * error;
    float i = pid->KI * pid->out_sum;
    float d = pid->KD * (error - pid->last);

    float ret = p + i + d;

    if (ret > pid->max)
        ret = pid->max;
    else if (ret < pid->min)
        ret = pid->min;

    pid->last = error;

    return ret;
}

int pid(PID_TypeDef *input){
    // error
    float error = input->target - input->new;

    float meas_delta = input->new - input->last;

    // integral
    float out_sum = input->out_sum + input->KI * error;

    // propotional on measurement
    out_sum -= input->KP * meas_delta;

    if (out_sum > input->max)
        out_sum = input->max;
    else if (out_sum < input->min)
        out_sum = input->min;

    // propotional on error
    //float ret = input->KP * error;
    // derivative
    float ret = out_sum - input->KD * meas_delta;

    if (ret > input->max)
        ret = input->max;
    else if (ret < input->min)
        ret = input->min;

    input->out_sum = out_sum;

    input->last = input->new;
    return (int)ret;
}

float kalman_state = 0, x0 = 0, p0 = 0;
float covariance = 0.1;

float kalmanfilter(float input){
    x0 = FACTOR_REAL * kalman_state;
    p0 = FACTOR_REAL * FACTOR_REAL * covariance + Q_NOISE;

    float k_temp = (H_MEAS_REAL * p0) / (H_MEAS_REAL * p0 * H_MEAS_REAL + ENV_NOISE);
    kalman_state = x0 + k_temp * (input - H_MEAS_REAL * x0);
    covariance = (1 - k_temp * H_MEAS_REAL) * p0;

    return kalman_state;
}

float complementary_filter(data_t *acc, data_t *gyro, float pitch){
    // Gyro raw value to angle and integrate. 131.2 is based on set gyro sensitivity
    pitch += (float)(-gyro->y / 131.2) * ((CALC_COMP + 1) / 1000.);

    float pitchacc = 0;
    // if we are over the limits of acc sensor
    int forcemangitude = abs(acc->x) + abs(acc->y) + abs(acc->z);

    if(forcemangitude > 8192 && forcemangitude < 32768 ){
        // accelerometer to angle (roll)
        pitchacc = atan2f((float)acc->x, (float)acc->z) * 180 / M_PI;
        // Weighted gyro and acccelerometer values. 
        pitch = pitch * 0.995 + pitchacc * 0.005;
        //printf("acc: %d gyro: %d \n", (int)pitchacc, (int)pitch);
    }
    return pitch;
}

void pid_reset(PID_TypeDef *input){
    input->last = 0.;
    input->new = 0.;
    input->out_sum = 0.;
    input->target = 0.;
}

void print_pid(PID_TypeDef *input){
    char * p[6];
    char * i[6];
    char * d[6];

    gcvt(input->KP, 3, p);
    gcvt(input->KI, 3, i);
    gcvt(input->KD, 3, d);
    printf("P: %s I: %s D: %s\n", p, i , d);
}

static void motion_control_update(Motion_TypeDef *mot){
    // 90 asteen korjaus. Tää pitäs tehä erilailla koska aiheuttaa gimbal lockkia
    mot->anglePID.new = mot->robot_angle - 90.;
    // PID, joka antaa moottorinopeuden
    int target_speed = (int)pid_steps(&mot->anglePID);
    // lasketaan joka toinen kierros kulman korjaus perustuen robotin oikeaan nopeuteen
    if (mot->loop_iter % 2 == 0){
        // Nopeus arvioidaan kalmanfiltterillä koska stepit eivät ole robotin oikea fyysinen nopeus
        mot->velocityPID.new = kalmanfilter((float)(mot->step1->step_count - mot->step_count));
        mot->step_count = mot->step1->step_count;
        if(mot->direction == MOVE_FORWARD){
            // unohda integraali kun liikutaan; muistetaan taas kun ollaan paikallaan
            mot->velocityPID.out_sum = 0.;
            if(mot->velocityPID.target < 15)
                mot->velocityPID.target += 0.5;
        }else if(mot->direction == MOVE_REVERSE){
            mot->velocityPID.out_sum = 0.;
            if(mot->velocityPID.target > -15)
                mot->velocityPID.target -= .5;
        }else if(mot->direction == MOVE_STOP){
            mot->velocityPID.target = 0.;
        }
        // asetetaan kohdekulma välillä +10 -10
        mot->anglePID.target = -pid_steps(&mot->velocityPID);
    }

    switch(mot->direction){
        case TURN_LEFT:
            mot->left_motor_offset = 200;
            mot->right_motor_offset = -100;
        break;
        case TURN_RIGHT:
            mot->left_motor_offset = -200;
            mot->right_motor_offset = 100;
        break;
        default:// forward or backwards
            mot->left_motor_offset = 0;
            mot->right_motor_offset = 0;
    }

    //printf("target angle: %d current speed: %d speed target: %d angle: %d steps: %d \n", 
    //(int)anglePID.target, (int) velocityPID.new, target_speed, (int)real_pitch, step1.step_count );

    // set stepper speed
    stepper_setspeed(mot->step1, -(target_speed + mot->left_motor_offset));
    stepper_setspeed(mot->step2, target_speed + mot->right_motor_offset);
}

// the balance main loop/update loop
void motion_loop(Motion_TypeDef *mot){
    // This changes balance state
    if(abs((int)(mot->robot_angle - 90.)) < 30 && mot->app_state == APP_RUN && mot->state == BL_STOP){
        // Detect that robot is lifted upright and 
        // reset all parameters to prepare for balancing
        mot->state = BL_START; 
        stepper_setspeed(mot->step1, 0);
        stepper_setspeed(mot->step2, 0);
        enable_stepper(mot->step1);
        enable_stepper(mot->step2);
        pid_reset(&mot->anglePID);
        pid_reset(&mot->velocityPID);
        
        mot->step1->step_count = 0;
        mot->step_count = 0;
        mot->setup = 0;
    }else if(mot->state == BL_RUN){
        // stop
        mot->state = BL_STOP;
    }

    // this acts on the balance state
    if(mot->state == BL_RUN){ // actual balance
        motion_control_update(mot);
        mot->loop_iter++;
    }else if(mot->state == BL_STOP){ // Stop
        disable_stepper(mot->step1);
        disable_stepper(mot->step2);
        mot->state = BL_IDLE;
    }else if(mot->state == BL_START){ // calibration time
        // Wait till robot is hold upright for some time
        if(mot->setup < 30){
            float angle = mot->robot_angle - 90.;
            // wait for upright position (robot should be held +-3 degrees from balance point)
            if(fabs(angle) < 3.){
                mot->setup++;
                // init values used in calculations
                mot->anglePID.last = angle;
            }
        }else{
            // All done calibrating
            mot->state = BL_RUN;
        }
    }
}

void motion_init(Motion_TypeDef *mot){
    //**** PID CONTROLLER INIT ****//
    mot->velocityPID = (PID_TypeDef){.KP = 0.45, .KI = 0.25, .KD = 0.005, 
                                .min = -13., .max = 13., .target = 0.};

    mot->anglePID = (PID_TypeDef){.KP = 150., .KI = .05 , .KD = 1.2,
                            .min = -1500, .max = 1500, 
                            .target = 1.5};

    mot->app_state = APP_READY;
    mot->direction = MOVE_STOP;
    mot->state = BL_IDLE;
}