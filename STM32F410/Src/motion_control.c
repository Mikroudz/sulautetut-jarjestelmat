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