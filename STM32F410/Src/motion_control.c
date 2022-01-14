#include "motion_control.h"

float last_meas = 0, out_sum = 0;

int pid(float meas_angle, float target){
    float error = target - meas_angle;

    float meas_delta = meas_angle - last_meas;
    // integral
    out_sum += KI * error;

    // propotional on measured angle
    out_sum -= KP * meas_delta;
    if (out_sum > MAX_VAL)
        out_sum = MAX_VAL;
    else if (out_sum < MIN_VAL)
        out_sum = MIN_VAL;

    // propotional on error
    float ret = KP * error;
    // derivative
    ret += out_sum - KD * meas_delta;

    if (ret > MAX_VAL)
        ret = MAX_VAL;
    else if (ret < MIN_VAL)
        ret = MIN_VAL;

    last_meas = meas_angle;
    return (int)ret;
}


float complementary_filter(data_t *acc, data_t *gyro, float pitch){

    pitch += (float)(gyro->x * BMX160_GYRO_SENSITIVITY_1000DPS) * ((float)CALC_BALANCE / 1000);
    float pitchacc;
    int forcemangitude = abs(acc->x) + abs(acc->y) + abs(acc->z);

    if(forcemangitude > 8192 && forcemangitude < 32768 ){
        pitchacc = atan2f((float)acc->x, (float)acc->z) * 180 / M_PI;
        pitch = pitch * 0.94 + pitchacc * 0.06;
    }

    return pitch;
}