#include "motion_control.h"

void pid(float meas_angle, float target){

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