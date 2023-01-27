#ifndef __LORA_APP_H
#define __LORA_APP_H

#include "main.h"
//#include "lora_sx1276.h"
#include <stdio.h>
#include "motion_control.h"

#define MSG_TYPE_PID 'A'
#define MSG_
#define MSG_TYPE_SENSOR 'C'

enum {
    LORA_INT_EMPTY = 0,
    LORA_INT_ACTIVE
};


uint8_t lora_create_measurement_message(RobotData_t *data, uint8_t *buf);
uint8_t lora_create_pid_message(PID_TypeDef *pid, uint8_t *buf, char type);
void lora_parse_pid(uint8_t *buf);
void lora_parse_message(uint8_t *buf);
#endif
