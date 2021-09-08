
#ifndef __BMX160_H
#define __BMX160_H

#include "main.h"
#include <stdio.h>


typedef struct {
    I2C_HandleTypeDef   *i2c;
    GPIO_TypeDef        *int1_port;
    uint16_t            int1_pin;
    uint8_t             data_buf[23];

} bmx160;


//static uint32_t read_register(bmx160 *imu, uint8_t address);
//static void write_register(bmx160 *imu, uint8_t address, uint32_t value);

uint8_t bmx160_init(bmx160 *imu, I2C_HandleTypeDef *i2c, 
    GPIO_TypeDef *int1_port,
    GPIO_TypeDef *int1_pin
    );
void imu_start_update(bmx160 *imu);
void imu_softreset(bmx160 *imu);
void imu_end_update(bmx160 *imu);

#endif
