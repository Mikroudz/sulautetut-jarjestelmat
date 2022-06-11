
#ifndef __BMX160_H
#define __BMX160_H

#include "main.h"
#include <stdio.h>

#define ADDR 0x68

#define REG_CHIPID 0x00
#define REG_DATA 0x04
#define REG_PMU_STATUS 0x03
#define REG_CMD 0x7E
#define REG_ACC_RANGE 0x41
#define REG_GYRO_CONF 0x42
#define REG_GYRO_RANGE 0x43
#define REG_FOC_CONF 0x69
#define REG_NV_CONF 0x70
#define REG_OFFSET 0x71

#define BMX160_I2C_WATCHDOG_ENABLE 0x4

/** Soft reset command */
#define BMX160_SOFT_RESET_CMD                    0xb6
#define BMX160_SOFT_RESET_DELAY_MS               15

/* Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define BMX160_MAGN_UT_LSB      (0.3F)

/** Range settings */
/* Accel Range */
#define BMX160_ACCEL_RANGE_2G                    0x03
#define BMX160_ACCEL_RANGE_4G                    0x05
#define BMX160_ACCEL_RANGE_8G                    0x08
#define BMX160_ACCEL_RANGE_16G                   0x0C

/* Gyro Range */
#define BMX160_GYRO_RANGE_2000_DPS               0x00
#define BMX160_GYRO_RANGE_1000_DPS               0x01
#define BMX160_GYRO_RANGE_500_DPS                0x02
#define BMX160_GYRO_RANGE_250_DPS                0x03
#define BMX160_GYRO_RANGE_125_DPS                0x04

/* Gyro Sample rate */
#define BMX160_GYRO_SAMPLE_800HZ                0x0b

/* Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000061035mg) */
#define BMX160_ACCEL_MG_LSB_2G      0.000061035F
/* Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000122070mg) */
#define BMX160_ACCEL_MG_LSB_4G      0.000122070F
/* Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000244141mg) */
#define BMX160_ACCEL_MG_LSB_8G      0.000244141F
/* Macro for mg per LSB at +/- 16g sensitivity (1 LSB = 0.000488281mg) */
#define BMX160_ACCEL_MG_LSB_16G     0.000488281F

/* Gyroscope sensitivity at 125dps */
#define BMX160_GYRO_SENSITIVITY_125DPS  0.0038110F // Table 1 of datasheet
/* Gyroscope sensitivity at 250dps */
#define BMX160_GYRO_SENSITIVITY_250DPS  0.0076220F // Table 1 of datasheet
/* Gyroscope sensitivity at 500dps */
#define BMX160_GYRO_SENSITIVITY_500DPS  0.0152439F
/* Gyroscope sensitivity at 1000dps */
#define BMX160_GYRO_SENSITIVITY_1000DPS 0.0304878F
/* Gyroscope sensitivity at 2000dps */
#define BMX160_GYRO_SENSITIVITY_2000DPS 0.0609756F

#define G_X_COMP -19
#define G_Y_COMP 11 
#define G_Z_COMP -65
#define A_X_COMP 8145
#define A_Y_COMP -298
#define A_Z_COMP 598



typedef struct {
    I2C_HandleTypeDef   *i2c;
    GPIO_TypeDef        *int1_port;
    uint16_t            int1_pin;
    uint8_t             data_buf[23];
    data_t              gyro; // data_t in main.h
    data_t              acc;
    data_t              magnetometer;
} bmx160;

//static uint32_t read_register(bmx160 *imu, uint8_t address);
//static void write_register(bmx160 *imu, uint8_t address, uint32_t value);

uint8_t bmx160_init(bmx160 *imu, I2C_HandleTypeDef *i2c, 
    GPIO_TypeDef *int1_port,
    uint16_t int1_pin
    );
void imu_start_update(bmx160 *imu);
void imu_softreset(bmx160 *imu);
void imu_end_update(bmx160 *imu);
void imu_print_values(bmx160 *imu);

#endif
