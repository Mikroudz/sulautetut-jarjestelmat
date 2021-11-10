#include "bmx160.h"

#define ADDR 0x68

#define REG_CHIPID 0x00
#define REG_DATA 0x04
#define REG_PMU_STATUS 0x03
#define REG_CMD 0x7E
#define REG_ACC_RANGE 0x41
#define REG_GYRO_RANGE 0x43

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


static void read_register(bmx160 *imu, uint8_t reg_addr, size_t data_len, uint8_t *buf){
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Master_Transmit(imu->i2c, (ADDR << 1), &reg_addr, 1, 1000);

    if(ret != HAL_OK){
        printf("IMU i2c vituiks meni: %d\n", ret);
    }else{
        ret = HAL_I2C_Master_Receive(imu->i2c, (ADDR << 1), (uint8_t *)buf, data_len, 1000);
        if(ret != HAL_OK){
            printf("IMU i2c vituiks meni\n");
        }
    }

    //return buf;
}

static void read_register_dma(bmx160 *imu, uint8_t reg_addr, size_t data_len, uint8_t *buf){
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Master_Transmit(imu->i2c, (ADDR << 1), &reg_addr, 1, 1000);

    if(ret != HAL_OK){
        printf("IMU i2c vituiks meni: %d\n", ret);
    }else{
        ret = HAL_I2C_Master_Receive_DMA(imu->i2c, (ADDR << 1), (uint8_t *) buf, data_len);
        if(ret != HAL_OK){
            printf("IMU i2c vituiks meni\n");
        }
    }
}

static void write_register(bmx160 *imu, uint8_t reg_addr, uint8_t value){
    HAL_StatusTypeDef ret;

    uint8_t txbuf[2] = {0};
    txbuf[0] = reg_addr;
    txbuf[1] = value;


    ret = HAL_I2C_Master_Transmit(imu->i2c, (ADDR << 1), (uint8_t *) txbuf, 2, 1000);

    if(ret != HAL_OK){
        printf("IMU TX i2c vituiks meni: %d\n", ret);
    }
}

uint8_t bmx160_init(bmx160 *imu, I2C_HandleTypeDef *i2c, 
    GPIO_TypeDef *int1_port,
    GPIO_TypeDef *int1_pin
){

    imu->i2c = i2c;
    uint8_t buf[1] = {0};
    imu_softreset(imu);
    write_register(imu, REG_CMD, 0x11);
    HAL_Delay(50);
    write_register(imu, REG_CMD, 0x15);
    HAL_Delay(100);
    write_register(imu, REG_CMD, 0x19);
    HAL_Delay(10);

    // set acc range 4G == 0b0101
    write_register(imu, REG_ACC_RANGE, 0b0101);

    // set gyro range +-250 deg/s
    write_register(imu, REG_GYRO_RANGE, 0b0011);

    read_register(imu, REG_CHIPID, 1, buf);
    printf("Read CHIPID: 0x%x\n\r", buf[0]);

    read_register(imu, REG_PMU_STATUS, 1, buf);
    printf("Read PMU_STATUS: 0x%x\n\r", buf[0]);

    // Clear stale data
    imu_start_update(imu);

}

void imu_start_update(bmx160 *imu){
    //printf("Read imu data\n");
    read_register_dma(imu, REG_DATA, 23, imu->data_buf);//imu->data_buf);

}   

void imu_print_values(bmx160 *imu){

    printf("x: %d, y: %d, z: %d, x: %d, y: %d, z: %d\n",
     imu->acc.x, imu->acc.y, imu->acc.z, imu->gyro.x, imu->gyro.y, imu->gyro.z);
}

// Kutsu dma takaisinkutsusta kun data on siirretty
void imu_end_update(bmx160 *imu){
    uint8_t *buf = imu->data_buf;
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    // Muuta arvot kiihtyvyys
    acc_x = (int16_t)((buf[15] << 8) | buf[14]);
    acc_y = (int16_t)((buf[17] << 8) | buf[16]);
    acc_z = (int16_t)((buf[19] << 8) | buf[18]);
    // skaalaa kiihtyvyys
    acc_y *= (BMX160_ACCEL_MG_LSB_4G * 10);
    acc_x *= (BMX160_ACCEL_MG_LSB_4G * 10);
    acc_z *= (BMX160_ACCEL_MG_LSB_4G * 10);

    // Parsee gyro
    gyro_x = (int16_t)((buf[9] << 8) | buf[8]);
    gyro_y = (int16_t)((buf[11] << 8) | buf[10]);
    gyro_z = (int16_t)((buf[13] << 8) | buf[12]);
    // skaalaa gyro
    gyro_y *= BMX160_GYRO_SENSITIVITY_250DPS;
    gyro_x *= BMX160_GYRO_SENSITIVITY_250DPS;
    gyro_z *= BMX160_GYRO_SENSITIVITY_250DPS;

    imu->acc.x = acc_x;
    imu->acc.y = acc_y;
    imu->acc.z = acc_z;

    imu->gyro.x = gyro_x;
    imu->gyro.y = gyro_y;
    imu->gyro.z = gyro_z;

    //printf("x: %d, y: %d, z: %d, x: %d, y: %d, z: %d\n", acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
}

void imu_softreset(bmx160 *imu){
    uint8_t buf = BMX160_SOFT_RESET_CMD;
    write_register(imu, REG_CMD, &buf);
    HAL_Delay(BMX160_SOFT_RESET_DELAY_MS);
}