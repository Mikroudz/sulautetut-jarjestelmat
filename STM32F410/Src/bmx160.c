#include "bmx160.h"

const uint8_t foc_calc_values[6] = {1, 8, 236, 255, 4, 7};


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

static uint8_t write_register(bmx160 *imu, uint8_t reg_addr, uint8_t value){
    HAL_StatusTypeDef ret;

    uint8_t txbuf[2] = {0};
    txbuf[0] = reg_addr;
    txbuf[1] = value;


    ret = HAL_I2C_Master_Transmit(imu->i2c, (ADDR << 1), (uint8_t *) txbuf, 2, 1000);

    if(ret != HAL_OK){
        printf("IMU TX i2c vituiks meni: %d\n", ret);
        return ret;
    }
}

uint8_t bmx160_init(bmx160 *imu, I2C_HandleTypeDef *i2c, 
    GPIO_TypeDef *int1_port,
    uint16_t int1_pin
){

    imu->i2c = i2c;
    uint8_t buf[8] = {0};
    imu_softreset(imu);
    HAL_Delay(50);
    write_register(imu, REG_CMD, 0x11);
    HAL_Delay(50);
    write_register(imu, REG_CMD, 0x15);
    HAL_Delay(100);
    write_register(imu, REG_CMD, 0x19);
    HAL_Delay(10);

    // set acc range 4G == 0b0101
    uint8_t ret = write_register(imu, REG_ACC_RANGE, BMX160_ACCEL_RANGE_4G);

    ret |= write_register(imu, REG_NV_CONF, BMX160_I2C_WATCHDOG_ENABLE);

    // set gyro range +-250 deg/s
    ret |= write_register(imu, REG_GYRO_RANGE, BMX160_GYRO_RANGE_250_DPS);

    ret |= write_register(imu, REG_GYRO_CONF, BMX160_GYRO_SAMPLE_800HZ);

    ret |= write_register(imu, REG_OFFSET + 6, 0xc0);

    // acc orientation, X -1G (on side), y & z 0 G
    ret |= write_register(imu, REG_FOC_CONF, 0x5f);
    HAL_Delay(10);

    if (ret != HAL_OK){
        return ret;
    }
    // clear stall
    read_register(imu, REG_DATA, 23, imu->data_buf);
    HAL_Delay(10);
    // start fast offset compensation
    //write_register(imu, REG_CMD, 0x03);
    //HAL_Delay(255);

    // set old values to FOC register
    for(int i = 0; i < 6; i++){
        write_register(imu, REG_OFFSET+i, foc_calc_values[i]);
        HAL_Delay(10);
    }


    read_register(imu, REG_OFFSET, 7, buf);

    for(int i = 0; i < 7; i++){
        printf("%d ", buf[i]);
    }

    read_register(imu, REG_CHIPID, 1, buf);
    printf("Read CHIPID: 0x%x\n\r", buf[0]);

    read_register(imu, REG_PMU_STATUS, 1, buf);
    printf("Read PMU_STATUS: 0x%x\n\r", buf[0]);

    read_register(imu, REG_GYRO_RANGE, 1, buf);
    printf("Read GYRO_RANGE: 0x%x\n\r", buf[0]);
    // Clear stale data
    imu_start_update(imu);
    return HAL_OK;
}

void bmx160_calibrate(bmx160 *imu){
    int32_t gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;

    for(int i = 0; i < 100; i++){
        read_register(imu, REG_DATA, 23, imu->data_buf);
        HAL_Delay(5);
    }

    for(int i = 0; i < 5000; i++){
        read_register(imu, REG_DATA, 23, imu->data_buf);
        imu_end_update(imu);
        gx += imu->gyro.x;
        gy += imu->gyro.y;
        gz += imu->gyro.z;

        ax += imu->acc.x;
        ay += imu->acc.y;
        az += imu->acc.z;
        HAL_Delay(5);
    }

    gx /= 4999;
    gy /= 4999;
    gz /= 4999;

    ax /= 4999;
    ay /= 4999;
    az /= 4999;
    printf("Raw mean:\n");
    printf("gx: %d, gy: %d, gz: %d, ax: %d, ay: %d, az: %d\n", gx, gy, gz, ax, ay, az);
    printf("Scaled mean:\n");


    printf("gx: %d, gy: %d, gz: %d, gx: %d, gy: %d, gz: %d\n", 
    (int)(gx * BMX160_GYRO_SENSITIVITY_250DPS / 0.061), (int)(gy* BMX160_GYRO_SENSITIVITY_250DPS / 0.061), (int)(gz* BMX160_GYRO_SENSITIVITY_250DPS / 0.061),
    (int)(ax * BMX160_ACCEL_MG_LSB_4G * 10 / 3.9), (int)(ay* BMX160_ACCEL_MG_LSB_4G * 10 / 3.9), (int)(az* BMX160_ACCEL_MG_LSB_4G * 10 / 3.9));
    HAL_Delay(10000);
}

void imu_start_update(bmx160 *imu){
    //printf("Read imu data\n");
    read_register_dma(imu, REG_DATA, 23, imu->data_buf);//imu->data_buf);

}   

void imu_print_values(bmx160 *imu){

    printf(" x: %d, y: %d, z: %d\n",
      imu->gyro.x, imu->gyro.y, imu->gyro.z);
}

// Kutsu dma takaisinkutsusta kun data on siirretty
void imu_end_update(bmx160 *imu){
    uint8_t *buf = imu->data_buf;
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    // Muuta arvot kiihtyvyys
    acc_x = (int16_t)(((uint16_t)buf[15] << 8) | buf[14]);
    acc_y = (int16_t)(((uint16_t)buf[17] << 8) | buf[16]);
    acc_z = (int16_t)(((uint16_t)buf[19] << 8) | buf[18]);
    // skaalaa kiihtyvyys
    /*acc_y *= (BMX160_ACCEL_MG_LSB_4G * 10);
    acc_x *= (BMX160_ACCEL_MG_LSB_4G * 10);
    acc_z *= (BMX160_ACCEL_MG_LSB_4G * 10);*/

    // Parsee gyro
    gyro_x = (int16_t)(((int16_t)buf[9] << 8) | buf[8]);
    gyro_y = (int16_t)(((int16_t)buf[11] << 8) | buf[10]);
    gyro_z = (int16_t)(((int16_t)buf[13] << 8) | buf[12]);
    // skaalaa gyro
    /*gyro_y *= BMX160_GYRO_SENSITIVITY_250DPS;
    gyro_x *= BMX160_GYRO_SENSITIVITY_250DPS;
    gyro_z *= BMX160_GYRO_SENSITIVITY_250DPS;*/

    imu->acc.x = acc_x;
    imu->acc.y = acc_y;
    imu->acc.z = acc_z;

    imu->gyro.x = gyro_x; //- G_X_COMP;
    imu->gyro.y = gyro_y; //- G_Y_COMP;
    imu->gyro.z = gyro_z; //- G_Z_COMP;

    //printf("x: %d, y: %d, z: %d, x: %d, y: %d, z: %d\n", acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
}

void imu_softreset(bmx160 *imu){
    uint8_t buf = BMX160_SOFT_RESET_CMD;
    write_register(imu, REG_CMD, buf);
    HAL_Delay(BMX160_SOFT_RESET_DELAY_MS);
}