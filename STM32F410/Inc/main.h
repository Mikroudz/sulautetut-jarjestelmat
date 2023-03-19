/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "retarget.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} data_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define M_PI 3.14159265359

// how often to run different functions in ms
#define CALC_BALANCE 10 // main loop
#define BLINK_LED 100
#define READ_IMU 10
#define READ_VOLTAGE 500
#define STEPPER_UPDATE_RATE 4000
#define CALC_COMP 10
#define SEND_LORA_MEAS 1000
#define SEND_LORA_TX 100
#define CALC_DISTANCE 500

// ADC status
#define ADC_DATA_PENDING 0
#define ADC_DATA_READY 1

// Imu tilat
#define IMU_DATA_PENDING 1
#define IMU_DATA_READY 2

// LoRa status
#define LORA_RX_DATA_PENDING 1
#define LORA_RX_DATA_READY 2

// misc constants
#define BATTERY_LOW 11.1 // volts

// Which pid parameters to send
#define PID_SEND_ANGLE 1
#define PID_SEND_VELOCITY 2

typedef enum {
  APP_READY = 0,
  APP_RUN,
  APP_STOP,
  LOW_BATTERY,
  INIT_FAIL
} MainState_t;

typedef enum {
  MOVE_STOP = 0,  
  MOVE_FORWARD,
  MOVE_REVERSE,
  TURN_LEFT,
  TURN_RIGHT
} MoveDirection_t;

typedef struct {
  uint16_t voltage;
  uint16_t distance_front;
  uint16_t distance_rear;
  uint16_t illumination;
  uint16_t tempereature;
  uint8_t robot_status;
  uint8_t send_pid;

} RobotData_t;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define vcc_meas_Pin GPIO_PIN_1
#define vcc_meas_GPIO_Port GPIOC
#define tmc2130_1_nss_Pin GPIO_PIN_4
#define tmc2130_1_nss_GPIO_Port GPIOA
#define tmc2130_2_dir_Pin GPIO_PIN_4
#define tmc2130_2_dir_GPIO_Port GPIOC
#define tmc2130_2_step_Pin GPIO_PIN_5
#define tmc2130_2_step_GPIO_Port GPIOC
#define tmc2130_2_nss_Pin GPIO_PIN_1
#define tmc2130_2_nss_GPIO_Port GPIOB
#define tmc2130_2_enable_Pin GPIO_PIN_2
#define tmc2130_2_enable_GPIO_Port GPIOB
#define tmc2130_1_dir_Pin GPIO_PIN_12
#define tmc2130_1_dir_GPIO_Port GPIOB
#define tmc2130_1_step_Pin GPIO_PIN_6
#define tmc2130_1_step_GPIO_Port GPIOC
#define tmc2130_1_enable_Pin GPIO_PIN_8
#define tmc2130_1_enable_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOA
#define lora_rx_enable_Pin GPIO_PIN_12
#define lora_rx_enable_GPIO_Port GPIOC
#define lora_tx_enable_Pin GPIO_PIN_4
#define lora_tx_enable_GPIO_Port GPIOB
#define lora_int_Pin GPIO_PIN_5
#define lora_int_GPIO_Port GPIOB
#define lora_int_EXTI_IRQn EXTI9_5_IRQn
#define IMU_I2C_SCL_Pin GPIO_PIN_6
#define IMU_I2C_SCL_GPIO_Port GPIOB
#define IMU_i2C_SDA_Pin GPIO_PIN_7
#define IMU_i2C_SDA_GPIO_Port GPIOB
#define lora_nreset_Pin GPIO_PIN_8
#define lora_nreset_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
