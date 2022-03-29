/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fmpi2c.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lora_sx1276.h"
#include "bmx160.h"
#include "stepper.h"
#include "motion_control.h"
#include "messaging.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t imu_data_status = IMU_DATA_PENDING;
volatile uint8_t lora_rx_status = LORA_RX_DATA_PENDING;
volatile uint8_t adc_data_status = ADC_DATA_PENDING;
volatile uint8_t uart_data_pending = 0;

uint8_t UART6_rxBuffer;
uint8_t uart_rx_buf[32];
Stepper_HandleTypeDef step1;
Stepper_HandleTypeDef step2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FMPI2C1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI5_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(100);
  RetargetInit(&huart6);

  uint32_t last_run_balance = 0;
  uint32_t last_read_voltage = 0;
  uint32_t last_blink = 0;
  uint32_t last_run_comp = 0;
  uint32_t last_lora_tx = 0;
  uint32_t last_lora_meas = 0;

  MainState_t app_state = STOP;

  lora_sx1276 lora;
  // Lora Nreset
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

  // SX1276 compatible module connected to SPI2, NSS pin connected to GPIO with label LORA_NSS
  uint8_t res = lora_init(&lora, &hspi2, GPIOB, GPIO_PIN_9, LORA_BASE_FREQUENCY_EU);
  if (res != LORA_OK) {
    // Initialization failed
    printf("Can't read LoRa unit, status: %d\n", res);
    app_state = INIT_FAIL;
  }else
    printf("LoRa init Successfull\n");

  bmx160 imu;
  res = bmx160_init(&imu, &hi2c1, GPIOA, GPIO_PIN_8);

  while(res > 0){
    printf("Can't read IMU unit, status: %d\n", res);
    app_state = INIT_FAIL;
    HAL_Delay(1000);
  }

  // Run this only to get IMU calibration parameters
  //bmx160_calibrate(&imu);

  //**** STEPPER CONTROLLER INIT ****//
  init_stepper(&step1, &htim5, &hspi1, 
              tmc2130_1_step_GPIO_Port, tmc2130_1_step_Pin,
              tmc2130_1_dir_GPIO_Port, tmc2130_1_dir_Pin, 
              tmc2130_1_enable_GPIO_Port, tmc2130_1_enable_Pin, 
              tmc2130_1_nss_GPIO_Port, tmc2130_1_nss_Pin);
  init_stepper(&step2, &htim9, &hspi5, 
              tmc2130_2_step_GPIO_Port, tmc2130_2_step_Pin,
              tmc2130_2_dir_GPIO_Port,tmc2130_2_dir_Pin,    
              tmc2130_2_enable_GPIO_Port, tmc2130_2_enable_Pin, 
              tmc2130_2_nss_GPIO_Port, tmc2130_2_nss_Pin);

  //**** BALANCE DATA INIT ****//
  Motion_TypeDef motion_data = {0};
  motion_data.step1 = &step1;
  motion_data.step2 = &step2;
  motion_init(&motion_data);

  //**** LORA RECEIVE START ****//
  uint8_t lora_rx_buffer[128];
  // interrupt on receive
  lora_enable_interrupt_rx_done(&lora);
  // Put LoRa modem into continuous receive mode
  lora_mode_receive_continuous(&lora);

  HAL_Delay(10);
  // Start UART data reception (from usb)
  HAL_UART_Receive_IT(&huart6, uart_rx_buf, 2);

  // measurement results for sending
  RobotData_t robot_data = {0};
  // hold ADC related data
  ADC_Voltage_TypeDef battery_data = {0};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(HAL_GetTick() - last_run_comp > CALC_COMP){
      // run complementary filter on raw IMU values and update angle value
      motion_data.robot_angle = complementary_filter(&imu.acc, &imu.gyro, motion_data.robot_angle);
      // trigger new imu data fetch
      imu_start_update(&imu);
      last_run_comp = HAL_GetTick();
    }

    // main balance loop
    if(HAL_GetTick() - last_run_balance > CALC_BALANCE){
      last_run_balance = HAL_GetTick();
      motion_loop(&motion_data);
    }

    // Read voltage to adc_raw every READ_VOLTAGE
    if(HAL_GetTick() - last_read_voltage > READ_VOLTAGE){
      // convert voltage
      robot_data.voltage = adc_voltage_update(&battery_data, HAL_ADC_GetValue(&hadc1));
      if(robot_data.voltage < BATTERY_LOW_UINT && robot_data.voltage > 0){
        motion_data.app_state = LOW_BATTERY;
        printf("Low voltage <11.1 \n");
      }
      // start new read
      adc_start(&hadc1);
      last_read_voltage = HAL_GetTick();
    }

    // Check if imu data is ready to be read
    if(imu_data_status == IMU_DATA_READY){
      // update values in imu structure. Read them like imu.acc.x etc...
      imu_end_update(&imu);
      // print values (for debug only)
      //imu_print_values(&imu);
      imu_data_status = IMU_DATA_PENDING;
    }
    
    // TODO: move this to a function
    if(lora_rx_status == LORA_RX_DATA_READY){
      // LoRa receive check
      uint8_t res;
      uint8_t len = lora_receive_packet(&lora, lora_rx_buffer, sizeof(lora_rx_buffer), &res);

      //uint8_t len = lora_receive_packet_blocking(&lora, lora_rx_buffer, sizeof(lora_rx_buffer), 10000, &res);
      if (res != LORA_OK) {
        printf("Receive faile, code: %d\n", res);
        // Receive failed
      }
      lora_rx_buffer[len] = 0;  // null terminate string to print it
      printf("Lora buff len: %d\n", len);
      printf("Lora RSSI: %d\n", lora_packet_rssi(&lora));
      printf("Lora SNR: %d\n", lora_packet_snr(&lora));

      for(int i = 0; i < len; i++){
        printf("0x%x ", lora_rx_buffer[i]);
      }
      printf("\n");

      lora_parse_message(lora_rx_buffer);
      motion_data.direction = (MoveDirection_t)lora_rx_buffer[0];
      
      // printf("'%s'\n", lora_rx_buffer);
      lora_rx_status = LORA_RX_DATA_PENDING;
    }

    // Led blinking
    if(HAL_GetTick() - last_blink > BLINK_LED){

      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
      //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
      if(motion_data.state != BL_RUN)
        last_blink = HAL_GetTick();
      else
        last_blink = HAL_GetTick() - 900;
    }

    // Check if we have messages to send
    if(HAL_GetTick() - last_lora_tx > SEND_LORA_TX){

      uint8_t tx_len = 0;
      uint8_t lora_tx_msg[15] = {};

      if(last_lora_meas % 5 == 0){
        tx_len = lora_create_measurement_message(&robot_data, lora_tx_msg);
        last_lora_meas = 0;
      }else if(robot_data.send_pid == PID_SEND_ANGLE){
        tx_len = lora_create_pid_message(&motion_data.anglePID, lora_tx_msg, 'A');
        robot_data.send_pid = 0;
      }else if(robot_data.send_pid == PID_SEND_VELOCITY){
        tx_len = lora_create_pid_message(&motion_data.velocityPID, lora_tx_msg, 'V');
        robot_data.send_pid = 0;
      }

      // send data if we have any
      if (tx_len > 0){
        // write to lora without dma, does not affect program performance
        uint8_t res = lora_send_packet(&lora, lora_tx_msg, tx_len);
        if (res != LORA_OK) {
          printf("Send fail: %d\n", res);
          // Send failed
        }
        printf("lora send\n");
        
        lora_mode_receive_continuous(&lora);
      }

      last_lora_meas++;
      last_lora_tx = HAL_GetTick();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FMPI2C1;
  PeriphClkInitStruct.Fmpi2c1ClockSelection = RCC_FMPI2C1CLKSOURCE_APB;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    uart_data_pending = 1;
}

// Timer interrupts
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
  if(htim->Instance == TIM5){ // stepper 1
    update_stepper(&step1);
  }else if(htim->Instance == TIM9){ // stepper 2
    update_stepper(&step2);
  }
}

// pin ch 5-9 callback. Loralle
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == lora_int_Pin){
    lora_rx_status = LORA_RX_DATA_READY;
  }
}

// ADC callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  adc_data_status = ADC_DATA_READY;
}

// i2c dma callback imulle
// FYI myös eventit pitää olla I2C:lle päällä cubemx:Stä että tää toimii
void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c){
  //if (hi2c->Instance==hi2c1.Instance){ 
  //   HAL_DMA_Abort_IT(hi2c->hdmarx);
  // }
  imu_data_status = IMU_DATA_READY;
}


// LoRa SPI callback RX done
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
  //if(hspi->Instance == )
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
