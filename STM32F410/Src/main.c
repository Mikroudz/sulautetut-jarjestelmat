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
#include "tmc2130.h"
#include "bmx160.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Imu tilat
#define IMU_DATA_PENDING 1
#define IMU_DATA_READY 2

// LoRa status
#define LORA_RX_DATA_PENDING 1
#define LORA_RX_DATA_READY 2

// Stepper step delay (5 kHz)
#define STEP_UPDATE_US 200
#define STEPS_MIN_PS 50
#define STEPS_MAX_PS 1500

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t imu_data_status = IMU_DATA_PENDING;
uint8_t lora_rx_status = LORA_RX_DATA_PENDING;

uint16_t target_steps_per_second = 0;
uint16_t steps_per_second_per_second = 1000;
uint16_t current_steps_ps = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void stepper_setaccel(uint16_t accel){
  // accell is in rps^2
  steps_per_second_per_second = 200 * (accel);
  // Autoreload
  TIM6->ARR = (uint16_t)((SystemCoreClock / (19 * steps_per_second_per_second)));
  // prescaler
  TIM6->PSC = (uint16_t)20 - 1;
  TIM5->PSC = (uint16_t)20 - 1;
}

void stepper_setspeed(uint16_t speed){
  // Speed in rpm/m
  // fullstep
  target_steps_per_second = 200 * (speed/60);
  HAL_TIM_Base_Start_IT(&htim6);
}
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
  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  HAL_Delay(20);
  RetargetInit(&huart6);

  lora_sx1276 lora;

  // SX1276 compatible module connected to SPI2, NSS pin connected to GPIO with label LORA_NSS
  uint8_t res = lora_init(&lora, &hspi2, GPIOB, GPIO_PIN_9, LORA_BASE_FREQUENCY_EU);
  if (res != LORA_OK) {
    // Initialization failed
    printf("Lora ei init koodi: %d\n", res);
  }else
    printf("lora init DONE\n");

  tmc2130 stepper1;
  tmc2130 stepper2;

  tmc2130_init(&stepper1, &hspi1,
    tmc2130_1_enable_GPIO_Port, tmc2130_1_dir_GPIO_Port, tmc2130_1_step_GPIO_Port, tmc2130_1_nss_GPIO_Port, 
    tmc2130_1_enable_Pin, tmc2130_1_dir_Pin, tmc2130_1_step_Pin, tmc2130_1_nss_Pin);

  bmx160 imu;
  bmx160_init(&imu, &hi2c1, GPIOA, GPIO_PIN_8);
  
  //tmc2130_init(&stepper2, &hspi5,
  //  tmc2130_2_enable_GPIO_Port, tmc2130_2_dir_GPIO_Port, tmc2130_2_step_GPIO_Port, tmc2130_2_nss_GPIO_Port, 
  //  tmc2130_2_enable_Pin, tmc2130_2_dir_Pin, tmc2130_2_step_Pin, tmc2130_2_nss_Pin);


  //**** LORA RECEIVE START ****//
  uint8_t lora_rx_buffer[128];
  // interrupt on receive
  lora_enable_interrupt_rx_done(&lora);
  // Put LoRa modem into continuous receive mode
  lora_mode_receive_continuous(&lora);

  //**** LORA RECEIVE END ****//

  HAL_Delay(1000);
  
  stepper_enable(&stepper1);
  stepper_setaccel(40);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    HAL_Delay(1000);
    if(imu_data_status == IMU_DATA_READY){
      imu_end_update(&imu);
      imu_data_status = IMU_DATA_PENDING;
    }

    if(lora_rx_status == LORA_RX_DATA_READY){
      // LoRa receive check
      // Wait for packet up to 10sec
      uint8_t res;
      uint8_t len = lora_receive_packet_dma_start(&lora, lora_rx_buffer, sizeof(lora_rx_buffer), &res);

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
      
     // printf("'%s'\n", lora_rx_buffer);
      lora_rx_status = LORA_RX_DATA_PENDING;
    }

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    uint16_t adc_raw = HAL_ADC_GetValue(&hadc1);
    float voltage_meas = (float)adc_raw * (3.27 / 4095.0) * 5.0;
    char * buf[6];
    gcvt(voltage_meas, 3, buf);
    printf("ADC val: %s\n", buf);
    //HAL_Delay(50);
    //printf("Read DRVreg: 0x%x\n\r", read_REG_DRVSTATUS(&stepper1));
    //printf("GSTAT val: 0x%x\n", read_REG_GSTAT(&stepper1));

    //HAL_TIM_Base_Stop_IT(&htim5);
    stepper_setspeed(70);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);

    HAL_Delay(1500);
    //stepper_enable(&stepper1);
    HAL_TIM_Base_Start_IT(&htim5);
    stepper_setspeed(1000);

    imu_start_update(&imu);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    adc_raw = HAL_ADC_GetValue(&hadc1);
    voltage_meas = (float)adc_raw * (3.27 / 4095.0) * 5.0;
    gcvt(voltage_meas, 3, buf);
    printf("ADC val: %s\n", buf);
    // Send packet can be as simple as
    // Receive buffer
    //lora_receivetest(&lora);
    //uint8_t res = lora_send_packet(&lora, (uint8_t *)"INF", 3);
    //if (res != LORA_OK) {
    //  printf("Send fail: %d\n", res);
      // Send failed
    //}
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);

    //stepper_disable(&stepper1);

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
