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

// ADC status
#define ADC_DATA_PENDING 0
#define ADC_DATA_READY 1

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
#define MAX_STEPS_PER_S 400
#define JERK_STEPS_PER_S 50

// how often to run different functions in ms
#define CALC_BALANCE 100 // main loop
#define BLINK_LED 1000
#define READ_IMU 500
#define READ_VOLTAGE 500
#define STEPPER_UPDATE_RATE 4000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t imu_data_status = IMU_DATA_PENDING;
volatile uint8_t lora_rx_status = LORA_RX_DATA_PENDING;
volatile uint8_t adc_data_status = ADC_DATA_PENDING;

 //*** TODO: ***//
 //rewrite this in a struct. DO NOT use the same struct as with tmc2130. This should be separate
//acceleration value
uint16_t steps_per_second_per_second = 1000;
// target steps
uint16_t target_sps_stepper2 = 0, target_sps_stepper1 = 0;
// current steps
uint16_t current_steps_stepper2 = 0, current_steps_stepper1 = 0;
// used for saving the target step value when direction is changed
uint16_t set_steps_stepper1 = 0, set_steps_stepper2 = 0;
// current direction of motor
StepDir stepper1_dir = FORWARD, stepper2_dir = FORWARD;
// new direction after stopping
StepDir stepper1_target_dir = FORWARD, stepper2_target_dir = FORWARD;

uint16_t stepper1_current_angle = 0;
uint16_t stepper1_target_angle = 0;
uint16_t stepper1_steps_to_take = 0;


//ADC 
uint16_t adc_raw = 0;

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
  // prescaler set to 20
  TIM6->PSC = (uint16_t)20 - 1;
  TIM5->PSC = (uint16_t)20 - 1;
  TIM9->PSC = (uint16_t)20 - 1;
}


// Check if we are changing directions (returns true or false)
uint8_t stepper1_directionchange(){
  return stepper1_target_dir ^ stepper1_dir;
}
// Check if we are changing directions (returns true or false)
uint8_t stepper2_directionchange(){
  return stepper2_target_dir ^ stepper2_dir;
}

void stepper1_setspeed(uint16_t speed){
  // Speed in rpm/m
  // fullstep
  if (stepper1_directionchange()){
    set_steps_stepper1 = 200 * (speed/60);
  }else{
    target_sps_stepper1 = 200 * (speed/60);
  }

  HAL_TIM_Base_Start_IT(&htim6);
}

void stepper2_setspeed(uint16_t speed){
  // Speed in rpm/m
  // fullstep
  if (stepper2_directionchange()){
    set_steps_stepper2 = 200 * (speed/60);

  }else {
    target_sps_stepper2 = 200 * (speed/60);
  }
  HAL_TIM_Base_Start_IT(&htim6);
}

//angle in deg 0 - 360
void stepper1_setangle(uint16_t angle){
  uint16_t angle_in_steps = angle * 0.55556;
  stepper1_target_angle = angle_in_steps;
}

//change direction
void stepper1_setdir(StepDir dir){
  // if acceleration is not changed save current to "set" var
  set_steps_stepper1 = target_sps_stepper1;
  // now change target to jerk speed
  target_sps_stepper1 = 50;
  stepper1_target_dir = dir;
  // start acceleration timer if is not running. This does nothing if it is enabled already
  HAL_TIM_Base_Start_IT(&htim6);
}

//change direction
void stepper2_setdir(StepDir dir){
  // if acceleration is not changed save current to "set" var
  set_steps_stepper2 = target_sps_stepper2;
  // now change target to jerk speed
  target_sps_stepper2 = 50;
  stepper2_target_dir = dir;
  // start acceleration timer if is not running. This does nothing if it is enabled already
  HAL_TIM_Base_Start_IT(&htim6);
}

void adc_start(void){

    HAL_ADC_Start_IT(&hadc1);
    //HAL_ADC_PollForConversion(&hadc1, 1000);
    //uint16_t adc_raw = HAL_ADC_GetValue(&hadc1);
    //float voltage_meas = (float)adc_raw * (3.27 / 4095.0) * 5.0;
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
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  HAL_Delay(1000);
  RetargetInit(&huart6);

  uint32_t last_run_balance = 0;
  uint32_t last_read_voltage = 0;
  uint32_t last_blink = 0;
  uint32_t last_read_imu = 0;
  uint32_t last_stepper_update = 0;


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
  
  tmc2130_init(&stepper2, &hspi5,
    tmc2130_2_enable_GPIO_Port, tmc2130_2_dir_GPIO_Port, tmc2130_2_step_GPIO_Port, tmc2130_2_nss_GPIO_Port, 
    tmc2130_2_enable_Pin, tmc2130_2_dir_Pin, tmc2130_2_step_Pin, tmc2130_2_nss_Pin);


  //**** LORA RECEIVE START ****//
  uint8_t lora_rx_buffer[128];
  // interrupt on receive
  lora_enable_interrupt_rx_done(&lora);
  // Put LoRa modem into continuous receive mode
  lora_mode_receive_continuous(&lora);

  //**** LORA RECEIVE END ****//

  HAL_Delay(10);
  
  stepper_enable(&stepper1);
  //stepper_enable(&stepper2);
  // accel is global
  stepper_setaccel(40);
  // start stepper clocks. htim9 is for stepper 2
  HAL_TIM_Base_Start_IT(&htim5);
  //HAL_TIM_Base_Start_IT(&htim9);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // TEST function for stepper. Comment this out when using calc balance
    if(HAL_GetTick() - last_stepper_update > STEPPER_UPDATE_RATE){
      if (target_sps_stepper1 > 299){
        stepper1_setspeed(60);
      }else if(target_sps_stepper1 < 300){
        stepper1_setspeed(300);
      }
      last_stepper_update = HAL_GetTick();
    }

    // run balance every 100 ms (set in CALC_BALANCE as milliseconds)
    if(HAL_GetTick() - last_run_balance > CALC_BALANCE){
      //take last time here
      last_run_balance = HAL_GetTick();

      // Calculate roll&pitch

      // Filter gyro&accelerometer with complementary filter


      // Run PID



      // Set stepper speed & direction if needed

      // EXAMPLE: 
      //stepper1_setspeed(70);
      //stepper2_setspeed(70);
      //stepper1_setdir(BACKWARD);
    }

    // Read voltage to adc_raw every READ_VOLTAGE
    if(HAL_GetTick() - last_read_voltage > READ_VOLTAGE){
      adc_start();
      last_read_voltage = HAL_GetTick();
    }

    // Update imu values
    if( HAL_GetTick() - last_read_imu > READ_IMU){
      imu_start_update(&imu);
      last_read_imu = HAL_GetTick();
    }

    // Check if imu data is ready to be read
    if(imu_data_status == IMU_DATA_READY){
      // update values in imu structure. Read them like imu.acc.x etc...
      imu_end_update(&imu);
      // print values (for debug only)
      imu_print_values(&imu);
      imu_data_status = IMU_DATA_PENDING;
    }
    
    // TODO: move this to a function
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


    // Check if adc (voltage) is correctly read and print it.
    if(adc_data_status == ADC_DATA_READY){
      char * buf[6];
      gcvt((float)adc_raw * (3.27 / 4095.0) * 5.0, 3, buf);
      printf("ADC val: %s\n", buf);
      adc_data_status = ADC_DATA_PENDING;
    }


    // blink one of leds every second
    if(HAL_GetTick() - last_blink > BLINK_LED){
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
      last_blink = HAL_GetTick();
    }


    // Send packet can be as simple as
    // Receive buffer
    //lora_receivetest(&lora);
    //uint8_t res = lora_send_packet(&lora, (uint8_t *)"INF", 3);
    //if (res != LORA_OK) {
    //  printf("Send fail: %d\n", res);
      // Send failed
    //}

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
// tim 5 callback. Käytetään steppaukseen stepper 1 ajurille
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){

  if(htim->Instance == TIM5){ // stepper 1
    HAL_GPIO_TogglePin(tmc2130_1_step_GPIO_Port, tmc2130_1_step_Pin);
  }else if(htim->Instance == TIM9){ // stepper 2
    HAL_GPIO_TogglePin(tmc2130_2_step_GPIO_Port, tmc2130_2_step_Pin);
  }else{ //htim 6
    uint8_t done = 0;
    // update step period for tim5 (stepper 1)
    if(current_steps_stepper1 < target_sps_stepper1){ // if accelerating
      current_steps_stepper1++;

      TIM5->ARR = (uint32_t)((SystemCoreClock / (19 * current_steps_stepper1)));
    }else if(current_steps_stepper1 > target_sps_stepper1){ // deaccelerating
      current_steps_stepper1--;
      TIM5->ARR = (uint32_t)((SystemCoreClock / (19 * current_steps_stepper1)));

    }else if (stepper1_directionchange()){ // check if we have a direction change pending
      // target was set to stopping speed ("jerk speed") where we can reverse stepper direction
      // now set actual target velocity
      target_sps_stepper1 = set_steps_stepper1;
      // and change direction
      stepper1_dir = stepper1_target_dir;
      // and update (direction) output pin
      HAL_GPIO_TogglePin(tmc2130_1_dir_GPIO_Port, tmc2130_1_dir_Pin);
    }else{
      // Speed achieved
      done++;
      //HAL_TIM_Base_Stop_IT(&htim6);
    }

    // update step period for tim9 (stepper 2)
    if(current_steps_stepper2 < target_sps_stepper2){ // if accelerating
      current_steps_stepper2++;

      TIM9->ARR = (uint32_t)((SystemCoreClock / (19 * current_steps_stepper2)));
    }else if(current_steps_stepper2 > target_sps_stepper2){ // deaccelerating
      current_steps_stepper2--;
      TIM9->ARR = (uint32_t)((SystemCoreClock / (19 * current_steps_stepper2)));
    }else if (stepper2_directionchange()){ // check if we have a direction change pending
      // target was set to stopping speed ("jerk speed") where we can reverse stepper direction
      // now set actual target velocity
      target_sps_stepper2 = set_steps_stepper2;
      // and change direction
      stepper2_dir = stepper2_target_dir;
      // and update (direction) output pin
      HAL_GPIO_TogglePin(tmc2130_2_dir_GPIO_Port, tmc2130_2_dir_Pin);
    }else{
      // Speed achieved
      done++;
    }
    if(done == 2)
      HAL_TIM_Base_Stop_IT(&htim6);
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
  adc_raw = HAL_ADC_GetValue(&hadc1);
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
