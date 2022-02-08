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

//ADC 
uint16_t adc_raw = 0;
int steps = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  HAL_Delay(100);
  RetargetInit(&huart6);

  uint32_t last_run_balance = 0;
  uint32_t last_read_voltage = 0;
  uint32_t last_blink = 0;
  uint32_t last_run_comp = 0;

  MainState_t app_state = STOP;

  MoveDirection_t robot_move = MOVE_STOP;

  lora_sx1276 lora;

  // SX1276 compatible module connected to SPI2, NSS pin connected to GPIO with label LORA_NSS
  uint8_t res = lora_init(&lora, &hspi2, GPIOB, GPIO_PIN_9, LORA_BASE_FREQUENCY_EU);
  if (res != LORA_OK) {
    // Initialization failed
    printf("Lora ei init koodi: %d\n", res);
    app_state = INIT_FAIL;
  }else
    printf("lora init DONE\n");

  bmx160 imu;
  uint8_t imu_status = bmx160_init(&imu, &hi2c1, GPIOA, GPIO_PIN_8);

  while(imu_status > 0){
    printf("IMU init kusi, pysaytetaan: %d\n", imu_status);
    app_state = INIT_FAIL;
    HAL_Delay(1000);
  }
  // Run this only to get IMU calibration parameters
  //bmx160_calibrate(&imu);

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

  //**** PID CONTROLLER INIT ****//
  PID_TypeDef velocityPID = {.KP = 0.45, .KI = 0.25, .KD = 0.005, 
                            .min = -13., .max = 13., .target = 0.};

  PID_TypeDef anglePID= {.KP = 150., .KI = .05 , .KD = 1.2,
                          .min = -1500, .max = 1500, 
                          .target = 1.5};

  //**** LORA RECEIVE START ****//
  uint8_t lora_rx_buffer[128];
  // interrupt on receive
  lora_enable_interrupt_rx_done(&lora);
  // Put LoRa modem into continuous receive mode
  lora_mode_receive_continuous(&lora);

  //**** LORA RECEIVE END ****//
  HAL_Delay(10);

  HAL_UART_Receive_IT(&huart6, uart_rx_buf, 2);

  uint8_t running = 0;
  uint8_t balance_setup = 0;
  // remember stepper "speed"
  int last_step_count = 0;
  // Main loop iteration counter
  uint32_t iter = 0;
  // robot tilt
  float real_pitch = 0;
  // Offsets to motor speed while turning
  int left_motor_offset = 0, right_motor_offset = 0;
  // voltage filtering
  uint16_t battery_meas[4] = {0};
  uint8_t battery_meas_pos = 0;

  app_state = APP_RUN;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(HAL_GetTick() - last_run_comp > CALC_COMP){
      // run complementary filter on raw IMU values.
      real_pitch = complementary_filter(&imu.acc, &imu.gyro, real_pitch);
      // trigger new imu data fetch
      imu_start_update(&imu);
      last_run_comp = HAL_GetTick();
    }

    // main balance loop
    if(HAL_GetTick() - last_run_balance > CALC_BALANCE){
      last_run_balance = HAL_GetTick();

      //printf("pitch: %d \n", (int)real_pitch);

      if(abs((int)(real_pitch - 90.)) < 30 && app_state == APP_RUN)
        running = 1; // start balancing
      else{
        running = 0;
      }
      // start balancing
      if(balance_setup && running){
        enable_stepper(&step1);
        enable_stepper(&step2);
        pid_reset(&anglePID);
        pid_reset(&velocityPID);

        balance_setup = 0;
        step1.step_count = 0;
      // stop
      }else if(!balance_setup && !running){
        disable_stepper(&step1);
        disable_stepper(&step2);
        balance_setup = 1;
      }
      //printf("pitch: %d\n", (int)(real_pitch - 85.));
      if(uart_data_pending){
          uart_data_pending = 0;
          if(uart_rx_buf[0] == '8'){
            robot_move = MOVE_FORWARD;
          }
          if(uart_rx_buf[0] == '2'){
            robot_move = MOVE_REVERSE;
          }
          if(uart_rx_buf[0] == '5'){
            robot_move = MOVE_STOP;
          }

          if(uart_rx_buf[0] == '9'){
            anglePID.KD += 0.5;
          }
          if(uart_rx_buf[0] == '6'){
            robot_move = TURN_RIGHT;
            //anglePID.KD -= 0.5;
          }


          if(uart_rx_buf[0] == '8'){
            velocityPID.KP += 0.01;
          }
          if(uart_rx_buf[0] == '5'){
            velocityPID.KP -= 0.01;
          }

          if(uart_rx_buf[0] == '7'){
            velocityPID.KD += 0.002;
          }
          if(uart_rx_buf[0] == '4'){
            robot_move = TURN_LEFT;
            //velocityPID.KD -= 0.002;
          }


          if(uart_rx_buf[0] == '1'){
            anglePID.KP += 1.;
          }
          if(uart_rx_buf[0] == '0'){
            anglePID.KP -= 1.;
          }

          printf("velocity ");
          print_pid(&velocityPID);
          printf("angle ");
          print_pid(&anglePID);
          HAL_UART_Receive_IT(&huart6, uart_rx_buf, 2);
        }
      // tasapainotus
      if(running){
        // 90 asteen korjaus. Tää pitäs tehä erilailla koska aiheuttaa gimbal lockkia
        anglePID.new = real_pitch - 90.;
        // PID, joka antaa moottorinopeuden
        int target_speed = (int)pid_steps(&anglePID);
        // lasketaan joka toinen kierros kulman korjaus perustuen robotin oikeaan nopeuteen
        if (iter % 2 == 0){
          // Nopeus arvioidaan kalmanfiltterillä koska stepit eivät ole ns oikea fyysinen nopeus
          velocityPID.new = kalmanfilter((float)(step1.step_count - last_step_count));
          last_step_count = step1.step_count;
          if(robot_move == MOVE_FORWARD){
            // unohda integraali kun liikutaan; muistetaan taas kun ollaan paikallaan
            velocityPID.out_sum = 0.;
            if(velocityPID.target < 15)
              velocityPID.target += 0.5;
          }else if(robot_move == MOVE_REVERSE){
            velocityPID.out_sum = 0.;
            if(velocityPID.target > -15)
              velocityPID.target -= .5;
          }else if(robot_move == MOVE_STOP){
            velocityPID.target = 0.;
          }


          // asetetaan kohdekulma välillä +10 -10
          anglePID.target = -pid_steps(&velocityPID);
        }


        switch(robot_move){
          case TURN_LEFT:
            left_motor_offset = 200;
            right_motor_offset = -100;
          break;
          case TURN_RIGHT:
            left_motor_offset = -200;
            right_motor_offset = 100;
          break;

          default:
            left_motor_offset = 0;
            right_motor_offset = 0;

        }

        //printf("target angle: %d current speed: %d speed target: %d angle: %d steps: %d \n", 
        //(int)anglePID.target, (int) velocityPID.new, target_speed, (int)real_pitch, step1.step_count );
        // stepperien nopeuden asettaminen

        stepper_setspeed(&step1, -(target_speed + left_motor_offset));
        stepper_setspeed(&step2, target_speed + right_motor_offset);
        iter++;
      }
    }

    // Read voltage to adc_raw every READ_VOLTAGE
    if(HAL_GetTick() - last_read_voltage > READ_VOLTAGE){
      float voltage_meas = 0;
      if(adc_raw > 0){
        battery_meas[battery_meas_pos] = adc_raw;
        if(battery_meas_pos < 4)
          battery_meas_pos++;
        else
          battery_meas_pos = 0;
        uint8_t meas_count = 0;
        uint32_t batt_total = 0;
        for(int i = 0; i < 4; i++){
          if(battery_meas[i] > 0){
            meas_count++;
            batt_total += battery_meas[i];
          }
        }
        voltage_meas = (float)(batt_total / meas_count) * (3.27 / 4095.0) * 5.0;
        if(voltage_meas < BATTERY_LOW){
          app_state = LOW_BATTERY;
          printf("Low voltage <11.1 \n");
        }
      }
      adc_start();
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


      robot_move = (MoveDirection_t)lora_rx_buffer[0];
      
      
     // printf("'%s'\n", lora_rx_buffer);
      lora_rx_status = LORA_RX_DATA_PENDING;
    }


    // Check if adc (voltage) is correctly read and print it.
    //if(adc_data_status == ADC_DATA_READY){
    //  char * buf[6];
    //  gcvt((float)adc_raw * (3.27 / 4095.0) * 5.0, 3, buf);
      //printf("ADC val: %s\n", buf);
    //  adc_data_status = ADC_DATA_PENDING;
    //}


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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    uart_data_pending = 1;
}

// ajastininterruptit
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
