#include "stm32f4xx.h"








/* Include my libraries here */
#include "defines.h"
#include "bmx160.h"
#include <math.h>

#include <stdio.h>
static int per = 16000;
float ref  = -0;
float Kp=0.05,Kd=40,Ki=40;
float interval=0;
int counter = 0;
int counter1 = 0;
float Acc_X = 0;
float Acc_Y = 0;
float Acc_Z = 0;
float gyro_X = 0;
float gyro_Y = 0;
float gyro_Z = 0;
float roll = 0;
float pitch = 0;
float gyroXangle = 0;
float gyroYangle = 1;
float dt = 0;
float dt1=0;
float deg=57.3;
float compAngleX=0;
float compAngleY=0;
static int i=0;
int j=10;
static int fix  = 10000;

float error=0,lasterror=0,duty=0;

// Prototypes
void InitializeTimer(void);
void StartTimer(void);
void EndTimer(void);
int GetTimerValue(void);
void SysTick_Handler(void);


void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

int main(void) {
    TM_MPU6050_t MPU6050_Data0;

    /* Initialize system */

    SystemInit();
    InitializeTimer();

    /* Initialize delay */
  //  TM_DELAY_Init();
    TM_MPU6050_Init(&MPU6050_Data0, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_250s);

	roll  = atan2f(Acc_Y,Acc_Z) * deg;
	pitch = atan2f(-1*Acc_X,Acc_Z) * deg;

	gyro_X=roll;
	gyro_Y=pitch;
	compAngleX=roll;
	compAngleY=pitch;

	StartTimer();



    while (1) {
    	if (i==40 || i>40)
    	{
    		i=0;
        /* Every 500ms */
            /* Reset time */


		// Calculate dt by Getting the timer Value in ms and Converting into seconds

		//EndTimer();
		//j=GetTimerValue();
		//dt = ((1 - GetTimerValue()/72000.000) + i)/1000;
		dt = 40;
		//Delay(5000);
		// Start Timer Again
		//StartTimer();


    	/* If sensor 1 is connected */
                /* Read all data from sensor 1 */
            TM_MPU6050_ReadAccelerometer(&MPU6050_Data0);
            TM_MPU6050_ReadGyroscope(&MPU6050_Data0);

            // TM_MPU6050_ReadAll(&MPU6050_Data0);


            	 Acc_X = 	             MPU6050_Data0.Accelerometer_X;
         	 	 Acc_Y =   			     MPU6050_Data0.Accelerometer_Y;
        		 Acc_Z =        	     MPU6050_Data0.Accelerometer_Z;
				 gyro_X =           	 MPU6050_Data0.Gyroscope_X;
				 gyro_Y =            	 MPU6050_Data0.Gyroscope_Y;
				 gyro_Z =           	 MPU6050_Data0.Gyroscope_Z;

				 //roll=atan2f(Acc_Y,Acc_Z) * deg;
				 pitch=atan2f(-Acc_X,Acc_Z) * deg;

				 //gyroXangle=gyro_X/131.0;;
				 gyroYangle=gyro_Y/131.0;

				 //compAngleX = 0.99 * (compAngleX + gyroXangle * dt/1000) + 0.01 * roll; //change vzriable names of compangles
				 compAngleY = 0.99 * (compAngleY + gyroYangle * dt/1000) + 0.01 * pitch;


				    //PD ALgorithm
				 	error=ref-compAngleY;
				 	interval+=error*dt;
				 	duty= (Kp*error + (Kd*1000*(error)/dt)+Ki*interval/1000);
//				 duty =16000;
				 	if(error>0)
				 	{

				 		if(duty<0)
			 			duty=-duty;


				 		TIM4->CCR2=0;       //13....away from arrow.Motor 2
				 		TIM4->CCR4=0;	  //15....along arrow....Motor1
				 		TIM4->CCR3=duty;     //14...along arrow...Motor2
				 		TIM4->CCR1=duty+250;       //12...away from arrow.Motor 1


				 		//Combine 12,14
				 		//Combine 13,15

				 	}
				 	else if(error<0)
				 	{
				 		if(duty<0)
				 			duty=-duty;

				 		TIM4->CCR2=duty;       //13....away from arrow.Motor 2
				 		TIM4->CCR4=duty+250;	  //15....along arrow....Motor1
				 		TIM4->CCR3=0;     //14...along arrow...Motor2
				 		TIM4->CCR1=0;       //12...away from arrow.Motor 1


				 		//Combine 12,14
				 		//Combine 13,15
				 	}

				 	else if(error==0)
				 	{
				 		TIM4->CCR2=0;       //13....away from arrow.Motor 2
				 		TIM4->CCR4=0;	  //15....along arrow....Motor1
				 		TIM4->CCR3=0;     //14...along arrow...Motor2
				 		TIM4->CCR1=0;       //12...away from arrow.Motor 1
				 		//Combine 12,14
				 		//Combine 13,15
				 	}

				 	lasterror=error;


    } // main if ends
}// while ends

}//main ends



// Timer Start Function
void StartTimer(void){
	  SysTick->LOAD  = 168000 - 1;                                  /* set reload register */
	  NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Systick Interrupt */
	  SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */
	  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
	                   SysTick_CTRL_TICKINT_Msk   |
	                   SysTick_CTRL_ENABLE_Msk;
}

// Timer Count End Function
void EndTimer(void){
	 SysTick->CTRL  = 0;
	 i=0;
}

// To Get Current Count
int GetTimerValue(void){
	 return SysTick->VAL;
}

// To account for Multiple Cycles of Timer
void SysTick_Handler(void)
{
	i++;
}

void InitializeTimer()
{

	/*Structures used in the configuration*/
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable TIM4 Clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* Enable GPIOD Pins that are used for on board LED's */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  //Enabled GPIOB we are going to use PB6 which is linked to TIM4_CH1 according to the
  //documentation
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /* Initialise  pins 13, 14 and 15 D - relating to on board LED's*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

  /* Setup PWM */
  uint16_t PrescalerValue = 0;
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 21000000) - 1;

  /* Setup timer defaults */
  TIM_TimeBaseStructure.TIM_Period = per;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  //notice the number 1 in TIM_OC1Init
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;


  /* Configure timer for PWM - channel 1*/
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* Configure timer for PWM - channel 2 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* Configure timer for PWM - channel 3*/
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* Configure timer for PWM - channel 4 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* Start timer*/
  TIM_Cmd(TIM4, ENABLE);

}