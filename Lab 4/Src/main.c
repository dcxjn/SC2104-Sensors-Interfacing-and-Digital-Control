/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <math.h>
#include <stdlib.h>
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
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#include "oled.h"
#include <math.h>

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t counter = 0;    // Timer 2 counter
int16_t count = 0;       // Convert counter to signed value
int16_t no_of_tick = 50; // number of tick used in SysTick to calculate speed, in msec
int16_t speed =0;        // speed in term of number of edges detected per Systick
int16_t rpm = 0;         // speed in rpm number of count/sec * 60 sec  divide by 260 count per round
int start=0;             // use to start stop the motor
int16_t pwmVal=0;       //pwm value to control motor speed
int16_t pwmMax = (7200 -1000); // Maximum PWM value = 7200 keep the maximum value too 7000
int err; // status for checking return

char buf[10];    //buffer for displaying values on OLED display

int16_t position = 0;    // position of the motor (1 rotation = 260 count)
extern int16_t oldpos;    // // see SysTick_Handler in stm32f4xx_it.c
int16_t angle = 0;        // angle of rotation, in degree resolution = 360/260
int16_t target_angle = 0; // target angle of rotation,
int16_t position_target; // target position
int16_t error;           // error between target and actual
int32_t error_area = 0;  // area under error - to calculate I for PI implementation
int32_t error_old, error_change;
float_t error_rate; // to calculate D for PID control
int32_t millisOld, millisNow, dt; // to calculate I and D for PID control
int16_t Kp = 0;
float_t Kd = 0;
float_t Ki = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	counter = __HAL_TIM_GET_COUNTER(htim);
	count = (int16_t)counter;
	position = count/4;  //x1 Encoding
	angle = count/2; // x2 encoding
}

void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin ) {

	if ( GPIO_Pin == USER_PB_Pin) {
		// toggle LED
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10); // LED
		if (start == 0){
			start = 1;
		    // reset all value to Zero
		    TIM2->CNT = 0; // Timer Counter Value
		    speed = 0;
		    position = 0;  // see SysTick_Handler in stm32f4xx_it.c
		    oldpos = 0; // see SysTick_Handler in stm32f4xx_it.c
		    angle = 0;
		    pwmVal = 0;
		    }
		else
			start = 0;
 	    }
}


void Motor_direction(uint8_t forward) {
	if (forward){// move forward
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel A - forward
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel B- forward
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel D- forward
		HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_RESET);
	  }
	else { // reverse
	    HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel A - reverse
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel B - reverse
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel D- reverse
		HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_SET);
	}
}


void check_Hall(void){
     while(1){ //checking hall sensor count per revolution on  MotorD connector
		  OLED_ShowString(0, 0, "Encoder Ticks ");
		  //OLED_ShowString(0, 10, "(per turn)");
		  sprintf(buf, "%6d", ((TIM2->CNT)>>2)); // divide by 4 due to 4 edges/interrupts per pole transition
		  OLED_ShowString(25, 10, buf);
		  angle = (int)(position*360/260);
		  OLED_ShowString(0, 30, "Angle = ");
		  sprintf(buf, "%4d", angle);//Hall Sensor = 26 poles/13 pulses, DC motor = 20x13 = 260 pulse per revolution
		  OLED_ShowString(55, 30, buf);
		  OLED_ShowString(0, 50, "Speed =    rpm"); // see SysTick_Handler in stm32f4xx_it.c
		  sprintf(buf, "%4d", speed);//Hall Sensor = 18 poles/9pulses, DC motor = 20x9 = 180 pulse per revolution
		  OLED_ShowString(55, 50, buf);
		  OLED_Refresh_Gram();
	  }
}

int16_t PID_Control(){
	  //Control Loop
	  if (abs(error)>2){ //more than 2 degree difference
		  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10); //Buzzer
    	  angle = (int)(position*360/265);  // supposed to be 260 tick per revolution?
  	      error = target_angle - angle;

        if (error > 0)
      	  Motor_direction(1); //forward
        else
      	  Motor_direction(0); //reverse direction

        millisNow = HAL_GetTick();
        dt = (millisNow - millisOld); // time elapsed in millisecond
        millisOld = millisNow; // store the current time for next round

        error_area = error_area + error*dt; // area under error for Ki

        error_change = error - error_old; // change in error
  	    error_old = error; //store the error for next round
        error_rate = error_change/dt; // for Kd

		pwmVal = (int)(error*Kp + error_area*Ki + error_rate*Kd);  // PID

  	  //pwmVal = 2000;   // overwrite PID above, minimum pwmVal = 1000

		if (pwmVal > pwmMax)  // Clamp the PWM to its maximum value
		   pwmVal = pwmMax;

		return(pwmVal);
    	//__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,pwmVal); // output the valie
	    } // if loop
}

void serial_uart(){
	 // send the values to serial port for display
		  angle = (int)(position*360/265);
		  sprintf(buf, "%4d", angle); //Hall Sensor = 26 poles/13 pulses, DC motor = 20x13 = 260 pulse per revolution
	      OLED_ShowString(50, 10, buf);
	      //send to serial port
	      HAL_UART_Transmit(&huart3, buf, 4, HAL_MAX_DELAY); // Send to Uart3 USB port

	      buf[0]=',';
	      HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port
	      sprintf(buf, "%4d", target_angle);
	   	  HAL_UART_Transmit(&huart3, buf, 4, HAL_MAX_DELAY); // Send to Uart3 USB port

	   	  buf[0]=',';
	   	  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port
	   	  sprintf(buf, "%4d", error);
	   	  HAL_UART_Transmit(&huart3, buf, 4, HAL_MAX_DELAY); // Send to Uart3 USB port

	   	  buf[0]=',';
	      HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port
	      sprintf(buf, "%4d", pwmVal);
	      HAL_UART_Transmit(&huart3, buf, 4, HAL_MAX_DELAY); // Send to Uart3 USB port
	      OLED_ShowString(40, 20, buf);
	      OLED_Refresh_Gram();

	   	  buf[0]=',';
	   	  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port
	   	  sprintf(buf, "%5d", error_area);
	   	  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port

	   	  buf[0]=',';
	   	  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port
	   	  sprintf(buf, "%3d", err);
	   	  HAL_UART_Transmit(&huart3, buf, 3, HAL_MAX_DELAY); // Send to Uart3 USB port

	   	  buf[0]=',';
	   	  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port
	   	  sprintf(buf, "%3d", error_rate);
	   	  HAL_UART_Transmit(&huart3, buf, 3, HAL_MAX_DELAY); // Send to Uart3 USB port


	   	  buf[0] = '\r';
	   	  buf[1] = '\n';  // move to next line on serial port
	   	  HAL_UART_Transmit(&huart3, buf, 2, HAL_MAX_DELAY); // Send through BT @9600
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t *oled_buf; // buffer to store value to be display on OLED
  int8_t i; // loop counter

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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //start TIM8-PWM to drive the DC motor
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  // on Motor A interface
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);  // on Motor D interface


  // start TIM2-Encoder to read Motor rotation in interrupt mode
  // Hall sensors produce 13 ticks/counts per turn, gear ratio = 20
  // 260 count per rotation of output (wheel)
  // 360 degree = 260 ticks/counts
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  rpm = (int)((1000/no_of_tick) * 60/260);  // For calculating motor rpm - by multiplying it with speed value

  OLED_Init();
  OLED_ShowString(10, 5, "SC2104/CE3002"); // show message on OLED display at line 10)

  oled_buf = "Lab 4"; // anther way to show message through buffer
  OLED_ShowString(40,30, oled_buf); //another message at line 50

  oled_buf = "DC Motor"; // anther way to show message through buffer
  OLED_ShowString(30,50, oled_buf); //another message at line 50

  uint8_t sbuf[] = "SC2104\n\r";  // send to serial port
  HAL_UART_Transmit(&huart3, sbuf, sizeof(sbuf), HAL_MAX_DELAY); // Send through Serial Port @115200
  //HAL_UART_Transmit(&huart2, sbuf, sizeof(sbuf), HAL_MAX_DELAY); // Send through BT @9600

  OLED_Refresh_Gram();
  HAL_Delay(2000); // pause for 2 second to show message
  OLED_Clear(); // get display ready

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  start = 0;
  angle = 0;
  target_angle = 720; // rotate 720 degree
  error = target_angle - angle;
  error_old = 0;
  error_area = 0;

  // motor drive here
  OLED_Clear();
  OLED_ShowString(0, 0, "Target: ");
  OLED_ShowString(0, 10, "Rotated:");
  OLED_ShowString(0, 20, "PWM:+");
  sprintf(buf, "%4d", target_angle);//Hall Sensor = 26 poles/13 pulses, DC motor = 20x13 = 260 pulse per revolution
  OLED_ShowString(50, 0, buf);
  OLED_Refresh_Gram();

  Kp = 20;       // 10
  Ki = 0.001;   // 0.001
  Kd = 0000;
  millisOld = HAL_GetTick(); // get time value before starting - for PID

  start = 1; // do a step response upon reset and power up

  while (1)
  {
	  if (start==0){ // reset and wait for the User PB to be pressed
    	  pwmVal = 0;
      	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,pwmVal);
    	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10); // LED
    	  //OLED_Clear(); // get display ready
    	  OLED_ShowString(15, 40, "Press User"); // show message on OLED display at line 40)
    	  OLED_ShowString(0, 50, "button to start"); // show message on OLED display at line 50)
    	  OLED_Refresh_Gram();
    	  err = 0;// for checking whether error has settle down near to zero
    	  angle = 0;
    	  error_old = 0;
    	  error_area = 0;
    	  error = target_angle - angle;
	      }
	  while (start==0){ //wait for the User PB to be pressed
    	  HAL_Delay(500);
    	  millisOld = HAL_GetTick(); // get time value before starting - for PID
		  }

	  pwmVal = PID_Control(); // call the PID control loop calculation
	  pwmVal = 2000;          // overwrite PID control above, minimum pwmVal = 1000?
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,pwmVal); // output PWM waveform to drive motor

	  if (abs(error) < 3){ // error is less than 3 deg
	      err++; // to keep track how long it has reached steady state
	      angle = (int)(position*360/260);  //calculate the angle
	      error = target_angle - angle; // calculate the error
	      }

	  serial_uart(); // send the various data to the serial port for display

      if (err > 5) { // error has settled to within the acceptance ranges
	   	 pwmVal = 0; //stop
   	     __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,pwmVal);
   	     //continue to send the data values for display
   	     for (i=0; i<50; i++)
   	    	serial_uart();

   	     start = 0;  // wait for PB to restart
		 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10); //Buzzer On
   	     HAL_Delay(500);
		 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10); //Buzzer Off
         }

      //HAL_Delay(50);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  } // while
  /* USER CODE END 3 */
}// main

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED1_Pin|OLED2_Pin|OLED3_Pin|OLED4_Pin
                          |LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Buzzer_Pin|DIN1_Pin|DIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED1_Pin OLED2_Pin OLED3_Pin OLED4_Pin
                           LED_Pin */
  GPIO_InitStruct.Pin = OLED1_Pin|OLED2_Pin|OLED3_Pin|OLED4_Pin
                          |LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin DIN1_Pin DIN2_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|DIN1_Pin|DIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_PB_Pin */
  GPIO_InitStruct.Pin = USER_PB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_PB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ch_B_Pin */
  GPIO_InitStruct.Pin = Ch_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ch_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ch_A_Pin */
  GPIO_InitStruct.Pin = Ch_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ch_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/*
void OLED_show(void *argument, int y, int x) // display message on OLED panel
{
	//uint8_t hello[20]="Hello World";
	OLED_Init();
	OLED_Display_On();
//	OLED_ShowString(10,10,argument);
	OLED_ShowString(y, x, argument);
	OLED_Refresh_Gram();
}
*/

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
