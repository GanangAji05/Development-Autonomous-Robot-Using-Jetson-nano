/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "PID.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Encoder(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float demandx;
float demandz;

float demand1;
float demand2;

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t counterValue1 = 0;
int16_t counterValue2 = 0;
int16_t counterValue3 = 0;
int16_t counterValue4 = 0;

volatile long Right_Encoder = 0;
volatile long Left_Encoder = 0;

//Looping
unsigned long currentMillis;
unsigned long previousMillis;

//Encoder variable
float encoder_left_diff;
float encoder_right_diff;

float encoder_left_error;
float encoder_right_error;

float encoder_left_prev;
float encoder_right_prev;

//int angleValue = 0;
char printMessage[200]={'\0'};

uint8_t rx_index;
uint8_t rx_data[2];
uint8_t rx_buffer[100];
uint8_t tranfer_cplt;
uint8_t tx_buffer[27] = "Welcome";


//PID Variable
//Kiri
float Kp = 1;
float Ki = 0;
float Kd = 0;

float Setpoint, Input , Output, Output1a;
float time1 = 10;

//Kanan
float Kp1 = 1;
float Ki1 = 0;
float Kd1 = 0;

float Setpoint1, Input1 , Output1, Output2a;
float time2 = 10;
/* USER CODE END 0 */

uint32_t millis(void){
	return HAL_GetTick();
}
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_TIM12_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

  //Encoder
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

  //UART
  HAL_UART_Receive_IT(&huart2, rx_data, 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  currentMillis = millis();

	  	  	  if (currentMillis - previousMillis >= 10){
	  	  		  previousMillis = currentMillis;

	  	  		  demand1 = demandx - (demandz*0.180);
	  	  		  demand2 = demandx + (demandz*0.180);

	  	  		  encoder_left_diff = Left_Encoder - encoder_left_prev;
	  	  		  encoder_right_diff = Right_Encoder - encoder_right_prev;

	  	  		  encoder_left_error = (demand1*240) - encoder_left_diff;
	  	  		  encoder_right_error = (demand2*240) - encoder_right_diff;

	  	  		  encoder_left_prev = Left_Encoder;
	  	  		  encoder_right_prev = Right_Encoder;

	  	  		  //Motor 1
	  	  		  PID(Kp, Ki, Kd, &Setpoint, &Input, time1, &Output, &Output1a);
	  	  	  	  Setpoint = demand1*240;
	  	  	 	  Input = encoder_left_diff;
	  	  	 	  //PID_Compute(&PID1);

	  	  	 	  //Motor 2
	  	  	 	  PID(Kp1, Ki1, Kd1, &Setpoint1, &Input1, time2, &Output1, &Output2a);
	  	  	 	  Setpoint1 = demand2*240;
	  	  	 	  Input1 = encoder_right_diff;
	  	  	 	  //PID_Compute(&PID2);

	  	  	 	  //Average Encoder
	  	  	 	  Right_Encoder = (counterValue3 + counterValue4)/2;
	  	  	 	  Left_Encoder = (counterValue1 + counterValue2)/2;


	  	 	  //MOTOR KIri
	  	 	  if (Output > 0)
	  	 	  {
	  	 		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	  	 		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Output1a);
	  	 		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Output1a);
	  	 		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	  	 	  }
	  	 	  else if(Output < 0)
	  	 	  {
	  	 		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Output1a);
	  	 		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	  	 		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	  	 		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Output1a);
	  	 	  }
	  	 	  else
	  	 	  {
	  	 		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	  	 		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	  	 		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	  	 		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	  	 	  }


	  //	 	  MOTOR Kanan
	  	 	  if (Output1 > 0)
	  	 	  {
	  	 		  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, Output2a);
	  	 		  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
	  	 		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, Output2a);
	  	 		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
	  	 	  }
	  	 	  else if (Output1 < 0)
	  	 	  {
	  	 		  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
	  	 		  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, Output2a);
	  	 		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
	  	 		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, Output2a);
	  	 	  }
	  	 	  else
	  	 	  {
	  	 		  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
	  	 		  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
	  	 		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
	  	 		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
	  	 	  }

	  sprintf(printMessage, "Output : %.2f, Output1a : %.2f, Output2 : %.2f , Output2a : %.2f \n \r", demandx , demandz, Output1 ,Output2a);
	  HAL_UART_Transmit(&huart2, (uint8_t*)printMessage, strlen(printMessage), 300);

	  Encoder();
	 }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void Encoder(void)
  {
  	counterValue1 = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
  	counterValue2 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  	counterValue3 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  	counterValue4 = (int16_t)__HAL_TIM_GET_COUNTER(&htim8);
}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
    	  if(huart->Instance == USART2)
    	  {
    		  if(rx_data[0]== '1' ){
    			  demandx = 0.5; //in m/s
    			  demandz = 0.5;
    		  }
    		  else if (rx_data[0]== '2' ) {
    			  demandx = -0.5; //in m/s
    			  demandz = -0.5;
    		  }
    		  else if (rx_data[0]== '3' ) {
    			  demandx = 0; //in m/s
    			  demandz = 1;
    		  }
    		  else if (rx_data[0]== '4' ) {
    			  //turn at 1 rad/sec
    			  demandx = 0;
    			  demandz = -1;
    		  }
    		  else if (rx_data[0]== '5' ) {
    			  //turn at 1 rad/sec
    			  demandx = 0.25;
    			  demandz = -0.5;
    		  }
    		  else if(rx_data[0]== '0'){
    			  demandx = 0;
    			  demandz = 0;
    		  }
    		  HAL_UART_Receive_IT(&huart2, rx_data, 1);
    	  }
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
