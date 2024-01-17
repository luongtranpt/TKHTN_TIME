/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

#include <stdio.h>
#include "stdlib.h"
#include "string.h"

#include "Humd.h"
#include "Temp.h"
#include "LCD_I2C.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define START_BYTE 1
#define END_BYTE 2
#define ERROR_FRAME 3
#define UART_BUFFER_LEN 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
void trans(char *s)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}
typedef enum user
{
	Mode_Normal,
	Mode_Humd,
	Mode_Temp
} user_t;
user_t Mode = Mode_Normal;


double Temp = 0.0; 

uint16_t adcv;// bien toan cuc ADC

volatile int a ;
int Humd ;
uint32_t tg;
volatile uint32_t frequency = 0;  // bien tan so toan cuc
volatile int flag_period ;
int hh;
uint8_t rx_data;
int check;

int t1,t2,t;

uint8_t Period = 0;
uint8_t ucRxData;
uint8_t ucRxBuffer[UART_BUFFER_LEN] = {0};
uint8_t ucRxFlag = 0;
uint8_t ucRxCnt = 0;
uint8_t flag_receive = 0;
uint8_t flag_trans = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void HumdTask(void);
void TempTask(void);
void LCDHumdTask(void);
void LCDTempTask(void);
//void LCDTask(void);
void UARTTask(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER TASK BEGIN */


uint8_t data_rx = 0 ;

/* USER TASK END */



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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	/*============= Init peripheral===============*/
	
  HAL_TIM_Base_Start(&htim1);// Bat timer
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
  LCD_I2C_Init();// Khoi dong LCD_I2C 
 // HAL_ADC_Start_IT(&hadc1);// Bat ADC
	HAL_UART_Receive_IT(&huart2,&data_rx,1); // khoi dong ngat nhan uart

	void (*ptr[])(void) = {
		&HumdTask,
		&LCDHumdTask,
		&TempTask,
		&LCDTempTask,
		//&LCDTask,
		&UARTTask
	};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if( flag_period == 1)
		{
			for(int i = 0; i < 5 ;i++)
			{
				(*ptr[i])();

			}
			flag_period = 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 35;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 599;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 35999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 35999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HumdTask(void)
{
	//t1 = (hh*65536 +__HAL_TIM_GetCounter(&htim2));
	HAL_Delay(10);
	Humd = Get_Humd(frequency-1000);
	//t2 = (hh*65536 + __HAL_TIM_GetCounter(&htim2));
	//t = t2 - t1;
}
void TempTask(void)
{

//	t1 = (hh*65536 +__HAL_TIM_GET_COUNTER(&htim3));
	HAL_ADC_Start_IT(&hadc1);
	HAL_Delay(10);
	Temp = temp_convert(adcv);
	HAL_ADC_Stop_IT(&hadc1);
//	t2 = (hh*65536 + __HAL_TIM_GET_COUNTER(&htim3));
//	t = t2-t1;
}
void LCDHumdTask(void)
//void LCDTask(void)
{
//	t1 = (hh*65536 +__HAL_TIM_GET_COUNTER(&htim3));
				LCD_I2C_Set_Cursor(0,0); // Dat vi tri hang 0, cot 0 cho con tro LCD 
			//	LCD_I2C_Clear();// XOA TOAN BO MAN HINH 
				LCD_I2C_Write_String("Humd:");//
				LCD_I2C_WRITE_NUMBER(Humd);// IN RA GIA TRI TAN SO 
				LCD_I2C_Write_String("%");
				HAL_Delay(80);
//	t2 = (hh*65536 + __HAL_TIM_GET_COUNTER(&htim3));
//	t = t2-t1;
}
void LCDTempTask(void)
{
//	t1 = (hh*65536 +__HAL_TIM_GET_COUNTER(&htim3));
				LCD_I2C_Set_Cursor(1,0);// Vi tri hang 0 cot 1 
				LCD_I2C_Write_String("Temp:");

				LCD_I2C_WRITE_Num_f(Temp);// IN RA GIA TRI NHIET DO 
				LCD_I2C_WRITE_DATA(0xDF);
				LCD_I2C_WRITE_DATA('C');
				HAL_Delay(80); // DOI  1S
//	t2 = (hh*65536 + __HAL_TIM_GET_COUNTER(&htim3));
//	t = t2-t1;
}
void UARTTask(void)
{
//	t1 = (hh*65536 +__HAL_TIM_GET_COUNTER(&htim3));
	if(flag_receive == 1)
	{
		if( strcmp( (char*) ucRxBuffer, "&norm*" ) == 0 ) Mode = Mode_Normal;
       else if( strcmp( (char*) ucRxBuffer, "&humd*") == 0 ) Mode = Mode_Humd;
       else if( strcmp( (char*) ucRxBuffer, "&temp*") == 0 ) Mode = Mode_Temp;
       else if( ucRxBuffer[1] == 'T' && ucRxBuffer[2] == '_' )
       {
       // check++;
        uint8_t per[2] = { ucRxBuffer[3], ucRxBuffer[4]};
				Period  = atoi( (char*) per);
       }
       memset((char *)ucRxBuffer, 0, strlen((char *)ucRxBuffer));
	  	 ucRxCnt = 0;
       ucRxFlag = 0;
       HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
			 flag_receive = 0;
	}
	if(flag_trans == 1)
	{
		char str[100];
		if(Period != 0)
		{
			 htim2.Init.Prescaler = Period*10 - 1;
		}
		check =  htim2.Init.Prescaler;
		switch(Mode)
		{
			case Mode_Temp:
				sprintf(str, " Temp = %.1lf\n", Temp);
				HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str),HAL_MAX_DELAY);
				HAL_Delay(100);
			break;
			case Mode_Humd:
				sprintf(str, "Humd = %d\n",Humd);
				HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);
				HAL_Delay(100);
			break;
			case Mode_Normal:
				sprintf(str, "Humd = %d\n",Humd);
				HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);
				HAL_Delay(100);
				sprintf(str, " Temp = %.1lf\n", Temp);
				HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str),HAL_MAX_DELAY);
				HAL_Delay(100);
			break;
			default:
				trans("Nhap sai Mode vui long nhap lai\n");
				//HAL_Delay(5);
		}
		flag_trans = 0;
	}
//	t2 = (hh*65536 +__HAL_TIM_GET_COUNTER(&htim3));
//	t = t2 - t1;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if( GPIO_Pin == GPIO_PIN_15)
	{
		uint32_t timer1_count = __HAL_TIM_GET_COUNTER(&htim1); // lay gia tri moi cua timer
    static uint32_t lastTime = 0;
    uint32_t period = (timer1_count - lastTime);
	
    // tinh tan so 
    frequency = 1000000/period; // Chuyen doi ra HZ
		//tg = tg + 1000000/period;
		//a++;
    lastTime = timer1_count;// gia tri cua cua timer
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc -> Instance == hadc1.Instance)
	{
		adcv = 0;
	  adcv= (uint16_t) HAL_ADC_GetValue(&hadc1);// Lay gia tri cua ADC		
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

//	t1 = (hh*65536 +__HAL_TIM_GET_COUNTER(&htim3));
if(ucRxData == '&')
	{
		memset((char *)ucRxBuffer, 0, strlen((char *)ucRxBuffer));
		ucRxBuffer[0] = ucRxData;
		ucRxFlag = START_BYTE;
		HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
	}
	else if (ucRxFlag == START_BYTE && ucRxCnt < UART_BUFFER_LEN)
	{
		ucRxBuffer[++ucRxCnt] = ucRxData;
		if (ucRxData == '*')
			flag_receive = 1;
		else
			HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
	}
	else if (ucRxCnt == UART_BUFFER_LEN)
	{
		ucRxFlag = ERROR_FRAME;
		memset((char *)ucRxBuffer, 0, strlen((char *)ucRxBuffer));
		ucRxCnt = 0;
		HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
	}
  HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
//	t2 = (hh*65536 +__HAL_TIM_GET_COUNTER(&htim3));
//	t = t2 - t1;
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM2) {
    //b = 1;
		flag_trans = 1;
  }
	if (htim->Instance == TIM3) {
   flag_period = 1;
  }
  /* USER CODE END Callback 1 */
}

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
