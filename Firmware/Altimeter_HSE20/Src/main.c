
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include "rocket.h"
#include "LPS22HB.h"
#include "uart_rx_dma.h"
#include "../uart_wrapper/uart_wrapper.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FATFS fatfs;
FIL file_Sys;
FIL file_Baro;
//FIL file_GNSS;

char logdir_Sys[20] = "SYSLOG.TXT";
char logdir_Baro[20] = "BAROLOG.CSV";
//char logdir_GNSS[20] = "GNSSLOG.CSV";

LPS22HB_t lps22hb;

uint8_t isLogBufferEmpty = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM15_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void System_Init(void);
void System_Config_FileDirectory(void);
int System_Check_BootCount(void);
void Inst_Log_Barometer(Rocket_Info_t* info);
//void Inst_Log_GNSS(void);
void LED_Flash_OnePulse(GPIO_TypeDef* port, uint16_t pin, uint16_t span_ms);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

Rocket_Info_t rocket_info;

/*typedef struct {
	uint32_t tim;
	uint32_t press;
	int16_t temp;
} EnvData_Container_t;

typedef struct {
	EnvData_Container_t envdata[32];
} EnvData_DataSet_t;

typedef struct {
	EnvData_DataSet_t packet[4];
	bool isLaunched = false;
	bool isDeployAllowed = false;
	bool isReachedApogee = false;
	bool isDeployTimerElapsed = false;
	bool initiateDeploy = false;
} Rocket_Info_t;

Rocket_Info_t rocket_info;

void Rocket_UpdateStatus_Launched(Rocket_Info_t info);
void Rocket_UpdateStatus_AllowDeploy(Rocket_Info_t info);
void Rocket_UpdateStatus_ReachedApogee(Rocket_Info_t info);
void Rocket_UpdateStatus_DeployTimerElapsed(Rocket_Info_t info);

*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_SDMMC1_SD_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  System_Init();

  //char buff[32];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


	  Rocket_Evaluate_AbleToDeploy_1stStage(&rocket_info);
	  if(Rocket_isAbleToDeploy_1stStage(&rocket_info)) {
		  PIN_H(TRIG_1ST_GPIO_Port, TRIG_1ST_Pin);
		  PIN_H(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

		  Rocket_Evaluate_AbleToDeploy_2ndStage(&rocket_info);
		  if(Rocket_isAbleToDeploy_2ndStage(&rocket_info)) {
			  PIN_H(TRIG_2ND_GPIO_Port, TRIG_2ND_Pin);
			  PIN_H(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
		  } else {
			  PIN_L(TRIG_2ND_GPIO_Port, TRIG_2ND_Pin);
			  PIN_L(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
		  }
	  } else {
		  PIN_L(TRIG_1ST_GPIO_Port, TRIG_1ST_Pin);
		  PIN_L(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	  }

	  //TODO　whileループ内のログの処理どうしよう
	  //→判定関数内でやればよいのでは
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_SDMMC1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 5;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00702991;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SDMMC1 init function */
static void MX_SDMMC1_SD_Init(void)
{

  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 2;

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 19338;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1765;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM15 init function */
static void MX_TIM15_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 39;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 32000;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 39999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 18000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_BLUE_Pin|LED_GREEN_Pin|LED_YELLOW_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG_2ND_Pin|TRIG_1ST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_BLUE_Pin LED_GREEN_Pin LED_YELLOW_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin|LED_GREEN_Pin|LED_YELLOW_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LAUNCHDETECT_EXTI_Pin */
  GPIO_InitStruct.Pin = LAUNCHDETECT_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LAUNCHDETECT_EXTI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG_2ND_Pin TRIG_1ST_Pin */
  GPIO_InitStruct.Pin = TRIG_2ND_Pin|TRIG_1ST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void System_Init(void) {
	FRESULT fRes;

	//	start timer for time record
	HAL_TIM_Base_Start(&htim2);

	uart_init(&huart2);

	Rocket_Init(&rocket_info);

	//	initialize LPS22HB
	LPS22HB_Set_Handle(&lps22hb, &hi2c1);
	LPS22HB_Set_Address(&lps22hb, LPS22HB_ADDR_L);
	LPS22HB_Init(&lps22hb, LPS22HB_ODR_75HZ);
	LPS22HB_Set_FIFO(&lps22hb, LPS22HB_FIFO_ENABLE, LPS22HB_FIFOMODE_STREAM);

	//	mount sd card
	fRes = f_mount(&fatfs, SDPath, 1);
	if(fRes == FR_OK) {
		//	ok
		LED_Flash_OnePulse(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 500);
		xprintf("[FatFs] mount succees\r\n");
	} else {
		//	fail → reset
		LED_Flash_OnePulse(LED_RED_GPIO_Port, LED_RED_Pin, 500);
		xprintf("[FatFs] mount failure\r\n");
		NVIC_SystemReset();
	}

	//	Configure log file directory
	System_Config_FileDirectory();

	//	open file for systemlog
	fRes = f_open(&file_Sys, logdir_Sys, FA_OPEN_APPEND | FA_WRITE);
	if(fRes == FR_OK) {
		//	ok
		LED_Flash_OnePulse(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 500);
		xprintf("[FatFs] syslog fopen succees\r\n");
	} else {
		//	fail → reset
		LED_Flash_OnePulse(LED_RED_GPIO_Port, LED_RED_Pin, 500);
		xprintf("[FatFs] syslog fopen failure\r\n");
		NVIC_SystemReset();
	}

	//	open file for baro/temp logging
	fRes = f_open(&file_Baro, logdir_Baro, FA_OPEN_APPEND | FA_WRITE);
	if(fRes == FR_OK) {
		//	ok
		LED_Flash_OnePulse(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 500);
		xprintf("[FatFs] baro fopen succees\r\n");
	} else {
		//	fail → reset
		LED_Flash_OnePulse(LED_RED_GPIO_Port, LED_RED_Pin, 500);
		xprintf("[FatFs] baro fopen failure\r\n");
		NVIC_SystemReset();
	}

	/*/	open file for gnss logging
	fRes = f_open(&file_GNSS, logdir_GNSS, FA_OPEN_APPEND | FA_WRITE);
	if(fRes == FR_OK) {
		//	ok
		LED_Flash_OnePulse(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 500);
		xprintf("[FatFs] gnss fopen succees\r\n");
	} else {
		//	fail → reset
		LED_Flash_OnePulse(LED_RED_GPIO_Port, LED_RED_Pin, 500);
		xprintf("[FatFs] gnss fopen failure\r\n");
		NVIC_SystemReset();
	}*/

	/* These uart interrupts halt any ongoing transfer if an error occurs, disable them */
	/* Disable the UART Parity Error Interrupt */
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_PE);
	/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_ERR);

	//	uart rx dma config
	msgrx_init(&huart1);

	//	start interrupt timer for reading barometer
	HAL_TIM_Base_Start_IT(&htim6);
	//	for only gnss (not for use now)
	//HAL_TIM_Base_Start_IT(&htim7);

}

void System_Config_FileDirectory(void)
{
	int bootcount;
	char logdir[8];

	bootcount = System_Check_BootCount();
	if ( bootcount < 0 ) {
		xprintf("[LogSys]bootcount check failure!\r\n");

		sprintf(logdir, "LOG");
		xprintf("[LogSys]log directory was set to \"LOG\" \r\n");
	} else {
		xprintf("[LogSys]bootcount is %d\r\n", bootcount);

		sprintf(logdir, "LOG_%03d", bootcount);
		xprintf("[LogSys]log directory was set to \"LOG_%03d\"\r\n", bootcount);
	}

	f_mkdir(logdir);

	sprintf(logdir_Sys, "LOG_%03d/SYSLOG.TXT", bootcount);
	sprintf(logdir_Baro, "LOG_%03d/BAROLOG.CSV", bootcount);
	//sprintf(logdir_GNSS, "LOG_%03d/GNSSLOG.CSV", bootcount);
}

/**
 * 	Check system boot count from config file in sdcard
 */
int System_Check_BootCount(void)
{
    FIL fileRead;
    FIL fileWrite;
    FRESULT res;

    char buff[3] = {};
    int bootcount = 0;


    res = f_open(&fileRead, "CONFIG/BOOTCNT.TXT", FA_READ );
    if ( FR_OK != res ) {

    	//	nothing file
    	res = f_mkdir("CONFIG");

    	if ( FR_OK == res || FR_EXIST == res ) {

        	//	completed make the directory or exist
        	res = f_open(&fileWrite, "CONFIG/BOOTCNT.TXT", FA_CREATE_ALWAYS | FA_WRITE );

        	if ( FR_OK != res ) {

        		//	failed to make file
        		xprintf("filesystem error!\r\n");

            	return -1;
        	} else {

        		//	succeed to make file
        		f_printf(&fileWrite, "%d", bootcount);  //  bootcount = 0
        	}
        }
    } else {

    	//	"BOOTCNT.TXT" file exist
        f_gets((TCHAR *)buff, 4	, &fileRead);
        bootcount = atoi(buff);
        bootcount++;

        f_close(&fileRead);

        res = f_open(&fileWrite, "CONFIG/BOOTCNT.TXT", FA_CREATE_ALWAYS | FA_WRITE );
    }

    f_printf(&fileWrite, "%d", bootcount);
    f_close(&fileWrite);

    return bootcount;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	//	打ち上げ検知時動作
	if(GPIO_Pin == LAUNCHDETECT_EXTI_Pin) {

		//	立ち上がり一回のみ検知、チャタリングで複数回割り込みするのを防ぐ
		if(Rocket_isLaunched(&rocket_info)) {
			//	フラグ更新→打ち上げした
			Rocket_UpdateStatus_Launched(&rocket_info);

			//ログ
			char buff[32];
			sprintf(buff, "%lu Launched\n", TIM2->CNT);
			f_puts(buff, &file_Sys);
			isLogBufferEmpty = 0;

			//	開放タイマースタート
			HAL_TIM_Base_Start_IT(&htim15);
			//	開放ロックタイマースタート
			HAL_TIM_Base_Start_IT(&htim16);
			PIN_H(LED_RED_GPIO_Port, LED_RED_Pin);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if(htim->Instance == TIM6) {
		Inst_Log_Barometer(&rocket_info);
		//Inst_Log_GNSS();
		//f_sync(&file_GNSS);

		//ログSD書き込み
		if(!isLogBufferEmpty) {
			f_sync(&file_Sys);
			isLogBufferEmpty = 1;
		}
	}
	/*
	if(htim->Instance == TIM7) {
		Inst_Log_GNSS();
	}*/

	//開放タイマー
	if(htim->Instance == TIM15) {
		Rocket_UpdateStatus_DeployTimerElapsed(&rocket_info);
		char buff[32];
		sprintf(buff, "%lu DeployTimerElapsed\n", TIM2->CNT);
		f_puts(buff, &file_Sys);
		isLogBufferEmpty = 0;

		PIN_H(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	}

	//開放ロックタイマー
	if(htim->Instance == TIM16) {
		Rocket_UpdateStatus_AllowDeploy(&rocket_info);
		char buff[32];
		sprintf(buff, "%lu AllowDeploy\n", TIM2->CNT);
		f_puts(buff, &file_Sys);
		isLogBufferEmpty = 0;
		//バッファに書き込んだときにフラグを立てて、立ってるときはタイマー割り込み時にf_syncする処理を書く

		PIN_H(LED_RED_GPIO_Port, LED_RED_Pin);
	}
}

/*void Inst_Log_Barometer(void) {
	int i;
	char buff[64];
	EnvData_Container_t sensdata[32];
	uint32_t time_stamp;

	for(i = 0; i < 32; i++) {

		time_stamp = TIM2->CNT;
		LPS22HB_Update_Data(&lps22hb);	//	FIFO whole read
		sensdata[i].tim = time_stamp - 1000000 * (31 - i) / 75;			//	time record of data
		sensdata[i].press = LPS22HB_Get_PressureRaw(&lps22hb);
		sensdata[i].temp = LPS22HB_Get_TemperatureRaw(&lps22hb);
		sprintf(buff,"%d,%lu,%lu,%d\r\n",
			  i,					//	data number of one cycle
			  sensdata[i].tim,
			  sensdata[i].press,
			  sensdata[i].temp
		);
		f_puts(buff, &file_Baro);
		xputs(buff);
	}
	PIN_H(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
	f_sync(&file_Baro);
	PIN_L(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
}*/

void Inst_Log_Barometer(Rocket_Info_t* info) {
	int i,j;
	char buff[64];
	EnvData_DataSet_t sensdata;
	uint32_t time_stamp;
	uint8_t queue;


	for(i = 0; i < 32; i++) {

		time_stamp = TIM2->CNT;
		LPS22HB_Update_Data(&lps22hb);	//	FIFO whole read
		sensdata.envdata[i].tim = time_stamp - 1000000 * (31 - i) / 75;			//	time record of data
		sensdata.envdata[i].press = LPS22HB_Get_PressureRaw(&lps22hb);
		sensdata.envdata[i].temp = LPS22HB_Get_TemperatureRaw(&lps22hb);
		sprintf(buff,"%d,%lu,%lu,%d\r\n",
			  i,					//	data number of one cycle
			  sensdata.envdata[i].tim,
			  sensdata.envdata[i].press,
			  sensdata.envdata[i].temp
		);
		//f_puts(buff, &file_Baro);
		xputs(buff);
	}

	Rocket_EnvData_ShiftDataSet(info);
	Rocket_EnvData_AddNewDataSet(info, sensdata);
	Rocket_AddQueue(info);

	if(Rocket_isLaunched(info)) {
		//	キューの数だけデータセットから過去のセンシングデータを読み出し、SDに書き込む(32個単位で最大128個)
		queue = Rocket_GetQueue(info);
		for(i = 0; i < queue; i++) {
			for(j = 0; j < 32; j++) {
				sprintf(buff,"%d,%lu,%lu,%d\r\n",
						j,					//	data number of one cycle,
						rocket_info.dataset[i].envdata[j].tim,
						rocket_info.dataset[i].envdata[j].press,
						rocket_info.dataset[i].envdata[j].temp
				);
				f_puts(buff, &file_Baro);
			}
		}

		PIN_H(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
		f_sync(&file_Baro);
		PIN_L(LED_BLUE_GPIO_Port, LED_BLUE_Pin);

		Rocket_ResetQueue(info);
	} else {
		//ひたすらなにもせずqueueをためる
	}
}
/*void Inst_Log_GNSS(void) {
	char c[1];

	/　*
	uint8_t i = 0;
	char sentence[128];
	char output_buf[128];
	* /

	f_printf(&file_GNSS, "%lu\r\n", TIM2->CNT);

	while(!msgrx_circ_buf_is_empty()) {
		c[0] = msgrx_circ_buf_get();
		xputs(c);
		f_putc(c[0], &file_GNSS);

		/　*
		sentence[i] = msgrx_circ_buf_get();
		if(sentence[i] == '\n') {
			time_stamp = TIM2->CNT;
			sprintf(output_buf, "%lu,", time_stamp);
			strncat(output_buf, sentence, strlen(sentence));
			xputs(output_buf);
			f_puts(output_buf, &file_GNSS);
			i = 0;
		} else {
			i++;
		}* /
	}
	PIN_H(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	f_sync(&file_GNSS);
	PIN_L(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
}*/

void LED_Flash_OnePulse(GPIO_TypeDef* port, uint16_t pin, uint16_t span_ms) {
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	HAL_Delay(span_ms);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
