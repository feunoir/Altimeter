/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "LPS22HB.h"
#include "uart_rx_dma.h"
#include "../uart_wrapper/uart_wrapper.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint32_t tim;
	uint32_t baro;
	int16_t temp;
} EnvData_Container_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
FATFS fatfs;
FIL fileSys;
FIL fileEnv;
//FIL file_GNSS;

char logdir_Sys[20] = "SYSLOG.TXT";
char logdir_Baro[20] = "BAROLOG.CSV";
//char logdir_GNSS[20] = "GNSSLOG.CSV";

LPS22HB_t lps22hb;
EnvData_Container_t envdata;
__IO ITStatus isEnvDataReady = RESET;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void System_Init(void);
void System_Config_FileDirectory(void);
int System_Get_BootCount(void);
void LED_Flash_OnePulse(GPIO_TypeDef* port, uint16_t pin, uint16_t span_ms);

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
  char buff[64];
  uint32_t timA, timB, timC;
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
  MX_I2C1_Init();
  MX_SDMMC1_SD_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  HAL_NVIC_DisableIRQ(EXTI3_IRQn);
  System_Init();
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(isEnvDataReady == SET) {
    	timA = TIM2->CNT;
		isEnvDataReady = RESET;

		HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);

		LPS22HB_Update_Data(&lps22hb);
		envdata.baro = LPS22HB_Get_PressureRaw(&lps22hb);
		envdata.temp = LPS22HB_Get_TemperatureRaw(&lps22hb);

		sprintf(buff, "%lu,%lu,%d\r\n", envdata.tim, envdata.baro, envdata.temp);
		f_puts(buff, &fileEnv);
		xputs(buff);
		timB = TIM2->CNT;	//　	about 0.7ms from timA, sometimes 3ms
		f_sync(&fileEnv);
		timC = TIM2->CNT;	//	about 6.5ms from timA, sometimes 20ms


		xprintf("process time: %lu %lu\r\n", timB-timA, timC-timA);
    }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the CPU, AHB and APB busses clocks 
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
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 2;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_BLUE_Pin|LED_GREEN_Pin|LED_YELLOW_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_BLUE_Pin LED_GREEN_Pin LED_YELLOW_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin|LED_GREEN_Pin|LED_YELLOW_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LPS22HB_DRDY_Pin */
  GPIO_InitStruct.Pin = LPS22HB_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LPS22HB_DRDY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */
void System_Init(void) {
	FRESULT fRes;

	//	start timer for time record
	HAL_TIM_Base_Start(&htim2);

	uart_init(&huart2);

	//	initialize LPS22HB
	LPS22HB_Set_Handle(&lps22hb, &hi2c1);
	LPS22HB_Set_Address(&lps22hb, LPS22HB_ADDR_L);
	LPS22HB_Init(&lps22hb, LPS22HB_ODR_1HZ);
	LPS22HB_Set_FIFO(&lps22hb, LPS22HB_FIFO_DISABLE, LPS22HB_FIFOMODE_BYPASS);
	LPS22HB_Set_DRDY(&lps22hb, LPS22HB_DRDY_ENABLE);

	LPS22HB_Update_Data(&lps22hb);

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
	fRes = f_open(&fileSys, logdir_Sys, FA_OPEN_APPEND | FA_WRITE);
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
	fRes = f_open(&fileEnv, logdir_Baro, FA_OPEN_APPEND | FA_WRITE);
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
	//__HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
	//HAL_TIM_Base_Start_IT(&htim6);
	//	for only gnss (not for use now)
	//HAL_TIM_Base_Start_IT(&htim7);

}

void System_Config_FileDirectory(void)
{
	int bootcount;
	char logdir[8];

	bootcount = System_Get_BootCount();
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
int System_Get_BootCount(void)
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
	if(GPIO_Pin == LPS22HB_DRDY_Pin) {
		// TODO: TIM2を読むコード+LPS22HBから気圧を取得する？
		envdata.tim = TIM2->CNT;
		isEnvDataReady = SET;
	}
}

void LED_Flash_OnePulse(GPIO_TypeDef* port, uint16_t pin, uint16_t span_ms) {
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	HAL_Delay(span_ms);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
