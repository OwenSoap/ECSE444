/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef * hdfsdm1_filter0);
//void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef * hdfsdm1_filter0);
//void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac1);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
volatile uint8_t RxReady = 0;
volatile uint8_t RxByte = 0;
#include "stm32l475e_iot01_qspi.h"
#include "arm_math.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_accelero.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
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
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;
DMA_HandleTypeDef hdma_dac_ch2;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId CheckPushButtonHandle;
osThreadId PlayNotesHandle;
osThreadId TransmissionHandle;
osThreadId ReadDataHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
void CheckStates(void const * argument);
void PlayNote(void const * argument);
void UartTransmission(void const * argument);
void StartReadData(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

SemaphoreHandle_t xMutexSending;
SemaphoreHandle_t xMutexReading;
SemaphoreHandle_t xMutexPlaying;
char CopyBuff[30*19+19];
char buffer[30*19+19] = {};
int audio[11] = {-1};
int audioCounter = 0;
//int counter = 0;
//float scale = 97;
//int samples = 22050;
//uint8_t sin1[22050];
//int16_t accXYZ[3];
/*
 * Flash melody
 */
int samples = 22050;
uint8_t sin1[22050];
int counter = 0;
int scale = 127;

/*
 * Accelrator
 */
int16_t accXYZ[3];
char printer[1] = {'p'};
char stopper[1] = {'s'};
char song[12] = {'p'};
int playAble = 0;
int sendAble = 0;
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim2);
//
//void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin);
//void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef * hdfsdm1_filter0);
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
	BSP_ACCELERO_Init();
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_DAC1_Init();
	MX_TIM2_Init();
	MX_QUADSPI_Init();
	MX_USART1_UART_Init();
	MX_I2C2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	BSP_QSPI_Init();
	if(BSP_QSPI_Erase_Block(0) != QSPI_OK)
	{

		Error_Handler();
	}
	else
	{
		// Green Light light once

	}
	if(BSP_QSPI_Erase_Block(65536) != QSPI_OK)
	{

		Error_Handler();
	}
	else
	{
		// Green Light light once

	}
	if(BSP_QSPI_Erase_Block(131072) != QSPI_OK)
	{

		Error_Handler();
	}
	else
	{
		// Green Light light once

	}

	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0);
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 1);

	for (int i = 0; i < samples; i++) {
		float32_t rad = M_PI * (float32_t)i / 22.0;
		float32_t sine = arm_sin_f32(rad);
		//sine = (sine+1)*127 * 2.0 / 3.0;
		sine = (sine+1) * 2.0 / 3.0;
		sin1[i] = (uint8_t)sine;
	}
	BSP_QSPI_Write(sin1, (uint32_t)0, samples);

	for (int i = 0; i < samples; i++) {
		float32_t rad = M_PI * (float32_t)i / 20.0;
		float32_t sine = arm_sin_f32(rad);
		sine = (sine+1)* 2.0 / 3.0;
		sin1[i] = (uint8_t)sine;
	}
	BSP_QSPI_Write(sin1, (uint32_t)samples, samples);

	for (int i = 0; i < samples; i++) {
		float32_t rad = M_PI * (float32_t)i / 18.0;
		float32_t sine = arm_sin_f32(rad);
		sine = (sine+1)* 2.0 / 3.0;
		sin1[i] = (uint8_t)sine;
	}
	BSP_QSPI_Write(sin1, (uint32_t)(samples*2), samples);

	for (int i = 0; i < samples; i++) {
		float32_t rad = M_PI * (float32_t)i / 16.0;
		float32_t sine = arm_sin_f32(rad);
		sine = (sine+1)*2.0 / 3.0;
		sin1[i] = (uint8_t)sine;
	}
	BSP_QSPI_Write(sin1, samples*3, samples);

	for (int i = 0; i < samples; i++) {
		float32_t rad = M_PI * (float32_t)i / 14.0;
		float32_t sine = arm_sin_f32(rad);
		sine = (sine+1)* 2.0 / 3.0;
		sin1[i] = (uint8_t)sine;
	}
	BSP_QSPI_Write(sin1, (uint32_t)(samples*4), samples);

	for (int i = 0; i < samples; i++) {
		float32_t rad = M_PI * (float32_t)i / 12.0;
		float32_t sine = arm_sin_f32(rad);
		sine = (sine+1)* 2.0 / 3.0;
		sin1[i] = (uint8_t)sine;
	}
	BSP_QSPI_Write(sin1, (uint32_t)(samples*5), samples);
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	xMutexSending = xSemaphoreCreateMutex();
	xMutexPlaying = xSemaphoreCreateMutex();
	xMutexReading = xSemaphoreCreateMutex();
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of CheckPushButton */
	osThreadDef(CheckPushButton, CheckStates, osPriorityNormal, 0, 128);
	CheckPushButtonHandle = osThreadCreate(osThread(CheckPushButton), NULL);

	/* definition and creation of PlayNotes */
	osThreadDef(PlayNotes, PlayNote, osPriorityIdle, 0, 128);
	PlayNotesHandle = osThreadCreate(osThread(PlayNotes), NULL);

	/* definition and creation of Transmission */
	osThreadDef(Transmission, UartTransmission, osPriorityIdle, 0, 128);
	TransmissionHandle = osThreadCreate(osThread(Transmission), NULL);

	/* definition and creation of ReadData */
	osThreadDef(ReadData, StartReadData, osPriorityIdle, 0, 128);
	ReadDataHandle = osThreadCreate(osThread(ReadData), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C2;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
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
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void)
{

	/* USER CODE BEGIN DAC1_Init 0 */

	/* USER CODE END DAC1_Init 0 */

	DAC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN DAC1_Init 1 */

	/* USER CODE END DAC1_Init 1 */
	/** DAC Initialization
	 */
	hdac1.Instance = DAC1;
	if (HAL_DAC_Init(&hdac1) != HAL_OK)
	{
		Error_Handler();
	}
	/** DAC channel OUT1 config
	 */
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/** DAC channel OUT2 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DAC1_Init 2 */

	/* USER CODE END DAC1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x10909CEC;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief QUADSPI Initialization Function
 * @param None
 * @retval None
 */
static void MX_QUADSPI_Init(void)
{

	/* USER CODE BEGIN QUADSPI_Init 0 */

	/* USER CODE END QUADSPI_Init 0 */

	/* USER CODE BEGIN QUADSPI_Init 1 */

	/* USER CODE END QUADSPI_Init 1 */
	/* QUADSPI parameter configuration*/
	hqspi.Instance = QUADSPI;
	hqspi.Init.ClockPrescaler = 255;
	hqspi.Init.FifoThreshold = 1;
	hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
	hqspi.Init.FlashSize = 1;
	hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
	hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
	if (HAL_QSPI_Init(&hqspi) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN QUADSPI_Init 2 */

	/* USER CODE END QUADSPI_Init 2 */

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
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1814;
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
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 20000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

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
	huart1.Init.BaudRate = 115200;
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	/* DMA2_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_RED_Pin */
	GPIO_InitStruct.Pin = LED_RED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PUSH_BUTTON_Pin */
	GPIO_InitStruct.Pin = PUSH_BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PUSH_BUTTON_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_GREEN_Pin */
	GPIO_InitStruct.Pin = LED_GREEN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void play() {
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
	BSP_QSPI_Read(sin1, counter*samples, (uint32_t)samples);
	for (int i = 0; i < samples; i++) {
		sin1[i] = sin1[i] * scale;
	}
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sin1, (uint32_t)samples, DAC_ALIGN_8B_R);
	osDelay(500);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_CheckStates */
/**
 * @brief  Function implementing the CheckPushButton thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_CheckStates */
void CheckStates(void const * argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(150);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_PlayNote */
/**
 * @brief Function implementing the PlayNotes thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PlayNote */
void PlayNote(void const * argument)
{
	/* USER CODE BEGIN PlayNote */
	/* Infinite loop */
	for(;;)
	{
		osDelay(100);
		xSemaphoreTake(xMutexPlaying, portMAX_DELAY);
		if(playAble)
		{
			play();
		}
		playAble=0;
		xSemaphoreGive(xMutexPlaying);
	}
	/* USER CODE END PlayNote */
}

/* USER CODE BEGIN Header_UartTransmission */
/**
 * @brief Function implementing the Transmission thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UartTransmission */
void UartTransmission(void const * argument)
{
	/* USER CODE BEGIN UartTransmission */
	/* Infinite loop */
	memset(CopyBuff, 0, strlen(CopyBuff));
	sprintf(CopyBuff, "                              \n                              \n               @@@            \n             *@@,*@           \n             @/   &.          \n             @   @@           \n             & @@@.           \n           .@@@@#             \n         @@@@@/               \n       @@@@   @               \n//////@@@(//(@@@@@@#//////////\n      @%   @@&,& (@@@         \n,,,,,,@(,,*@,,,(,,,#@(,,,,,,,,\n       (%   %   (  #&         \n          %%.   %%%           \n                 ,            \n         .@@#    %            \n        .@@@@/   &            \n          &&, .@.             \n");
	memset(buffer, 0, strlen(buffer));
	strcat(buffer, CopyBuff);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
			(uint16_t) strlen(buffer), 30000);

	for(;;)
	{

		osDelay(100);
		xSemaphoreTake(xMutexSending, portMAX_DELAY);
		if(sendAble)
		{
			if (counter == 1) {
				memset(CopyBuff, 0, strlen(CopyBuff));
				sprintf(CopyBuff, "                              \n                              \n                              \n                              \n                              \n                              \n                              \n                              \n                              \n             &                \n/////////////&////////////////\n             &                \n,,,,,,,,,,,,,&,,,,,,,,,,,,,,,,\n             &                \n             &                \n             %                \n       .@@@@@@                \n      #@@@@@@.                \n         .                    \n");
				memset(buffer, 0, strlen(buffer));
				strcat(buffer, CopyBuff);
				HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
						(uint16_t) strlen(buffer), 30000);
				if(audioCounter < 10)
				{
					song[audioCounter+1] = '1';
				}

			} else if (counter == 2) {
				memset(CopyBuff, 0, strlen(CopyBuff));
				sprintf(CopyBuff, "                              \n                              \n                              \n                              \n                              \n                              \n                              \n                              \n              ,               \n              *               \n//////////////#///////////////\n              *               \n,,,,,,,,,,,,,,/,,,,,,,,,,,,,,,\n              *               \n              *               \n        ,@@@@@@               \n       .@@@@@@                \n                              \n                              \n");
				memset(buffer, 0, strlen(buffer));
				strcat(buffer, CopyBuff);
				HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
						(uint16_t) strlen(buffer), 30000);
				if(audioCounter < 10)
				{
					song[audioCounter+1] = '2';
				}
			} else if (counter == 3){
				memset(CopyBuff, 0, strlen(CopyBuff));
				sprintf(CopyBuff, "                              \n                              \n                              \n                              \n                              \n                              \n                              \n           *                  \n           /                  \n           /                  \n///////////#(/////////////////\n           /                  \n,,,,,,,,,,,(,,,,,,,,,,,,,,,,,,\n           /                  \n    .@@@@@@*                  \n    @@@@@@*                   \n                              \n                              \n                              \n");
				memset(buffer, 0, strlen(buffer));
				strcat(buffer, CopyBuff);
				HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
						(uint16_t) strlen(buffer), 30000);
				if(audioCounter < 10)
				{
					song[audioCounter+1] = '3';
				}
			} else if (counter == 4)
			{
				memset(CopyBuff, 0, strlen(CopyBuff));
				sprintf(CopyBuff, "                              \n                              \n                              \n                              \n                              \n                              \n                %             \n                %             \n                %             \n                %             \n////////////////&/////////////\n                %             \n,,,,,,,,,,,,,,,,%,,,,,,,,,,,,,\n          @@@@@@%             \n         #@@@@@.              \n                              \n                              \n                              \n                              \n");
				memset(buffer, 0, strlen(buffer));
				strcat(buffer, CopyBuff);
				HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
						(uint16_t) strlen(buffer), 30000);
				if(audioCounter < 10)
				{
					song[audioCounter+1] = '4';
				}
			}else if(counter == 5)
			{
				memset(CopyBuff, 0, strlen(CopyBuff));
				sprintf(CopyBuff, "                              \n                              \n                              \n                              \n                              \n                              \n                              \n                              \n                              \n             @@@@(            \n///////////@@@@@@@////////////\n          * #&(               \n,,,,,,,,,,(,,,,,,,,,,,,,,,,,,,\n          *                   \n          *                   \n          *                   \n          *                   \n          *                   \n          .                   \n");
				memset(buffer, 0, strlen(buffer));
				strcat(buffer, CopyBuff);
				HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
						(uint16_t) strlen(buffer), 30000);
				if(audioCounter < 10)
				{
					song[audioCounter+1] = '5';
				}
			}else if(counter == 0)
			{
				memset(CopyBuff, 0, strlen(CopyBuff));
				sprintf(CopyBuff, "                              \n                              \n                              \n                              \n                              \n                              \n                              \n                              \n             @@@@@/           \n           %@@@@@@.           \n///////////&//////////////////\n           %                  \n,,,,,,,,,,,%,,,,,,,,,,,,,,,,,,\n           %                  \n           %                  \n           %                  \n           (                  \n                              \n                              \n");
				memset(buffer, 0, strlen(buffer));
				strcat(buffer, CopyBuff);
				HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
						(uint16_t) strlen(buffer), 30000);
				if(audioCounter < 10)
				{
					song[audioCounter+1] = '0';
				}
			}
			audioCounter++;
			if(audioCounter > 10)
			{
				audioCounter == 100;
			}

		}
		sendAble = 0;
		xSemaphoreGive(xMutexSending);
	}
	/* USER CODE END UartTransmission */
}

/* USER CODE BEGIN Header_StartReadData */
/**
 * @brief Function implementing the ReadData thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartReadData */
void StartReadData(void const * argument)
{
	/* USER CODE BEGIN StartReadData */
	/* Infinite loop */
	for(;;)
	{

		osDelay(10);
		xSemaphoreTake(xMutexSending, portMAX_DELAY);
		xSemaphoreTake(xMutexPlaying, portMAX_DELAY);
		BSP_ACCELERO_AccGetXYZ(accXYZ);
		if ((int) accXYZ[0] > 400) {
			counter = 0;
			playAble = 1;
			sendAble = 1;
			osDelay(100);
		} else if ((int) accXYZ[0] < -400) {
			counter = 1;
			playAble = 1;
			sendAble = 1;
			osDelay(100);
		} else if ((int) accXYZ[1] > 400) {
			counter = 2;
			playAble = 1;
			sendAble = 1;
			osDelay(100);
		} else if ((int) accXYZ[1] < -400) {
			counter = 3;
			playAble = 1;
			sendAble = 1;
			osDelay(100);
		} else if ((int) accXYZ[2] > 1400) {
			counter = 4;
			playAble = 1;
			sendAble = 1;
			osDelay(100);
		} else if ((int) accXYZ[2] < 400) {
			counter = 5;
			playAble = 1;
			sendAble = 1;
			osDelay(100);
		}
		if(HAL_GPIO_ReadPin(PUSH_BUTTON_GPIO_Port, PUSH_BUTTON_Pin) == GPIO_PIN_RESET)
		{
			counter = 0;
			playAble = 0;
			sendAble = 0;
			HAL_UART_Transmit(&huart1, (uint8_t*) song,
					(uint16_t) 12, 30000);
			osDelay(1000);
			song[0] = 'p';
		}
		xSemaphoreGive(xMutexSending);
		xSemaphoreGive(xMutexPlaying);
	}
	/* USER CODE END StartReadData */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
