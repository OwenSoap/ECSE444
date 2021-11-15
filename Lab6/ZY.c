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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
#include "stm32l475e_iot01_qspi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUF_SIZE 21504
#define C6SAMPLES 44
#define D6SAMPLES 38
#define E6SAMPLES 35
#define F6SAMPLES 32
#define G6SAMPLES 28
#define A6SAMPLES 25
#define pi 2 * acos(0.0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac1);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int addrNum = 0;
int flag = 0;
int addr = 0;
float32_t sineWave = 0;
uint8_t intSineWave = 0;
uint8_t read[BUF_SIZE];
uint8_t voice[BUF_SIZE];
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DAC1_Init();
	MX_QUADSPI_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	int toneNum = 0;
	float sine = 0;
	float rad = 0;

	BSP_QSPI_Init();
	HAL_TIM_Base_Start_IT(&htim2);

	if (BSP_QSPI_Erase_Block(0) != QSPI_OK) {
		Error_Handler();
	} else {
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 1);
	}

	if (BSP_QSPI_Erase_Block(65536) != QSPI_OK) {
		Error_Handler();
	} else {
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 1);
	}

	for (int tone = 0; tone < 6; tone++) {
		if (tone == 0) {
			for (sine = 0; sine < BUF_SIZE; sine++) {
				rad = sine / (C6SAMPLES / 2) * pi;
				sineWave = arm_sin_f32(rad);
				sineWave = 2.0 / 3.0 * (sineWave + 1) * 127;
				intSineWave = (uint8_t) (sineWave);
				voice[(int) sine] = intSineWave;
				addrNum = (tone * BUF_SIZE + (int) (sine));
			}
			if (BSP_QSPI_Write(voice, (uint32_t) (addrNum - BUF_SIZE + 1),
					(uint32_t) BUF_SIZE) != QSPI_OK) {
				Error_Handler();
			} else {
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 0);
			}
		}

		if (tone == 1) {
			for (sine = 0; sine < BUF_SIZE; sine++) {
				rad = sine / (E6SAMPLES / 2) * pi;
				sineWave = arm_sin_f32(rad);
				sineWave = 2.0 / 3.0 * (sineWave + 1) * 127;
				intSineWave = (uint8_t) (sineWave);
				voice[(int) sine] = intSineWave;
				addrNum = (tone * BUF_SIZE + (int) (sine));
			}
			if (BSP_QSPI_Write(voice, (uint32_t) (addrNum - BUF_SIZE + 1),
					(uint32_t) BUF_SIZE) != QSPI_OK) {
				Error_Handler();
			} else {
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 0);
			}
		}

		if (tone == 2) {
			for (sine = 0; sine < BUF_SIZE; sine++) {
				rad = sine / (D6SAMPLES / 2) * pi;
				sineWave = arm_sin_f32(rad);
				sineWave = 2.0 / 3.0 * (sineWave + 1) * 127;
				intSineWave = (uint8_t) (sineWave);
				voice[(int) sine] = intSineWave;
				addrNum = (tone * BUF_SIZE + (int) (sine));
			}
			if (BSP_QSPI_Write(voice, (uint32_t) (addrNum - BUF_SIZE + 1),
					(uint32_t) BUF_SIZE) != QSPI_OK) {
				Error_Handler();
			} else {
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 0);
			}
		}

		if (tone == 3) {
			for (sine = 0; sine < BUF_SIZE; sine++) {
				rad = sine / (F6SAMPLES / 2) * pi;
				sineWave = arm_sin_f32(rad);
				sineWave = 2.0 / 3.0 * (sineWave + 1) * 127;
				intSineWave = (uint8_t) (sineWave);
				voice[(int) sine] = intSineWave;
				addrNum = (tone * BUF_SIZE + (int) (sine));
			}
			if (BSP_QSPI_Write(voice, (uint32_t) (addrNum - BUF_SIZE + 1),
					(uint32_t) BUF_SIZE) != QSPI_OK) {
				Error_Handler();
			} else {
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 0);
			}
		}

		if (tone == 4) {
			for (sine = 0; sine < BUF_SIZE; sine++) {
				rad = sine / (G6SAMPLES / 2) * pi;
				sineWave = arm_sin_f32(rad);
				sineWave = 2.0 / 3.0 * (sineWave + 1) * 127;
				intSineWave = (uint8_t) (sineWave);
				voice[(int) sine] = intSineWave;
				addrNum = (tone * BUF_SIZE + (int) (sine));
			}
			if (BSP_QSPI_Write(voice, (uint32_t) (addrNum - BUF_SIZE + 1),
					(uint32_t) BUF_SIZE) != QSPI_OK) {
				Error_Handler();
			} else {
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 0);
			}
		}

		if (tone == 5) {
			for (sine = 0; sine < BUF_SIZE; sine++) {
				rad = sine / (A6SAMPLES / 2) * pi;
				sineWave = arm_sin_f32(rad);
				sineWave = 2.0 / 3.0 * (sineWave + 1) * 127;
				intSineWave = (uint8_t) (sineWave);
				voice[(int) sine] = intSineWave;
				addrNum = (tone * BUF_SIZE + (int) (sine));
			}
			if (BSP_QSPI_Write(voice, (uint32_t) (addrNum - BUF_SIZE + 1),
					(uint32_t) BUF_SIZE) != QSPI_OK) {
				Error_Handler();
			} else {
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 0);
			}
		}
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
		if (flag == 1) {
			flag = 0;
			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			addr = toneNum * BUF_SIZE;
			toneNum++;
			if (toneNum > 5) {
				toneNum = 0;
			}
			if (BSP_QSPI_Read(&read, addr, (uint32_t) BUF_SIZE) != QSPI_OK) {
				Error_Handler();
			}
		}
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, read, (uint32_t) 21504,
		DAC_ALIGN_8B_R);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void) {

	/* USER CODE BEGIN DAC1_Init 0 */

	/* USER CODE END DAC1_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC1_Init 1 */

	/* USER CODE END DAC1_Init 1 */
	/** DAC Initialization
	 */
	hdac1.Instance = DAC1;
	if (HAL_DAC_Init(&hdac1) != HAL_OK) {
		Error_Handler();
	}
	/** DAC channel OUT1 config
	 */
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC1_Init 2 */

	/* USER CODE END DAC1_Init 2 */

}

/**
 * @brief QUADSPI Initialization Function
 * @param None
 * @retval None
 */
static void MX_QUADSPI_Init(void) {

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
	if (HAL_QSPI_Init(&hqspi) != HAL_OK) {
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
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1814;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_R_Pin */
	GPIO_InitStruct.Pin = LED_R_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_R_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_G_Pin */
	GPIO_InitStruct.Pin = LED_G_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_G_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac1) {
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 1);
	flag = 1;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
	__BKPT();
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
