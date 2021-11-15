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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_accelero.h"
#include "stdio.h"
#include "string.h"
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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId readDataHandle;
osThreadId transDataHandle;
osThreadId pushCheckHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void StartReadData(void const *argument);
void StartTransData(void const *argument);
void StartCheckButtom(void const *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int16_t magnetoXYZ[3];
int16_t gyroXYZ[3];
int16_t accXYZ[3];
uint32_t timeout = 10;
char buf[55];
char t[20];
char p[20];

char Mx[60];
char My[60];
char Mz[60];

char Ax[60];
char Ay[60];
char Az[60];

float t_data = 0;
float p_data = 0;
int flag = 0;
int read = 1;
int i = 1;

int pushFlag = 0;
int readingData = 0;
int sendingData = 0;

int button = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim2);
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
	BSP_TSENSOR_Init();
	BSP_ACCELERO_Init();
	BSP_MAGNETO_Init();
	BSP_PSENSOR_Init();
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C2_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
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
	/* definition and creation of readData */
	osThreadDef(readData, StartReadData, osPriorityNormal, 0, 128);
	readDataHandle = osThreadCreate(osThread(readData), NULL);
	/* definition and creation of transData */
	osThreadDef(transData, StartTransData, osPriorityIdle, 0, 128);
	transDataHandle = osThreadCreate(osThread(transData), NULL);
	/* definition and creation of pushCheck */
	osThreadDef(pushCheck, StartCheckButtom, osPriorityIdle, 0, 128);
	pushCheckHandle = osThreadCreate(osThread(pushCheck), NULL);
	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();
	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
//
//		if (read == 1) {
//			t_data = BSP_TSENSOR_ReadTemp();
//			p_data = BSP_PSENSOR_ReadPressure();
//			BSP_MAGNETO_GetXYZ(magnetoXYZ);
//			BSP_GYRO_GetXYZ(gyroXYZ);
//			BSP_ACCELERO_AccGetXYZ(accXYZ);
//			read = 0;
//		}
//		if (read == 0) {
//			if (flag == 1) {
//				sprintf(buf, "The temperature now is %d\n", (int) t_data);
//				HAL_UART_Transmit(&huart1, buf, 26, timeout);
//			} else if (flag == 2) {
//				sprintf(buf, "The pressure now is %d\n", (int) p_data);
//				HAL_UART_Transmit(&huart1, buf, 25, timeout);
//			} else if (flag == 3) {
//				sprintf(buf, "The magneto data is X = %d, Y = %d, Z = %d\n",
//						(int) magnetoXYZ[0], (int) magnetoXYZ[1],
//						(int) magnetoXYZ[2]);
//				HAL_UART_Transmit(&huart1, buf, 52, timeout);
//			} else if (flag == 4) {
//				sprintf(buf, "The gyro data is X = %d, Y = %d, Z = %d\n",
//						(int) gyroXYZ[0], (int) gyroXYZ[1], (int) gyroXYZ[2]);
//				HAL_UART_Transmit(&huart1, buf, 49, timeout);
//			} else if (flag == 5) {
//				sprintf(Ax, "Acc X:%d \n ", accXYZ[0]);
//				sprintf(Ay, "Acc Y:%d \n ", accXYZ[1]);
//				sprintf(Az, "Acc Z:%d \n  ", accXYZ[2]);
//				memset(buf, 0, strlen(buf));
//				strcat(buf, Ax);
//				strcat(buf, Ay);
//				strcat(buf, Az);
//				HAL_UART_Transmit(&huart1, buf, 53, timeout);
//			}
//		}
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_I2C2;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
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
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

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
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

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
	htim2.Init.Period = 8000000;
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
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
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

	/*Configure GPIO pin : PB_Pin */
	GPIO_InitStruct.Pin = PB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PB_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_G_Pin */
	GPIO_InitStruct.Pin = LED_G_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_G_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	if (GPIO_Pin == PB_Pin) {
//		HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
//		flag++;
//		if (flag > 5) {
//			flag = 1;
//		}
//	}
//}
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim2) {
//	read = 1;
//	i++;
//	if (i >= 10) {
//		i = 0;
//	}
//}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartReadData */
/**
 * @brief Function implementing the readData thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartReadData */
void StartReadData(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(100);
		if (pushFlag == 1 && sendingData == 0) {
			readingData = 1;
			if (button == 1) {
				t_data = BSP_TSENSOR_ReadTemp();
				readingData = 0;
			} else if (button == 2) {
				p_data = BSP_PSENSOR_ReadPressure();
				readingData = 0;
			} else if (button == 3) {
				BSP_MAGNETO_GetXYZ(magnetoXYZ);
				readingData = 0;
			} else if (button == 4) {
				BSP_GYRO_GetXYZ(gyroXYZ);
				readingData = 0;
			} else if (button == 5) {
				BSP_ACCELERO_AccGetXYZ(accXYZ);
				readingData = 0;
			}
		}

	}
	/* USER CODE END 5 */
}
/* USER CODE BEGIN Header_StartTransData */
/**
 * @brief Function implementing the transData thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTransData */
void StartTransData(void const *argument) {
	/* USER CODE BEGIN StartTransData */
	/* Infinite loop */
	for (;;) {
		osDelay(100);
		if (pushFlag == 1 && readingData == 0) {
			sendingData = 1;
			if (button == 1) {
				sprintf(buf, "The temperature now is %d\n", (int) t_data);
				HAL_UART_Transmit(&huart1, buf, 26, timeout);
				sendingData = 0;

			} else if (button == 2) {
				sprintf(buf, "The pressure now is %d\n", (int) p_data);
				HAL_UART_Transmit(&huart1, buf, 25, timeout);
				sendingData = 0;

			} else if (button == 3) {
				sprintf(buf, "The magneto data is X = %d, Y = %d, Z = %d\n",
						(int) magnetoXYZ[0], (int) magnetoXYZ[1],
						(int) magnetoXYZ[2]);
				HAL_UART_Transmit(&huart1, buf, 52, timeout);
				sendingData = 0;

			} else if (button == 4) {
				sprintf(buf, "The gyro data is X = %d, Y = %d, Z = %d\n",
						(int) gyroXYZ[0], (int) gyroXYZ[1], (int) gyroXYZ[2]);
				HAL_UART_Transmit(&huart1, buf, 49, timeout);
				sendingData = 0;

			} else if (button == 5) {
				sprintf(buf, "The accelero data is X = %d, Y = %d, Z = %d\n",
						(int) accXYZ[0], (int) accXYZ[1], (int) accXYZ[2]);
				HAL_UART_Transmit(&huart1, buf, 53, timeout);
				sendingData = 0;
			}
		}
	}
	/* USER CODE END StartTransData */
}
/* USER CODE BEGIN Header_StartCheckButtom */
/**
 * @brief Function implementing the pushCheck thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCheckButtom */
void StartCheckButtom(void const *argument) {
	/* USER CODE BEGIN StartCheckButtom */
	/* Infinite loop */
	for (;;) {
		osDelay(200);
		if (HAL_GPIO_ReadPin(PB_GPIO_Port, PB_Pin) == 0) {
			pushFlag = 1;
			HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
			if (readingData == 0 && sendingData == 0) {
				button++;
			}
		}
		if (button > 5) {
			button = 1;
		}
	}
	/* USER CODE END StartCheckButtom */
}
/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
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
void Error_Handler(void) {
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

