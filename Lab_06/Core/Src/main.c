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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PCA9685.h"
#include <string.h>
#include <stdio.h>
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
/* USER CODE BEGIN PFP */
void Show_Dimming_Effect(void);
void Show_Blink_Effect(void);
void GPIO_Callback(bool state);
void I2C_Transmit(uint8_t devId, uint8_t data[], uint32_t len);
void Handle_Command(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx;
uint8_t command[64];
uint8_t command_ready = 0;
bool in_sleep = false;
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(10);
  LED_Driver_Init(GPIO_Callback, I2C_Transmit);
  HAL_UART_Receive_IT(&huart3, &rx, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  if (command_ready)
	  {
		  command_ready = 0;
		  Handle_Command();
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
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Show_Dimming_Effect(void)
{
	for (int i = 0; i <= 100; i++)
	{
		LED_Driver_Set_PWM_All(i, 0);
		HAL_Delay(50);
	}
	for (int i = 100; i >= 0; i--)
	{
		LED_Driver_Set_PWM_All(i, 0);
		HAL_Delay(50);
	}
}

void Show_Blink_Effect(void)
{
	for (int i = 0; i < 16; i++)
	{
		LED_Driver_Turn_On(i);
		HAL_Delay(50);
	}

	for (int i = 0; i < 4; i++)
	{
		LED_Driver_Turn_On_All();
		HAL_Delay(200);
		LED_Driver_Turn_Off_All();
		HAL_Delay(200);
	}
}

void GPIO_Callback(bool state)
{
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void I2C_Transmit(uint8_t devId, uint8_t data[], uint32_t len)
{
	HAL_I2C_Master_Transmit(&hi2c1, devId, data, len, 1000);
}

void Handle_Command(void)
{
	int num;
	char cmd1[sizeof("disable")];
	char cmd2[sizeof("duty")];

	if (sscanf(command, "%s %s %d", cmd1, cmd2, &num) == 3)
	{
		if (strcmp(cmd1, "set") == 0)
		{
			if (strcmp(cmd2, "duty") == 0)
			{
				if (num < 0 || num > 100)
				{
					HAL_UART_Transmit(&huart3, "ERROR: wrong duty value\r\n", sizeof("ERROR: wrong duty value\r\n"), 100);
					return;
				}
				else
				{
					LED_Driver_Set_PWM_All(num, 0);
					HAL_UART_Transmit(&huart3, "OK\r\n", sizeof("OK\r\n"), 100);
					return;
				}
			}
			else if(strcmp(cmd2, "freq") == 0)
			{
				if (!in_sleep)
				{
					HAL_UART_Transmit(&huart3, "ERROR: enter sleep mode to change frequency\r\n", sizeof("ERROR: enter sleep mode to change frequency\r\n"), 100);
					return;
				}
				else {
					if(num < 24 || num > 1526)
					{
						HAL_UART_Transmit(&huart3, "ERROR: wrong frequency value\r\n", sizeof("ERROR: wrong frequency value\r\n"), 100);
						return;
					}
					else
					{
						LED_Driver_Set_Frequency(num);
						HAL_UART_Transmit(&huart3, "OK\r\n", sizeof("OK\r\n"), 100);
						return;
					}
				}
			}
		}
		else
		{
			HAL_UART_Transmit(&huart3, "ERROR: unknown command\r\n", sizeof("ERROR: unknown command\r\n"), 100);
			return;
		}
	}
	else if(sscanf(command, "%s %d", cmd1, &num) == 2)
	{
		if(strcmp(cmd1, "sleep") == 0)
		{
			if(num == 0 || num == 1)
			{
				LED_Driver_Set_Sleep_Mode(num);
				in_sleep = num;
				HAL_UART_Transmit(&huart3, "OK\r\n", sizeof("OK\r\n"), 100);
				return;
			}
			else
			{
				HAL_UART_Transmit(&huart3, "ERROR: wrong sleep parameter\r\n", sizeof("ERROR: wrong sleep parameter\r\n"), 100);
				return;
			}
		}
		else
		{
			HAL_UART_Transmit(&huart3, "ERROR: unknown command\r\n", sizeof("ERROR: unknown command\r\n"), 100);
			return;
		}
	}
	else if (strcmp(command, "enable") == 0)
	{
		if(!in_sleep)
		{
			LED_Driver_Turn_On_All();
			HAL_UART_Transmit(&huart3, "OK\r\n", sizeof("OK\r\n"), 100);
			return;
		}
		else
		{
			HAL_UART_Transmit(&huart3, "ERROR: unavailable action in sleep mode\r\n", sizeof("ERROR: unavailable action in sleep mode\r\n"), 100);
			return;
		}

	}
	else if (strcmp(command, "disable") == 0)
	{
		LED_Driver_Turn_Off_All();
		HAL_UART_Transmit(&huart3, "OK\r\n", sizeof("OK\r\n"), 100);
		return;
	}
	else
	{
		HAL_UART_Transmit(&huart3, "ERROR: unknown command\r\n", sizeof("ERROR: unknown command\r\n"), 100);
		return;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t cmd[64];
	static uint8_t index;
	cmd[index] = rx;

	if (rx == '\n')
	{
		cmd[index] = 0;
		index = 0;
		strncpy((char *)command, (char *)cmd, sizeof(command));
		command_ready = 1;
	}
	else if (rx == '\r')
	{

	}
	else {
		cmd[index++] = rx;
	}
	HAL_UART_Receive_IT(&huart3, &rx, 1);
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
