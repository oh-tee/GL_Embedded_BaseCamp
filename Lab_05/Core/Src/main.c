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
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
const uint32_t MIN_EXT_T = -24;
const uint32_t MAX_EXT_T = 100;
const uint32_t RANGE_EXT_T = MAX_EXT_T - MIN_EXT_T;
const uint32_t EXT_T_ADC0 = 2507;
const uint32_t AVG_SLOPE = 25;
uint32_t external_temperature = 0;

uint8_t rx;
uint8_t command[64];
uint8_t command_ready = 0;

uint16_t LED_Pins[] = {	LED1_RED_Pin,
						LED2_BLUE_Pin,
						LED3_GREEN_Pin,
						LED4_ORANGE_Pin };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Measure_External_Temperature(void);
void Handle_Command(void);
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
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3, &rx, 1);
  uint32_t current_tick = HAL_GetTick();
  uint32_t previous_tick = 0;
  char temperature_msg[sizeof("Temperature: ___ C\r\n")] = {};
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
	  current_tick = HAL_GetTick();

	  if (current_tick - previous_tick >= 5000)
	  {
		  Measure_External_Temperature();
		  sprintf(temperature_msg, "Temperature: %d C\r\n", external_temperature);
		  HAL_UART_Transmit(&huart3, temperature_msg, sizeof(temperature_msg), 100);
		  previous_tick = current_tick;
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
void Measure_External_Temperature(void)
{
	HAL_StatusTypeDef dac_poll_result;
	uint32_t adc_value;

	HAL_ADC_Start(&hadc1);
	dac_poll_result = HAL_ADC_PollForConversion(&hadc1, 1);
	if (dac_poll_result == HAL_OK)
	{
		adc_value = HAL_ADC_GetValue(&hadc1);
		external_temperature = (EXT_T_ADC0 - adc_value) / AVG_SLOPE;
	}
}

void Handle_Command(void)
{
	int num;
	char cmd[sizeof("toggle")];
	char ok_response_msg[sizeof("OK: led N is off\r\n")] = {0};

	if(sscanf(command, "led %d %s", &num, cmd) != 2)
	{
		HAL_UART_Transmit(&huart3, "ERROR: unknown command\r\n", sizeof("ERROR: unknown command\r\n"), 100);
		return;
	}
	if(num < 1 || num > 4)
	{
		HAL_UART_Transmit(&huart3, "ERROR: wrong led number\r\n", sizeof("ERROR: wrong led number\r\n"), 100);
		return;
	}

	if(strcmp(cmd, "on") == 0)
	{
		HAL_GPIO_WritePin(GPIOD, LED_Pins[num-1], GPIO_PIN_SET);
		sprintf(ok_response_msg, "OK: led %d is on\r\n", num);
	}
	else if(strcmp(cmd, "off") == 0)
	{
		HAL_GPIO_WritePin(GPIOD, LED_Pins[num-1], GPIO_PIN_RESET);
		sprintf(ok_response_msg, "OK: led %d is off\r\n", num);
	}
	else if(strcmp(cmd, "toggle") == 0)
	{
		HAL_GPIO_TogglePin(GPIOD, LED_Pins[num-1]);
		if (HAL_GPIO_ReadPin(GPIOD, LED_Pins[num-1]) == 1)
		{
			sprintf(ok_response_msg, "OK: led %d is on\r\n", num);
		}
		else
		{
			sprintf(ok_response_msg, "OK: led %d is off\r\n", num);
		}
	}
	else
	{
		HAL_UART_Transmit(&huart3, "ERROR: wrong led command\r\n", sizeof("ERROR: wrong led command\r\n"), 100);
		return;
	}
	HAL_UART_Transmit(&huart3, ok_response_msg, sizeof(ok_response_msg), 100);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{

	case BTN1_TOP_Pin:
		HAL_GPIO_TogglePin(GPIOD, LED1_RED_Pin);
		if (HAL_GPIO_ReadPin(GPIOD, LED1_RED_Pin))
		{
			HAL_UART_Transmit(&huart3, "LED1 is on\r\n", sizeof("LED1 is on\r\n"), 10);
		}
		else
		{
			HAL_UART_Transmit(&huart3, "LED1 is off\r\n", sizeof("LED1 is off\r\n"), 10);
		}
		break;

	case BTN2_RIGHT_Pin:
		HAL_GPIO_TogglePin(GPIOD, LED2_BLUE_Pin);
		if (HAL_GPIO_ReadPin(GPIOD, LED2_BLUE_Pin))
		{
			HAL_UART_Transmit(&huart3, "LED2 is on\r\n", sizeof("LED2 is on\r\n"), 10);
		}
		else
		{
			HAL_UART_Transmit(&huart3, "LED2 is off\r\n", sizeof("LED2 is off\r\n"), 10);
		}
		break;

	case BTN3_BOTTOM_Pin:
		HAL_GPIO_TogglePin(GPIOD, LED3_GREEN_Pin);
		if (HAL_GPIO_ReadPin(GPIOD, LED3_GREEN_Pin))
		{
			HAL_UART_Transmit(&huart3, "LED3 is on\r\n", sizeof("LED3 is on\r\n"), 10);
		}
		else
		{
			HAL_UART_Transmit(&huart3, "LED3 is off\r\n", sizeof("LED3 is off\r\n"), 10);
		}
		break;

	case BTN4_LEFT_Pin:
		HAL_GPIO_TogglePin(GPIOD, LED4_ORANGE_Pin);
		if (HAL_GPIO_ReadPin(GPIOD, LED4_ORANGE_Pin))
		{
			HAL_UART_Transmit(&huart3, "LED4 is on\r\n", sizeof("LED4 is on\r\n"), 10);
		}
		else
		{
			HAL_UART_Transmit(&huart3, "LED4 is off\r\n", sizeof("LED4 is off\r\n"), 10);
		}
		break;
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
		strncpy(command, cmd, sizeof(command));
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
