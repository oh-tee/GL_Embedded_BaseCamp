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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ALERT_POT_PERC		80
#define ALERT_INT_TEMP		30
#define ALERT_EXT_TEMP		30
#define ALERT_LED_PORT		GPIOD
#define ALERT_LED_PIN		GPIO_PIN_14
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const uint32_t MIN_INT_T = -40;
static const uint32_t MAX_INT_T = 125;
static const uint32_t RANGE_INT_T = MAX_INT_T - MIN_INT_T;
static const uint32_t INT_T_OFFSET = 25;
static const uint32_t INT_T_V25 = 760; //in mV

static const uint32_t MIN_EXT_T = -24;
static const uint32_t MAX_EXT_T = 100;
static const uint32_t RANGE_EXT_T = MAX_EXT_T - MIN_EXT_T;
static const uint32_t EXT_T_ADC_0C = 2507;

static const uint32_t AVG_SLOPE = 25;

static const uint32_t alert_blink_intervals[] = {500 - 1, 200 - 1, 100 - 1}; // ms between LED toggles

static uint32_t potentiometer_level = 0;
static int32_t internal_temperature = 0;
static int32_t external_temperature = 0;

static bool potentiometer_alert = 0;
static bool internal_temp_alert = 0;
static bool external_temp_alert = 0;

static bool alert_led_on = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Measure_Potentiometer(void);
void Measure_Internal_Temperature(void);
void Measure_External_Temperature(void);
void Check_Alerts(void);
static void Timer_Callback(TIM_HandleTypeDef *htim);
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
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  htim6.PeriodElapsedCallback = Timer_Callback;
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Measure_Potentiometer();
	  Measure_Internal_Temperature();
	  Measure_External_Temperature();
	  Check_Alerts();
	  HAL_Delay(500);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Measure_Potentiometer(void)
{
	HAL_StatusTypeDef dac_poll_result;
	uint32_t adc_value;

	HAL_ADC_Start(&hadc3);
	dac_poll_result = HAL_ADC_PollForConversion(&hadc3, 1);
	if (dac_poll_result == HAL_OK)
	{
		adc_value = HAL_ADC_GetValue(&hadc3);
		potentiometer_level = adc_value * 100 / 4096; //in percent
		TIM4->CCR4 = potentiometer_level;
		potentiometer_alert = potentiometer_level >= ALERT_POT_PERC;
	}
}

void Measure_Internal_Temperature(void)
{
	HAL_StatusTypeDef dac_poll_result;
	uint32_t adc_value;
	uint32_t voltage;

	HAL_ADC_Start(&hadc1);
	dac_poll_result = HAL_ADC_PollForConversion(&hadc1, 1);
	if (dac_poll_result == HAL_OK)
	{
		adc_value = HAL_ADC_GetValue(&hadc1);
		voltage = 3300 * adc_value / 4096; // in mV
		internal_temperature = (voltage - INT_T_V25) / AVG_SLOPE + INT_T_OFFSET;
		TIM4->CCR2 = (internal_temperature - MIN_INT_T) * 100 / RANGE_INT_T;
		internal_temp_alert = internal_temperature >= ALERT_INT_TEMP;
	}
}

void Measure_External_Temperature(void)
{
	HAL_StatusTypeDef dac_poll_result;
	uint32_t adc_value;

	HAL_ADC_Start(&hadc2);
	dac_poll_result = HAL_ADC_PollForConversion(&hadc2, 1);
	if (dac_poll_result == HAL_OK)
	{
		adc_value = HAL_ADC_GetValue(&hadc2);
		external_temperature = (EXT_T_ADC_0C - adc_value) / AVG_SLOPE;
		TIM4->CCR1 = (external_temperature - MIN_EXT_T) * 100 / RANGE_EXT_T;
		external_temp_alert = external_temperature >= ALERT_EXT_TEMP;
	}
}

void Check_Alerts(void)
{
	static uint8_t prev_lvl = 0;
	uint8_t cur_lvl = potentiometer_alert + internal_temp_alert + external_temp_alert;

	if (cur_lvl > 0)
	{
		alert_led_on = 1;
		if (cur_lvl != prev_lvl)
		{
			TIM6->ARR = alert_blink_intervals[cur_lvl - 1];
			TIM6->CNT = 0;
		}
	}
	else
	{
		alert_led_on = 0;
		HAL_GPIO_WritePin(ALERT_LED_PORT, ALERT_LED_PIN, GPIO_PIN_RESET);
	}

	prev_lvl = cur_lvl;
}

static void Timer_Callback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim6) {
		if (alert_led_on)
		{
			HAL_GPIO_TogglePin(ALERT_LED_PORT, ALERT_LED_PIN);
		}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
