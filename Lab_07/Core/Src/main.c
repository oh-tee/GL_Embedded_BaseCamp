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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "SST25VF016B.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LINES_COUNT 		20
#define MAX_CHAR_IN_LINE 	65
#define SECTOR_BYTES		4095
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Prepare Transmit and Receive Arrays
uint8_t TransmitArray[100] = {0};
uint8_t ReceiveArray[sizeof(TransmitArray)] = {0};
char text[LINES_COUNT][MAX_CHAR_IN_LINE] = 	{	"=========================================\n",
												"From: Olha Tepla, helgaminchenko@gmail.com\n",
												"Mentor: Oleksandr Mordyk, oleksandr.mordyk@globallogic.com\n",
												"Date: 30.11.2021\n",
												"==========================================\n",
												"TIME CAPSULE\n",
												"==========================================\n",
												"Oh, you may not think I'm pretty,\n",
												"But don't judge on what you see,\n",
												"I'll eat myself if you can find\n",
												"A smarter hat than me.\n",
												"You can keep your bowlers black,\n",
												"Your top hats sleek and tall,\n",
												"For I'm the Hogwarts Sorting Hat\n",
												"And I can cap them all.\n",
												"There's nothing hidden in your head\n",
												"The Sorting Hat can't see,\n",
												"So try me on and I will tell you\n",
												"Where you ought to be.\n",
												"==========================================\n"};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Write_Time_Capsule(void);
void Read_Time_Capsule(void);
void Print(char buf[], uint32_t len);
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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // CS = HIGH
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(100);

//  Memory_Driver_Erase_Flash();
//  HAL_Delay(1000);
//
//  Write_Time_Capsule();
//  HAL_Delay(1000);

  Read_Time_Capsule();
  HAL_Delay(1000);
//  volatile uint8_t result = Memory_Driver_Write(test_Tx_buf, 0x00, sizeof(test_Tx_buf));
//  HAL_Delay(1000);

//  Memory_Driver_Read(test_Rx_buf, 0x00, sizeof(test_Rx_buf));
//  HAL_Delay(1000);

////  // Prepare READ_ID command
////  TransmitArray[0] = 0x90;
////  TransmitArray[1] = 0x00;
////  TransmitArray[2] = 0x00;
////  TransmitArray[3] = 0x00;
//
//  // Prepare READ_MEM command
//    TransmitArray[0] = 0x03;
//    TransmitArray[1] = 0x00;
//    TransmitArray[2] = 0x00;
//    TransmitArray[3] = 0x00;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  // CS = LOW
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
//	  HAL_Delay(1);
//	  // Exchange data
////	  HAL_SPI_Transmit(&hspi1, TransmitArray, 4, 100);
////	  HAL_Delay(1);
////	  HAL_SPI_Receive(&hspi1, ReceiveArray, sizeof(ReceiveArray), 100);
////	  HAL_Delay(1);
//	  HAL_SPI_TransmitReceive(&hspi1, TransmitArray, ReceiveArray, sizeof(ReceiveArray), 1000);
//
//	  HAL_Delay(1);
//	  // CS = HIGH
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
//
//	  HAL_Delay(1);
//	  bool foundNewLine = false;
//	  for (uint32_t i = 0; i < sizeof(ReceiveArray); i++) {
//		  if (ReceiveArray[i] == '\n') {
//			  foundNewLine = true;
//			  break;
//		  }
//	  }
//	  if (foundNewLine) {
//		  foundNewLine = false; // place breakpoint here
//	  }
//
//	  *((uint32_t *) &TransmitArray[0]) += 0x1000; // increment address to next 4096 block
//	  TransmitArray[0] = 0x03;

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
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Write_Time_Capsule(void)
{
	uint32_t address = 0x00;

	for (int line = 0; line < LINES_COUNT; line++, address += SECTOR_BYTES)
	{
		Memory_Driver_Write(text[line], address, MAX_CHAR_IN_LINE);
	}
}

void Read_Time_Capsule(void)
{
	char buf[MAX_CHAR_IN_LINE];

	uint32_t address = 0;

	for (int line = 0; line < LINES_COUNT; line++, address += SECTOR_BYTES)
	{
		Memory_Driver_Read(buf, address, MAX_CHAR_IN_LINE);
		HAL_Delay(100);
		Print(buf, MAX_CHAR_IN_LINE);
	}
}

void Print(char buf[], uint32_t len)
{
	for (int i = 0; i < len; i++){
		if (buf[i] == '\n' || buf[i] == '\0')
		{
			break;
		}
		HAL_UART_Transmit(&huart3, &buf[i], 1, 100);
	}
	HAL_UART_Transmit(&huart3, "\r\n", 2, 100);
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
