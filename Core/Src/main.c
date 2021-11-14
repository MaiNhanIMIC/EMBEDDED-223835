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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <strings.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	KHONG_DUOC_NHAN,
	DUOC_NHAN
}button_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int global = 10;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void MY_Handler();

void TIM1_Init()
{
	__HAL_RCC_TIM1_CLK_ENABLE();
	uint16_t* PSC = (uint16_t*)0x40010028;
	uint16_t* ARR = (uint16_t*)0x4001002c;
	uint16_t* CNT = (uint16_t*)0x40010024;
	uint32_t* CR1 = (uint32_t*)0x40010000;
	uint32_t* SR  = (uint32_t*)0x40010010;
	uint32_t* DIER =(uint32_t*)0x4001000c;

	*ARR = 3999;
	*PSC = 15999;
	*DIER |= 1;
	*CR1 |= 1;

}
void my_delay(int time_sec)
{
	uint32_t* SR  = (uint32_t*)0x40010010;
	for(int i = 0; i < time_sec; i++)
	{
		while(!(*SR & 1));
		*SR &= ~1;
	}
}

void my_time1_handle()
{
	uint32_t* SR  = (uint32_t*)0x40010010;
	*SR &= ~1;
}

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
  /* USER CODE BEGIN 2 */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  uint32_t* GPIOD_MODER = (uint32_t*)0x40020c00;
  *GPIOD_MODER |= 0b1010101<<24;		//set output mode for PD12, 13, 14 and 15

  uint32_t* GPIOD_OTYPER= (uint32_t*)0x40020c04;
  *GPIOD_OTYPER &= ~(0b1111<<12);

  uint32_t* GPIOD_ODR= (uint32_t*)0x40020c14;

  /***************** BEGIN SET UP GPIO********************/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  uint32_t* GPIOA_MODER = (uint32_t*)0x40020000;
  *GPIOA_MODER &= ~(0b11);		//set PA0 in INPUT mode

  uint32_t* GPIOA_PUPDR = (uint32_t*)0x4002000c;
  *GPIOA_PUPDR &= ~(0b11);		//set no pul up/down for PA0

  uint16_t* GPIOA_IDR = (uint16_t*)0x40020010;
  button_state_t button_state = 0;
  /***************** END OF SETiNG FOR GPIO********************/

  /***************** BEGIN SET UP EXIT********************/
  uint32_t* EXTICR1 = (uint32_t*)0x40013808;
  *EXTICR1 &= ~(0b1111);
  uint32_t* RTSR = (uint32_t*)0x40013c08;
  *RTSR |= 1;
  uint32_t* IMR = (uint32_t*)0x40013c00;
  *IMR |= 1;
  /***************** END OF SETTING EXIT********************/

  /***************** BEGIN SET UP NVIC (Core ARM)********************/
  uint32_t* NVIC_ISER0 = (uint32_t*)0xe000e100;
  *NVIC_ISER0 |= 1<<6;
  *NVIC_ISER0 |= 1<<25;
  /***************** END OF SETTING NVIC (Core ARM)********************/




  memcpy((void*)0x20000000, (void*)0x08000000, 0x198);


  uint32_t* VTOR = (uint32_t*)0xe000ed08;
  *VTOR = 0x20000000;
  /* USER CODE END 2 */

  uint32_t* Function = (uint32_t*)0x20000058;
  *Function = (int)MY_Handler | 1;
  uint32_t* TIM1_HANDLER = (uint32_t*)0x200000A4;
  *TIM1_HANDLER = (int)my_time1_handle | 1;

  button_state = global;

  TIM1_Init();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


		  *GPIOD_ODR |= (1<<12);	//on LED
		  my_delay(2);
		  *GPIOD_ODR &= ~(1<<12);	//off LED
		  my_delay(2);

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void MY_Handler()
{
	uint32_t* PR = (uint32_t*)0x40013c14;
	*PR |= 1;
}

void EXTI0_IRQHandler()
{

	uint32_t* PR = (uint32_t*)0x40013c14;
	*PR |= 1;
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
