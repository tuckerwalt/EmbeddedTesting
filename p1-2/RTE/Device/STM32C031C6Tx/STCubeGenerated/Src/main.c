/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// USER LED -> output GPIOA pin 5
// USER BTN -> input GPIOC pin 13

#define RCC_IOPENR (*(volatile unsigned int *)(0x40021000U + 0x34U))
#define GPIOA_OFF (0x01U << 0)
#define PIN5 (0x01U << 5)
#define BIGPIN5 (0x3U << 10)

void delay(volatile unsigned int iter)
{
  while (iter > 0)
    --iter;
}

// not atomic
// read the output, and flip the selected pin(s)
void toggle_GPIO(GPIO_TypeDef *loc, uint32_t pin)
{
  volatile uint32_t odr = loc->ODR;
  loc->BSRR = ((odr & pin) << 16) | (~odr & pin);
}

int main(void)
{
  uint32_t tmp;
  
  //RCC_IOPENR |= GPIOA_OFF; // enable clock for GPIOA
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  
  // Define pin5 of GPIOA as an output
  tmp = GPIOA->MODER;
  tmp &= ~GPIO_MODER_MODE5; // Clear pin5 port mode
  tmp |= GPIO_MODER_MODE5_0; // set first bit of pin5 port mode GPIOA (01 = output)
  GPIOA->MODER = tmp;
  
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT5; // set pin 5 output type as push-pull (0)
  
  tmp = GPIOA->OSPEEDR;
  tmp &= ~GPIO_OSPEEDR_OSPEED5; // clear pin5 speed
  tmp |= GPIO_OSPEEDR_OSPEED5_0; // set first bit of pin5 speed GPIOA (01 = low speed)
  GPIOA->OSPEEDR = tmp;
  
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD5; // Clear pin5 pullup/pulldown (0 = neither)
  
  // make sure the pin is reset
  GPIOA->BSRR = GPIO_BSRR_BR5; // reset pin 5 (LED)
  while (1)
  {
    //GPIOA BSRR = GPIOA(0x50000000) + BSRR(0x18)
    //GPIOA->BSRR = GPIO_BSRR_BS5; // set pin 5 (LED)
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    toggle_GPIO(GPIOA, GPIO_PIN_5);
    delay(500000);

    //GPIOA->BSRR = GPIO_BSRR_BR5; // reset pin 5 (LED)
    toggle_GPIO(GPIOA, GPIO_PIN_5);
    delay(1000000);
    
  }
  
  return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int dummy(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  //SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int counter = 0;
  
  while (1)
  {
    counter += 3;
    counter *= 3;
    //printf("hi\n");
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  return 0;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
