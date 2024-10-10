/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "bsp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIME_BUTTON_HELD_TO_SWITCH_MS 2000
#define LED_DELAY_INCREMENT_MS 250
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef enum 
{
  IDLE,
  WAITING_ON_BTN_RELEASE,
  BTN_HELD
} BLINK_STATE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
void blink_once(uint16_t blink_state, uint32_t delay)
{
  if (blink_state)
  {
    BSP_LED4_toggle();
    BSP_delay(delay);
  }
}

void blink(volatile BTN_STATE *btn_state)
{
  uint16_t tick_scale = 0;
  BLINK_STATE blink_state = IDLE;
  uint32_t saved_ticks = 0;
  uint16_t bBlinking = 1;
  while (1)
  {
    switch(blink_state)
    {
      case IDLE:
        if (*btn_state == PRESSED)
        {
          blink_state = WAITING_ON_BTN_RELEASE;
          saved_ticks = BSP_GetTicks();
        }
        blink_once(bBlinking, ((tick_scale + 1) * LED_DELAY_INCREMENT_MS) / 2); 
        break;
      case WAITING_ON_BTN_RELEASE:
        if (*btn_state == PRESSED)
        {
          if (BSP_GetTicks() - saved_ticks >= TIME_BUTTON_HELD_TO_SWITCH_MS)
          {
            // swap states
            bBlinking ^= 0x1;
            BSP_LED4_reset();
            blink_state = BTN_HELD;
          }
          else
          {
            blink_once(bBlinking, ((tick_scale + 1) * LED_DELAY_INCREMENT_MS) / 2);
          }
        }
        else // only switch states if we are currently blinking
        {
          if (bBlinking)
            tick_scale = (tick_scale + 1) & 0x3U;
          blink_state = IDLE;
        }
        break;
      case BTN_HELD:
        if (*btn_state == RELEASED)
        {
          blink_state = IDLE;
        }
        blink_once(bBlinking, ((tick_scale + 1) * LED_DELAY_INCREMENT_MS) / 2);
        break;
      default:
        break;
    }
    
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t tmp;
  volatile BTN_STATE state = RELEASED;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //HAL_Init();

  /* USER CODE BEGIN Init */
  
  BSP_init(&state);
  
  // Initialize the EXTI block
  //EXTI->EXTICR[3] |= EXTI_EXTICR4_EXTI13_1;// HUH? pg 256 reference manual
  //HAL_EXTI_SetConfigLine();
  
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
  
  // make sure the pin is reset
  GPIOA->BSRR = GPIO_BSRR_BR5; // reset pin 5 (LED)
  while (1)
  {
    blink(&state);
    /*if (state == 1)
    {
      // Error state, just blink endlessly
      BSP_LED4_toggle();
      BSP_delay(500000);
    }*/
    //GPIOA BSRR = GPIOA(0x50000000) + BSRR(0x18)
    //GPIOA->BSRR = GPIO_BSRR_BS5; // set pin 5 (LED)
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    //BSP_LED4_toggle();
    //BSP_delay(500000);

    //GPIOA->BSRR = GPIO_BSRR_BR5; // reset pin 5 (LED)
    //BSP_LED4_toggle();
    //BSP_delay(1000000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
  return 0;
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
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void EXTI4_15_IRQHandler(void)
{
  // PC13 Rising (button released)
  if (EXTI->RPR1 & EXTI_RPR1_RPIF13)
  {
    BSP_BUTTON_released();
    EXTI->RPR1 = EXTI_RPR1_RPIF13;
  }
  // PC13 Falling (button pressed)
  else if (EXTI->FPR1 & EXTI_FPR1_FPIF13)
  {

    BSP_BUTTON_pressed();
    EXTI->FPR1 = EXTI_FPR1_FPIF13; 
  }
  else
    while (1);
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
