#include "bsp.h"
#include "stm32c0xx_hal.h"

#define EXTICR4 (*(volatile uint32_t)(0x40021800U + 0x60U + 0xCU))
  
static volatile uint16_t *state;
  
//EXTI base address 0x40021800 
// EXTICR4: 0x4002186C
static void BSP_EXTI_BTN_init(void)
{
  GPIO_InitTypeDef init = {0};
  
  init.Pin = GPIO_PIN_13;
  init.Mode = GPIO_MODE_IT_RISING_FALLING;
  init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &init);
  
  //HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  NVIC_SetPriority(EXTI4_15_IRQn, 0);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void BSP_init(volatile uint16_t *s)
{
  uint32_t tmp;
  state = s;
  
  //RCC_IOPENR |= GPIOA_OFF; // enable clock for GPIOA and GPIOC
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
  
  // Define pin5 of GPIOA as output (user LED4)
  tmp = GPIOA->MODER;
  tmp &= ~GPIO_MODER_MODE5; // Clear pin5 port mode
  tmp |= GPIO_MODER_MODE5_0; // set first bit of pin5 port mode GPIOA (01 = output)
  GPIOA->MODER = tmp;
  
  // Define pin13 of GPIOC as an input (user button)
  tmp = GPIOC->MODER;
  tmp &= ~GPIO_MODER_MODE13; // Clear pin13 port mode (00 = input)
  GPIOC->MODER = tmp;
  
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT5; // set pin 5 output type as push-pull (0)
  
  tmp = GPIOA->OSPEEDR;
  tmp &= ~GPIO_OSPEEDR_OSPEED5; // clear pin5 speed
  tmp |= GPIO_OSPEEDR_OSPEED5_0; // set first bit of pin5 speed GPIOA (01 = low speed)
  GPIOA->OSPEEDR = tmp;
  
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD5; // Clear pin5 pullup/pulldown (0 = neither)
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD13; // Clear GPIOC-13 pullup/pulldown (0 = neither)
  
  // Enable interrupt for PC13(button)
  BSP_EXTI_BTN_init();
}

static void BSP_GPIO_toggle(GPIO_TypeDef *loc, uint32_t pin)
{
  volatile uint32_t odr = loc->ODR;
  loc->BSRR = ((odr & pin) << 16) | (~odr & pin);
}

static void BSP_LED4_set(void)
{
  GPIOA->BSRR = GPIO_BSRR_BS5; // set pin 5 (LED)
}

static void BSP_LED4_reset(void)
{
  GPIOA->BSRR = GPIO_BSRR_BR5; // reset pin 5 (LED)
}

void BSP_LED4_toggle(void)
{
  BSP_GPIO_toggle(GPIOA, GPIO_PIN_5);
}

void BSP_delay(volatile uint32_t iter)
{
  while (iter > 0)
    --iter;
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
    BSP_LED4_set();
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
    BSP_LED4_reset();
}

void EXTI4_15_IRQHandler(void)
{
  // PC13 Rising (button released), turn off the LED
  if (EXTI->RPR1 & EXTI_RPR1_RPIF13)
  {
    if (*state == 0)
    {
      BSP_LED4_reset();
    }
    EXTI->RPR1 = EXTI_RPR1_RPIF13;
  }
  // PC13 Falling (button pressed), turn on the LED
  else if (EXTI->FPR1 & EXTI_FPR1_FPIF13)
  {
    if (*state == 0)
    {
      BSP_LED4_set();
    }
    EXTI->FPR1 = EXTI_FPR1_FPIF13;
  }
  else 
  {
    *state = 1;
  } 
    
    
  //BSP_LED4_toggle();
  //BSP_delay(100000);
  //NVIC_ClearPendingIRQ(EXTI4_15_IRQn);
}