#include "bsp.h"
#include "stm32c0xx_hal.h"

void BSP_init(void)
{
  uint32_t tmp;
  
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
}

static void BSP_GPIO_toggle(GPIO_TypeDef *loc, uint32_t pin)
{
  volatile uint32_t odr = loc->ODR;
  loc->BSRR = ((odr & pin) << 16) | (~odr & pin);
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