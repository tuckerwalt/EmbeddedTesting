#include "bsp.h"
#include "stm32c0xx_hal.h"

#define EXTICR4 (*(volatile uint32_t)(0x40021800U + 0x60U + 0xCU))
  
static volatile BTN_STATE *state;
static volatile uint32_t ticks;
static uint16_t end_delay;
static uint16_t in_delay;

//EXTI base address 0x40021800 
// EXTICR4: 0x4002186C
static void BSP_EXTI_BTN_init(void)
{
  GPIO_InitTypeDef init = {0};
  
  //init.Pin = GPIO_PIN_13;
  //init.Mode = GPIO_MODE_IT_RISING_FALLING;
  //init.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init(GPIOC, &init);
  
  // set EXTI13 to 0x2 (PC13) (see ref manual p.256)
  EXTI->EXTICR[3] |= 2 << 8;
  EXTI->IMR1 |= GPIO_PIN_13;
  EXTI->RTSR1 |= GPIO_PIN_13;
  EXTI->FTSR1 |= GPIO_PIN_13;
  
  //HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  NVIC_SetPriority(EXTI4_15_IRQn, 0);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}

static void BSP_EXTI_BTN_PA15_init(void)
{
  GPIO_InitTypeDef init = {0};
  
  //init.Pin = GPIO_PIN_13;
  //init.Mode = GPIO_MODE_IT_RISING_FALLING;
  //init.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init(GPIOC, &init);
  
  // clear EXTI15 to 0x0 for PA15 (see ref manual p.256)
  EXTI->EXTICR[3] &= ~(0xFF << 24);
  EXTI->IMR1 |= GPIO_PIN_15;
  EXTI->RTSR1 |= GPIO_PIN_15;
  EXTI->FTSR1 |= GPIO_PIN_15;
  
  //HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  NVIC_SetPriority(EXTI4_15_IRQn, 0);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}

static void BSP_OUTPUT_init(GPIO_TypeDef *loc, uint32_t pin)
{
  uint32_t tmp;
  tmp = loc->MODER;
  tmp &= ~(0x3UL << (2 * pin)); // Clear pin port mode
  tmp |= (0x1UL << (2 * pin)); // set first bit of port mode (01 = output)
  loc->MODER = tmp;
  
  loc->OTYPER &= ~(0x1 << pin); // set pin output type as push-pull (0)
  
  tmp = loc->OSPEEDR;
  tmp &= ~(0x3UL << (2 * pin)); // clear pin speed
  tmp |= (0x1UL << (2 * pin)); // set first bit of pin speed (01 = low speed)
  loc->OSPEEDR = tmp;
  
  loc->PUPDR &= ~(0x3UL << (2 * pin)); // Clear pin pullup/pulldown (0 = neither)
}

static void BSP_INPUT_init(GPIO_TypeDef *loc, uint32_t pin)
{
  uint32_t tmp;
  
  tmp = loc->MODER;
  tmp &= ~(0x3UL << (2 * pin)); // Clear pin port mode (00 = input)
  loc->MODER = tmp;
  
  loc->PUPDR &= ~(0x3UL << (2 * pin)); // Clear pin pullup/pulldown (0 = neither)
}

void BSP_init(volatile BTN_STATE *s, uint8_t breadboard_attached)
{
  uint32_t tmp;
  state = s;
  end_delay = 0;
  in_delay = 1;
  
  //RCC_IOPENR |= GPIOA_OFF; // enable clock for GPIOA and GPIOC
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
  
  // PA5(User LED4) output
  BSP_OUTPUT_init(GPIOA, 5);
  
  if (breadboard_attached)
  {
    // PA10(D2) output line to breadboard LED
    BSP_OUTPUT_init(GPIOA, 10);
  }
  
  // Set PC13(User Btn) as input
  BSP_INPUT_init(GPIOC, 13);
  
  if (breadboard_attached)
  {
    BSP_INPUT_init(GPIOA, 15);
  }
  // Enable interrupt for PC13(button)
  BSP_EXTI_BTN_init();
  BSP_EXTI_BTN_PA15_init();
  
  // Enable the systick, pinging every ms
  ticks = 0;
  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000);
}

static void BSP_GPIO_toggle(GPIO_TypeDef *loc, uint32_t pin)
{
  volatile uint32_t odr = loc->ODR;
  loc->BSRR = ((odr & pin) << 16) | (~odr & pin);
}

void BSP_LED4_set(void)
{
  GPIOA->BSRR = GPIO_BSRR_BS5; // set pin 5 (LED)
}

void BSP_LED4_reset(void)
{
  GPIOA->BSRR = GPIO_BSRR_BR5; // reset pin 5 (LED)
}

void BSP_LED4_toggle(void)
{
  BSP_GPIO_toggle(GPIOA, GPIO_PIN_5);
}

void BSP_WHITELED_set(void)
{
  GPIOA->BSRR = GPIO_BSRR_BS10; // set pin 10 (LED)
}

void BSP_WHITELED_reset(void)
{
  GPIOA->BSRR = GPIO_BSRR_BR10; // reset pin 10 (LED)
}

void BSP_WHITELED_toggle(void)
{
  BSP_GPIO_toggle(GPIOA, GPIO_PIN_10);
}

void BSP_delay(volatile uint32_t ms)
{
  in_delay = 1;
  uint32_t current = ticks;
  while (!end_delay && (ticks - current < ms));
  end_delay = 0;
  in_delay = 0;
}

void BSP_BUTTON_pressed(void)
{
  *state = PRESSED;
  //if (in_delay)
    //end_delay = 1;
}

void BSP_BUTTON_released(void)
{
  *state = RELEASED;
  //if (in_delay)
    //end_delay = 1;
}

/*
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
*/
void BSP_IncTick(void)
{
  ++ticks;
}

uint32_t BSP_GetTicks(void)
{
  return ticks;
}

/*void EXTI4_15_IRQHandler(void)
{
  //uint16_t curstate = *state;
  //static uint32_t 
  // PC13 Rising (button released), turn off the LED
  if (EXTI->RPR1 & EXTI_RPR1_RPIF13)
  {
    BSP_BUTTON_released();
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
  if (EXTI->FPR1 & EXTI_FPR1_FPIF13)
  {
    if (curstate < 10)
    {
      *state = (curstate + 1) & 0x3U;
      if (in_delay)
        end_delay = 1;
    }
    
    BSP_BUTTON_pressed();
    EXTI->FPR1 = EXTI_FPR1_FPIF13; 
  }
  else 
  {
    *state = 10;
  } 
    
    
  //BSP_LED4_toggle();
  //BSP_delay(100000);
  //NVIC_ClearPendingIRQ(EXTI4_15_IRQn);
}
*/