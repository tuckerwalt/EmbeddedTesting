#include "main.h"
#include "bsp.h"
#include "tasks.h"

#define TIME_BUTTON_HELD_TO_SWITCH_MS 2000
#define LED_DELAY_INCREMENT_MS 250
#define BREADBOARD_ATTACHED 1

static volatile uint8_t whiteled_blinking;
static volatile uint8_t led4_blinking;
static volatile uint8_t whiteled_tick_scale;
static volatile uint8_t led4_tick_scale;
static volatile uint32_t tick_whiteled_pressed;
static volatile uint32_t tick_led4_pressed;
static volatile uint8_t whiteled_button_waiting_for_release;
static volatile uint8_t led4_button_waiting_for_release;
static volatile uint8_t whiteled_pressed;
static volatile uint8_t led4_pressed;

void WHITELED_ButtonReleased(void)
{
  whiteled_pressed = 0;
  if (BSP_GetTicks() - tick_whiteled_pressed < TIME_BUTTON_HELD_TO_SWITCH_MS && whiteled_blinking)
  {
    whiteled_tick_scale = (whiteled_tick_scale + 1) & 0x3U;
  }
  whiteled_button_waiting_for_release = 0;
}

// Record the time that the button has been pressed for
void WHITELED_ButtonPressed(void)
{
  whiteled_pressed = 1;
  tick_whiteled_pressed = BSP_GetTicks();
}

void LED4_ButtonReleased(void)
{
  led4_pressed = 0;
  if (BSP_GetTicks() - tick_led4_pressed < TIME_BUTTON_HELD_TO_SWITCH_MS && led4_blinking)
  {
    led4_tick_scale = (led4_tick_scale + 1) & 0x3U;
  }
  led4_button_waiting_for_release = 0;
}

// Record the time that the button has been pressed for
void LED4_ButtonPressed(void)
{
  led4_pressed = 1;
  tick_led4_pressed = BSP_GetTicks();
}

uint32_t stack_led4[40];
uint32_t *sp_led4 = &stack_led4[40];
OSTask led4task;
void main_LED4()
{
  static uint32_t lastblinktick = 0;
  uint32_t cur_ticks = 0;
  uint32_t delay = 0;
  
  while (1)
  {
    __disable_irq();
    cur_ticks = BSP_GetTicks();
    delay = ((led4_tick_scale + 1) * LED_DELAY_INCREMENT_MS) / 2;
    if (led4_blinking && cur_ticks - lastblinktick >= delay)
    {
      lastblinktick = cur_ticks;
      BSP_LED4_toggle();
    }
    
    //NVIC_DisableIRQ(EXTI4_15_IRQn);
    //__disable_irq();
    if (led4_pressed && !led4_button_waiting_for_release && cur_ticks - tick_led4_pressed > TIME_BUTTON_HELD_TO_SWITCH_MS)
    {
      led4_button_waiting_for_release = 1;
      led4_blinking ^= 0x1;
      if (!led4_blinking)
        BSP_LED4_reset();
    }
    __enable_irq();
    //NVIC_EnableIRQ(EXTI4_15_IRQn);
  }
}

uint32_t stack_whiteled[40];
uint32_t *sp_whiteled = &stack_whiteled[40];
OSTask whiteledtask;
void main_WHITELED()
{
  static uint32_t lastblinktick = 0;
  uint32_t cur_ticks = 0;
  uint32_t delay = 0;
  
  while (1)
  {
    __disable_irq();
    cur_ticks = BSP_GetTicks();
    delay = ((whiteled_tick_scale + 1) * LED_DELAY_INCREMENT_MS) / 2;
    if (whiteled_blinking && cur_ticks - lastblinktick >= delay)
    {
      lastblinktick = cur_ticks;
      BSP_WHITELED_toggle();
    }
    
    //NVIC_DisableIRQ(EXTI4_15_IRQn);
    if (whiteled_pressed && !whiteled_button_waiting_for_release && cur_ticks - tick_whiteled_pressed > TIME_BUTTON_HELD_TO_SWITCH_MS)
    {
      whiteled_button_waiting_for_release = 1;
      whiteled_blinking ^= 0x1;
      if (!whiteled_blinking)
        BSP_WHITELED_reset();
    }
    //NVIC_EnableIRQ(EXTI4_15_IRQn);
    __enable_irq();
  }
}

int main(void)
{
  __disable_irq();
  volatile BTN_STATE state = RELEASED;
  led4_tick_scale = 0;
  led4_blinking = 1;
  tick_led4_pressed = 0;
  whiteled_tick_scale = 0;
  whiteled_blinking = 1;
  tick_whiteled_pressed = 0;
  led4_button_waiting_for_release = 0;
  whiteled_button_waiting_for_release = 0;
  
  BSP_init(&state, BREADBOARD_ATTACHED);
  OSInit();
  // Exception return happens with POP r7, pc
  
  /*
  *(--sp_led4) = (0x1U << 24); // Thumb state in xPSR
  *(--sp_led4) = (uint32_t)main_LED4; // PC (start of main_LED4)
  *(--sp_led4) = 0x0000000EU; // LR
  *(--sp_led4) = 0x0000000CU; // R12
  *(--sp_led4) = 0x00000003U; // R3
  *(--sp_led4) = 0x00000002U; // R2
  *(--sp_led4) = 0x00000001U; // R1
  *(--sp_led4) = 0x00000000U; // R0
  *(--sp_led4) = 0xFFFFFFF9U; // PC (Special exception return instruction) see ref manual p. 20
  *(--sp_led4) = 0x00000000U; // R7
  
  *(--sp_whiteled) = (0x1U << 24); // Thumb state in xPSR
  *(--sp_whiteled) = (uint32_t)main_WHITELED; // PC (start of main_LED4)
  *(--sp_whiteled) = 0x0000000EU; // LR
  *(--sp_whiteled) = 0x0000000CU; // R12
  *(--sp_whiteled) = 0x00000003U; // R3
  *(--sp_whiteled) = 0x00000002U; // R2
  *(--sp_whiteled) = 0x00000001U; // R1
  *(--sp_whiteled) = 0x00000000U; // R0
  *(--sp_whiteled) = 0xFFFFFFF9U; // PC (Special exception return instruction) see ref manual p. 20
  *(--sp_whiteled) = 0x00000000U; // R7
  */
  
  OSTask_start(&led4task, main_LED4, stack_led4, sizeof(stack_led4));
  OSTask_start(&whiteledtask, main_WHITELED, stack_whiteled, sizeof(stack_whiteled));
  
  OSRun();
  while (1){};
  
  return 0;
}


void EXTI4_15_IRQHandler(void)
{
  //NVIC_DisableIRQ(EXTI4_15_IRQn);
  __disable_irq();
  static uint32_t last_action_tick = 0;
  
  
  // PC13 Rising (button released)
  if (EXTI->RPR1 & EXTI_RPR1_RPIF13)
  {
    LED4_ButtonReleased();
    EXTI->RPR1 = EXTI_RPR1_RPIF13;
  }
  // PC13 Falling (button pressed)
  else if (EXTI->FPR1 & EXTI_FPR1_FPIF13)
  {
    LED4_ButtonPressed();
    EXTI->FPR1 = EXTI_FPR1_FPIF13; 
  }
  else if (EXTI->FPR1 & EXTI_FPR1_FPIF15)
  {
    if (BSP_GetTicks() - last_action_tick > 4)
    {
      //BSP_BUTTON_released();
      WHITELED_ButtonReleased();
      last_action_tick = BSP_GetTicks();
    }
    EXTI->FPR1 = EXTI_FPR1_FPIF15;
  }
  else if (EXTI->RPR1 & EXTI_RPR1_RPIF15)
  {
    if (BSP_GetTicks() - last_action_tick > 4)
    {
      //BSP_BUTTON_pressed();
      WHITELED_ButtonPressed();
      last_action_tick = BSP_GetTicks();
    }
    EXTI->RPR1 = EXTI_RPR1_RPIF15;
  }
  else
    while (1);
  
  //NVIC_EnableIRQ(EXTI4_15_IRQn);
  __enable_irq();
}