#include "main.h"
#include "bsp.h"
#include "tasks.h"

#define TIME_BUTTON_HELD_TO_SWITCH_MS 2000
#define LED_DELAY_INCREMENT_MS 250
#define BREADBOARD_ATTACHED 1

static uint8_t bBlinking;
static uint8_t tick_scale;
static uint32_t tick_pressed;
static uint8_t bButtonHeldWaitingToRelease;

void ButtonReleased(void)
{
  if (BSP_GetTicks() - tick_pressed < TIME_BUTTON_HELD_TO_SWITCH_MS && bBlinking)
  {
    tick_scale = (tick_scale + 1) & 0x3U;
  }
  bButtonHeldWaitingToRelease = 0;
}

// Record the time that the button has been pressed for
void ButtonPressed(void)
{
  tick_pressed = BSP_GetTicks();
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
    cur_ticks = BSP_GetTicks();
    delay = ((tick_scale + 1) * LED_DELAY_INCREMENT_MS) / 2;
    if (cur_ticks - lastblinktick >= delay)
    {
      lastblinktick = cur_ticks;
      BSP_LED4_toggle();
    }
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
    cur_ticks = BSP_GetTicks();
    delay = ((tick_scale + 1) * LED_DELAY_INCREMENT_MS) / 2;
    if (cur_ticks - lastblinktick >= delay)
    {
      lastblinktick = cur_ticks;
      BSP_WHITELED_toggle();
    }
  }
}

int main(void)
{
  volatile BTN_STATE state = RELEASED;
  __disable_irq();
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
  static uint32_t last_action_tick = 0;
  
  NVIC_DisableIRQ(EXTI4_15_IRQn);
  // PC13 Rising (button released)
  if (EXTI->RPR1 & EXTI_RPR1_RPIF13)
  {
    BSP_BUTTON_released();
    ButtonReleased();
    EXTI->RPR1 = EXTI_RPR1_RPIF13;
  }
  // PC13 Falling (button pressed)
  else if (EXTI->FPR1 & EXTI_FPR1_FPIF13)
  {
    BSP_BUTTON_pressed();
    ButtonPressed();
    EXTI->FPR1 = EXTI_FPR1_FPIF13; 
  }
  else if (EXTI->FPR1 & EXTI_FPR1_FPIF15)
  {
    if (BSP_GetTicks() - last_action_tick > 4)
    {
      BSP_BUTTON_released();
      ButtonReleased();
      last_action_tick = BSP_GetTicks();
    }
    EXTI->FPR1 = EXTI_FPR1_FPIF15;
  }
  else if (EXTI->RPR1 & EXTI_RPR1_RPIF15)
  {
    if (BSP_GetTicks() - last_action_tick > 4)
    {
      BSP_BUTTON_pressed();
      ButtonPressed();
      last_action_tick = BSP_GetTicks();
    }
    EXTI->RPR1 = EXTI_RPR1_RPIF15;
  }
  else
    while (1);
  
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}