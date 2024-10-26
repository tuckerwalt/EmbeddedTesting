#ifndef WT_BSP_H_
#define WT_BSP_H_

//#include "stm32c0xx_hal.h"
#include "stdint.h"

typedef enum 
{
  RELEASED,
  PRESSED
} BTN_STATE;

void BSP_init(volatile BTN_STATE *state, uint8_t breadboard_attached);

void BSP_LED4_toggle(void);
void BSP_LED4_set(void);
void BSP_LED4_reset(void);

void BSP_WHITELED_toggle(void);
void BSP_WHITELED_set(void);
void BSP_WHITELED_reset(void);

void BSP_delay(volatile uint32_t iter);

void BSP_IncTick(void);
uint32_t BSP_GetTicks(void);

void BSP_BUTTON_pressed(void);
void BSP_BUTTON_released(void);
#endif // WT_BSP_H_