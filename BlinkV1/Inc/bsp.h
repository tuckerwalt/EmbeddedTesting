#ifndef WT_BSP_H_
#define WT_BSP_H_

//#include "stm32c0xx_hal.h"
#include "stdint.h"

void BSP_init(volatile uint16_t *state);

void BSP_LED4_toggle(void);

void BSP_delay(volatile uint32_t iter);

void BSP_IncTick();
#endif // WT_BSP_H_