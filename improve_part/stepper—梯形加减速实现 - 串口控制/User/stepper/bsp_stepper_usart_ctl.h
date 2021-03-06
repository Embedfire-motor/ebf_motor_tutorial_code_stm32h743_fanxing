#ifndef __BSP_STEPPER_USART_CTL_H
#define	__BSP_STEPPER_USART_CTL_H

#include "stm32h7xx.h"
#include <math.h>
#include <stdlib.h>
#include "./stepper/bsp_stepper_T_speed.h"
#include "./usart/bsp_debug_usart.h"
#include "./delay/core_delay.h"

extern void ShowHelp(void);
extern void DealSerialData(void);

#endif
