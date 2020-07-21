#ifndef __BASIC_TIM_H
#define	__BASIC_TIM_H

#include "stm32h7xx.h"

#define GENERAL_TIM                     TIM2
#define GENERAL_TIM_CLK_ENABLE()  			__TIM2_CLK_ENABLE()

#define GENERAL_TIM_IRQ                  TIM2_IRQn
#define GENERAL_TIM_INT_IRQHandler       TIM2_IRQHandler

extern TIM_HandleTypeDef TIM_Base;

void TIMx_Configuration(void);

#endif /* __BASIC_TIM_H */

