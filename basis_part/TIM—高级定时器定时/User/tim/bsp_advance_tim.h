#ifndef __BASIC_TIM_H
#define	__BASIC_TIM_H

#include "stm32h7xx.h"

#define ADVANCE_TIM           		  TIM1
#define ADVANCE_TIM_CLK_ENABLE()  	__TIM1_CLK_ENABLE()

#define ADVANCE_TIM_IRQn		        TIM1_UP_IRQn
#define ADVANCE_TIM_IRQHandler      TIM1_UP_IRQHandler

extern TIM_HandleTypeDef TIM_TimeBaseStructure;

void TIMx_Configuration(void);

#endif /* __BASIC_TIM_H */

