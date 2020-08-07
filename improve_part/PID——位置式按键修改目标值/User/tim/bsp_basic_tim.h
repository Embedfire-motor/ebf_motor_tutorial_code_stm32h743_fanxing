#ifndef __BASIC_TIM_H
#define	__BASIC_TIM_H

#include "stm32h7xx.h"

#define BASIC_TIM           		  TIM6
#define BASIC_TIM_CLK_ENABLE()   	__TIM6_CLK_ENABLE()

#define BASIC_TIM_IRQn				    TIM6_DAC_IRQn
#define BASIC_TIM_IRQHandler    	TIM6_DAC_IRQHandler

/* 累计 TIM_Period个后产生一个更新或者中断*/		
	//当定时器从0计数到BASIC_PERIOD_COUNT-1，即为BASIC_PERIOD_COUNT次，为一个定时周期
#define BASIC_PERIOD_COUNT    (50*50)

	// 通用控制定时器时钟源TIMxCLK = HCLK = 240MHz 
	// 设定定时器频率为=TIMxCLK/TIM_Prescaler=5khz
#define BASIC_PRESCALER_COUNT   (48000)


/* 以下两宏仅适用于定时器时钟源TIMxCLK=240MHz，预分频器为：48000-1 的情况 */
#define SET_BASIC_TIM_PERIOD(T)     __HAL_TIM_SET_AUTORELOAD(&TIM_TimeBaseStructure, (T)*50 - 1)    // 设置定时器的周期（1~1000ms）
#define GET_BASIC_TIM_PERIOD()      ((__HAL_TIM_GET_AUTORELOAD(&TIM_TimeBaseStructure)+1)/50.0)     // 获取定时器的周期，单位ms

extern TIM_HandleTypeDef TIM_TimeBaseStructure;
void TIMx_Configuration(void);

#endif /* __BASIC_TIM_H */
