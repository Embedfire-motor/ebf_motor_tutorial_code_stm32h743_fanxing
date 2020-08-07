#ifndef __BASIC_TIM_H
#define	__BASIC_TIM_H

#include "stm32h7xx.h"

#define BASIC_TIM           		  TIM6
#define BASIC_TIM_CLK_ENABLE()   	__TIM6_CLK_ENABLE()

#define BASIC_TIM_IRQn				    TIM6_DAC_IRQn
#define BASIC_TIM_IRQHandler    	TIM6_DAC_IRQHandler

/* �ۼ� TIM_Period�������һ�����»����ж�*/		
	//����ʱ����0������BASIC_PERIOD_COUNT-1����ΪBASIC_PERIOD_COUNT�Σ�Ϊһ����ʱ����
#define BASIC_PERIOD_COUNT    (50*50)

	// ͨ�ÿ��ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK = 240MHz 
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/TIM_Prescaler=5khz
#define BASIC_PRESCALER_COUNT   (48000)


/* ��������������ڶ�ʱ��ʱ��ԴTIMxCLK=240MHz��Ԥ��Ƶ��Ϊ��48000-1 ����� */
#define SET_BASIC_TIM_PERIOD(T)     __HAL_TIM_SET_AUTORELOAD(&TIM_TimeBaseStructure, (T)*50 - 1)    // ���ö�ʱ�������ڣ�1~1000ms��
#define GET_BASIC_TIM_PERIOD()      ((__HAL_TIM_GET_AUTORELOAD(&TIM_TimeBaseStructure)+1)/50.0)     // ��ȡ��ʱ�������ڣ���λms

extern TIM_HandleTypeDef TIM_TimeBaseStructure;
void TIMx_Configuration(void);

#endif /* __BASIC_TIM_H */
