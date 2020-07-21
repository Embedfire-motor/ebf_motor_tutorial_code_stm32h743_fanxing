#ifndef __BASIC_TIM_H
#define	__BASIC_TIM_H

#include "stm32h7xx.h"

/* ͨ�ö�ʱ�� */
#define GENERAL_TIM           		    	  TIM2
#define GENERAL_TIM_CLK_ENABLE()       		__TIM2_CLK_ENABLE()

/* ͨ�ö�ʱ��PWM��� */
/* PWM������� */
#define GENERAL_OCPWM_PIN            		  GPIO_PIN_15              
#define GENERAL_OCPWM_GPIO_PORT      		  GPIOA                      
#define GENERAL_OCPWM_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()
#define GENERAL_OCPWM_AF					        GPIO_AF1_TIM2

/* �߼����ƶ�ʱ�� */
#define ADVANCE_TIM           		    	  TIM8
#define ADVANCE_TIM_CLK_ENABLE()      		__TIM8_CLK_ENABLE()

/* ����/�Ƚ��ж� */
#define ADVANCE_TIM_IRQn					        TIM8_CC_IRQn
#define ADVANCE_TIM_IRQHandler        		TIM8_CC_IRQHandler
/* �߼����ƶ�ʱ��PWM���벶�� */
/* PWM���벶������ */
#define ADVANCE_ICPWM_PIN              		GPIO_PIN_6              
#define ADVANCE_ICPWM_GPIO_PORT        		GPIOC                      
#define ADVANCE_ICPWM_GPIO_CLK_ENABLE()  	__HAL_RCC_GPIOC_CLK_ENABLE()
#define ADVANCE_ICPWM_AF					        GPIO_AF3_TIM8
#define ADVANCE_IC1PWM_CHANNEL        		TIM_CHANNEL_1
#define ADVANCE_IC2PWM_CHANNEL        		TIM_CHANNEL_2

extern TIM_HandleTypeDef  TIM_PWMOUTPUT_Handle;
extern TIM_HandleTypeDef  TIM_PWMINPUT_Handle;

void TIMx_Configuration(void);
#endif /* __ADVANCE_TIM_H */
