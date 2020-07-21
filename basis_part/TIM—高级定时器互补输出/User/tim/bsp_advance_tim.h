#ifndef __BASIC_TIM_H
#define	__BASIC_TIM_H

#include "stm32h7xx.h"

/* ��ʱ�� */
#define ADVANCE_TIM           				    TIM8
#define ADVANCE_TIM_CLK_ENABLE()  		  	__TIM8_CLK_ENABLE()

/* TIM8ͨ��1������� */
#define ADVANCE_OCPWM_PIN           		  GPIO_PIN_6              
#define ADVANCE_OCPWM_GPIO_PORT     		  GPIOC                      
#define ADVANCE_OCPWM_GPIO_CLK_ENABLE() 	__HAL_RCC_GPIOC_CLK_ENABLE()
#define ADVANCE_OCPWM_AF					        GPIO_AF3_TIM8

/* TIM8ͨ��1����������� */
#define ADVANCE_OCNPWM_PIN            		GPIO_PIN_5              
#define ADVANCE_OCNPWM_GPIO_PORT      		GPIOA                      
#define ADVANCE_OCNPWM_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOA_CLK_ENABLE()
#define ADVANCE_OCNPWM_AF					        GPIO_AF3_TIM8

/* TIM8��·�������� */
#define ADVANCE_BKIN_PIN              		GPIO_PIN_6              
#define ADVANCE_BKIN_GPIO_PORT        		GPIOA                      
#define ADVANCE_BKIN_GPIO_CLK_ENABLE()  	__HAL_RCC_GPIOA_CLK_ENABLE()
#define ADVANCE_BKIN_AF						        GPIO_AF3_TIM8


extern TIM_HandleTypeDef TIM_TimeBaseStructure;

void TIMx_Configuration(void);
#endif /* __ADVANCE_TIM_H */
