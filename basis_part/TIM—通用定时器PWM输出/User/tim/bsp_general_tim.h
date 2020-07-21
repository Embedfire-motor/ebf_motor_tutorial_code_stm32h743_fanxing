#ifndef __BASIC_TIM_H
#define	__BASIC_TIM_H

#include "stm32h7xx.h"

#define GENERAL_TIM                        	TIM2
#define GENERAL_TIM_GPIO_AF                 GPIO_AF1_TIM2
#define GENERAL_TIM_CLK_ENABLE()  					__TIM2_CLK_ENABLE()


/*PWM引脚*/
#define GENERAL_TIM_CH1_GPIO_PORT           GPIOA
#define GENERAL_TIM_CH1_PIN                 GPIO_PIN_15

#define GENERAL_TIM_CH2_GPIO_PORT           GPIOB
#define GENERAL_TIM_CH2_PIN                 GPIO_PIN_3  //SPI_FLASH占用

#define GENERAL_TIM_CH3_GPIO_PORT           GPIOB
#define GENERAL_TIM_CH3_PIN                 GPIO_PIN_10 //RS232占用

#define GENERAL_TIM_CH4_GPIO_PORT           GPIOB
#define GENERAL_TIM_CH4_PIN                 GPIO_PIN_11 //RS232占用

extern TIM_HandleTypeDef  TIM_TimeBaseStructure;

extern void TIMx_Configuration(void);
extern void TIM2_SetPWM_pulse(int channel,int compare);

#endif /* __BASIC_TIM_H */

