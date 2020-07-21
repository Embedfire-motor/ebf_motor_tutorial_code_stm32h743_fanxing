#ifndef __EXTI_H
#define	__EXTI_H

#include "stm32h7xx.h"

//���Ŷ���
/*******************************************************/
#define KEY1_INT_GPIO_PORT                GPIOA
#define KEY1_INT_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE();
#define KEY1_INT_GPIO_PIN                 GPIO_PIN_0
#define KEY1_INT_EXTI_IRQ                 EXTI0_IRQn
#define KEY1_IRQHandler                   EXTI0_IRQHandler

#define KEY3_INT_GPIO_PORT                GPIOC
#define KEY3_INT_GPIO_CLK_ENABLE()        __GPIOC_CLK_ENABLE();
#define KEY3_INT_GPIO_PIN                 GPIO_PIN_13
#define KEY3_INT_EXTI_IRQ                 EXTI15_10_IRQn
#define KEY3_IRQHandler                   EXTI15_10_IRQHandler
/*******************************************************/


void EXTI_Key_Config(void);

#endif /* __EXTI_H */
