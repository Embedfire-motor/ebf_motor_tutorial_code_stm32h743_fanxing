#ifndef __BASIC_TIM_H
#define	__BASIC_TIM_H

#include "stm32h7xx.h"

/*�궨��*/
#define GENERAL_TIM                        	TIM4
#define GENERAL_TIM_GPIO_AF                 GPIO_AF2_TIM4
#define GENERAL_TIM_CLK_ENABLE()  					__TIM4_CLK_ENABLE()

#define PWM_CHANNEL_1                       TIM_CHANNEL_1
//#define PWM_CHANNEL_2                       TIM_CHANNEL_2
//#define PWM_CHANNEL_3                       TIM_CHANNEL_3
//#define PWM_CHANNEL_4                       TIM_CHANNEL_4

/* �ۼ� TIM_Period�������һ�����»����ж�*/		
/* ����ʱ����0������PWM_PERIOD_COUNT����ΪPWM_PERIOD_COUNT+1�Σ�Ϊһ����ʱ���� */
#define PWM_PERIOD_COUNT     999

//��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
//				PCLK1 = HCLK / 4 
//				=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
/* �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(PWM_PRESCALER_COUNT+1) */
#define PWM_PRESCALER_COUNT     4800

/*PWM����*/
#define GENERAL_TIM_CH1_GPIO_PORT           GPIOD
#define GENERAL_TIM_CH1_PIN                 GPIO_PIN_12

//#define GENERAL_TIM_CH2_GPIO_PORT           GPIOD
//#define GENERAL_TIM_CH2_PIN                 GPIO_PIN_13

extern TIM_HandleTypeDef  TIM_TimeBaseStructure;

extern void TIMx_Configuration(void);
extern void TIM2_SetPWM_pulse(uint32_t channel,int compare);
void set_steering_gear_dutyfactor(uint16_t dutyfactor);
void set_steering_gear_angle(uint16_t angle);
void show_help(void);
void deal_serial_data(void);

#endif
