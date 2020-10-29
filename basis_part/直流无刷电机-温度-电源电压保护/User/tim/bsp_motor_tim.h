#ifndef __BSP_MOTOR_TIM_H
#define	__BSP_MOTOR_TIM_H

#include "stm32h7xx.h"
#include ".\bldcm_control\bsp_bldcm_control.h"

/* ������ƶ�ʱ�� */
#define MOTOR_TIM           				      TIM8
#define MOTOR_TIM_CLK_ENABLE()  			    __TIM8_CLK_ENABLE()
extern TIM_HandleTypeDef  htimx_bldcm;

/* �ۼ� TIM_Period�������һ�����»����ж�		
	����ʱ����0������5599����Ϊ5600�Σ�Ϊһ����ʱ���� */
#define PWM_PERIOD_COUNT     (5600)

#define PWM_MAX_PERIOD_COUNT    (PWM_PERIOD_COUNT - 100)

/* ��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
					PCLK1 = HCLK / 2
					=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	 �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(PWM_PRESCALER_COUNT+1)/PWM_PERIOD_COUNT = 14.28KHz*/
#define PWM_PRESCALER_COUNT     (3)

/* TIM8ͨ��1������� */
#define MOTOR_OCPWM1_PIN           		    GPIO_PIN_5
#define MOTOR_OCPWM1_GPIO_PORT     		    GPIOI
#define MOTOR_OCPWM1_GPIO_CLK_ENABLE() 	  __GPIOI_CLK_ENABLE()
#define MOTOR_OCPWM1_AF					          GPIO_AF3_TIM8

/* TIM8ͨ��2������� */
#define MOTOR_OCPWM2_PIN           		    GPIO_PIN_6
#define MOTOR_OCPWM2_GPIO_PORT     		    GPIOI
#define MOTOR_OCPWM2_GPIO_CLK_ENABLE() 	  __GPIOI_CLK_ENABLE()
#define MOTOR_OCPWM2_AF					          GPIO_AF3_TIM8

/* TIM8ͨ��3������� */
#define MOTOR_OCPWM3_PIN           		    GPIO_PIN_7
#define MOTOR_OCPWM3_GPIO_PORT     		    GPIOI
#define MOTOR_OCPWM3_GPIO_CLK_ENABLE() 	  __GPIOI_CLK_ENABLE()
#define MOTOR_OCPWM3_AF					          GPIO_AF3_TIM8

/* TIM8ͨ��1����������� */
#define MOTOR_OCNPWM1_PIN            		  GPIO_PIN_13
#define MOTOR_OCNPWM1_GPIO_PORT      		  GPIOH
#define MOTOR_OCNPWM1_GPIO_CLK_ENABLE()	  __GPIOH_CLK_ENABLE()
#define MOTOR_OCNPWM1_AF					        GPIO_AF3_TIM8

/* TIM8ͨ��2����������� */
#define MOTOR_OCNPWM2_PIN            		  GPIO_PIN_14
#define MOTOR_OCNPWM2_GPIO_PORT      		  GPIOH
#define MOTOR_OCNPWM2_GPIO_CLK_ENABLE()	  __GPIOH_CLK_ENABLE()
#define MOTOR_OCNPWM2_AF					        GPIO_AF3_TIM8

/* TIM8ͨ��3����������� */
#define MOTOR_OCNPWM3_PIN            		  GPIO_PIN_15
#define MOTOR_OCNPWM3_GPIO_PORT      		  GPIOH
#define MOTOR_OCNPWM3_GPIO_CLK_ENABLE()	  __GPIOH_CLK_ENABLE()
#define MOTOR_OCNPWM3_AF					        GPIO_AF3_TIM8

#define TIM_COM_TS_ITRx                   TIM_TS_ITR3    // �ڲ���������(TIM8->ITR3->TIM5)

/* ������������ʱ�� */
#define HALL_TIM           				      TIM5
#define HALL_TIM_CLK_ENABLE()  			    __TIM5_CLK_ENABLE()

extern TIM_HandleTypeDef htimx_hall;

/* �ۼ� TIM_Period�������һ�����»����ж�		
	����ʱ����0������9999����Ϊ10000�Σ�Ϊһ����ʱ���� */
#define HALL_PERIOD_COUNT     (10000)

/* ��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
					PCLK1 = HCLK / 4 
					=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	 �趨��ʱ��Ƶ��Ϊ = TIMxCLK / (PWM_PRESCALER_COUNT + 1) / PWM_PERIOD_COUNT = 10Hz
   ���� T = 100ms */
#define HALL_PRESCALER_COUNT     (2400)

/* TIM5 ͨ�� 1 ���� */
#define HALL_INPUTU_PIN           		    GPIO_PIN_10
#define HALL_INPUTU_GPIO_PORT     		    GPIOH
#define HALL_INPUTU_GPIO_CLK_ENABLE() 	  __GPIOH_CLK_ENABLE()
#define HALL_INPUTU_AF					          GPIO_AF2_TIM5

/* TIM5 ͨ�� 2 ���� */
#define HALL_INPUTV_PIN           		    GPIO_PIN_11
#define HALL_INPUTV_GPIO_PORT     		    GPIOH
#define HALL_INPUTV_GPIO_CLK_ENABLE() 	  __GPIOH_CLK_ENABLE()
#define HALL_INPUTV_AF					          GPIO_AF2_TIM5

/* TIM5 ͨ�� 3 ���� */
#define HALL_INPUTW_PIN           		    GPIO_PIN_12
#define HALL_INPUTW_GPIO_PORT     		    GPIOH
#define HALL_INPUTW_GPIO_CLK_ENABLE() 	  __GPIOH_CLK_ENABLE()
#define HALL_INPUTW_AF					          GPIO_AF2_TIM5

#define HALL_TIM_IRQn                    TIM5_IRQn
#define HALL_TIM_IRQHandler              TIM5_IRQHandler

extern TIM_HandleTypeDef TIM_TimeBaseStructure;

void TIMx_Configuration(void);
void stop_pwm_output(void);
void start_pwm_output(void);
void set_pwm_pulse(uint16_t pulse);

void hall_enable(void);
void hall_disable(void);
void hall_tim_config(void);

#endif /* __BSP_MOTOR_TIM_H */

