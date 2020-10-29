#ifndef __BSP_MOTOR_TIM_H
#define	__BSP_MOTOR_TIM_H

#include "stm32h7xx.h"
#include ".\bldcm_control\bsp_bldcm_control.h"

/*****************************����ӿ�1�궨��*******************************************/
/* ������ƶ�ʱ�� */
#define MOTOR1_TIM           				      TIM1
#define MOTOR1_TIM_CLK_ENABLE()  			    __TIM1_CLK_ENABLE()
extern TIM_HandleTypeDef  motor1_htimx_bldcm;

/* �ۼ� TIM_Period�������һ�����»����ж�		
	����ʱ����0������5599����Ϊ5600�Σ�Ϊһ����ʱ���� */
#define MOTOR1_PWM_PERIOD_COUNT     (5600)

#define MOTOR1_PWM_MAX_PERIOD_COUNT    (MOTOR1_PWM_PERIOD_COUNT - 100)

/* �߼����ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK = 240MHz 
	 �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(PWM_PRESCALER_COUNT+1)/PWM_PERIOD_COUNT = 21.48KHz*/
#define MOTOR1_PWM_PRESCALER_COUNT     (2)

/* TIM1ͨ��1������� */
#define MOTOR1_OCPWM1_PIN           		    GPIO_PIN_8
#define MOTOR1_OCPWM1_GPIO_PORT     		    GPIOA
#define MOTOR1_OCPWM1_GPIO_CLK_ENABLE() 	  __GPIOA_CLK_ENABLE()
#define MOTOR1_OCPWM1_AF					          GPIO_AF1_TIM1

/* TIM1ͨ��2������� */
#define MOTOR1_OCPWM2_PIN           		    GPIO_PIN_9
#define MOTOR1_OCPWM2_GPIO_PORT     		    GPIOA
#define MOTOR1_OCPWM2_GPIO_CLK_ENABLE() 	  __GPIOA_CLK_ENABLE()
#define MOTOR1_OCPWM2_AF					          GPIO_AF1_TIM1

/* TIM1ͨ��3������� */
#define MOTOR1_OCPWM3_PIN           		    GPIO_PIN_10
#define MOTOR1_OCPWM3_GPIO_PORT     		    GPIOA
#define MOTOR1_OCPWM3_GPIO_CLK_ENABLE() 	  __GPIOA_CLK_ENABLE()
#define MOTOR1_OCPWM3_AF					          GPIO_AF1_TIM1

/* TIM1ͨ��1����������� */
#define MOTOR1_OCNPWM1_PIN            		  GPIO_PIN_13
#define MOTOR1_OCNPWM1_GPIO_PORT      		  GPIOB
#define MOTOR1_OCNPWM1_GPIO_CLK_ENABLE()	  __GPIOB_CLK_ENABLE()
#define MOTOR1_OCNPWM1_AF					        	GPIO_AF1_TIM1

/* TIM1ͨ��2����������� */
#define MOTOR1_OCNPWM2_PIN            		  GPIO_PIN_14
#define MOTOR1_OCNPWM2_GPIO_PORT      		  GPIOB
#define MOTOR1_OCNPWM2_GPIO_CLK_ENABLE()	  __GPIOB_CLK_ENABLE()
#define MOTOR1_OCNPWM2_AF					       	  GPIO_AF1_TIM1

/* TIM1ͨ��3����������� */
#define MOTOR1_OCNPWM3_PIN            		  GPIO_PIN_15
#define MOTOR1_OCNPWM3_GPIO_PORT      		  GPIOB
#define MOTOR1_OCNPWM3_GPIO_CLK_ENABLE()	  __GPIOB_CLK_ENABLE()
#define MOTOR1_OCNPWM3_AF					        	GPIO_AF1_TIM1

#define MOTOR1_TIM_COM_TS_ITRx              TIM_TS_ITR3    // �ڲ���������(TIM1->ITR2->TIM3)

/* ������������ʱ�� */
#define MOTOR1_HALL_TIM           				  TIM3
#define MOTOR1_HALL_TIM_CLK_ENABLE()  			__TIM3_CLK_ENABLE()

extern TIM_HandleTypeDef motor1_htimx_hall;

/* �ۼ� TIM_Period�������һ�����»����ж�		
	����ʱ����0������65536����Ϊ65536�Σ�Ϊһ����ʱ���� */
#define MOTOR1_HALL_PERIOD_COUNT     (0xFFFF)

/* �߼����ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK / 2 = 240MHz
	 �趨��ʱ��Ƶ��Ϊ = TIMxCLK / (PWM_PRESCALER_COUNT) / PWM_PERIOD_COUNT = 28.6Hz
   ���� T = 34.96ms */
#define MOTOR1_HALL_PRESCALER_COUNT     (128)

/* TIM3 ͨ�� 1 ���� */
#define MOTOR1_HALL_INPUTU_PIN           		    GPIO_PIN_6
#define MOTOR1_HALL_INPUTU_GPIO_PORT     		    GPIOC
#define MOTOR1_HALL_INPUTU_GPIO_CLK_ENABLE() 	  __GPIOC_CLK_ENABLE()
#define MOTOR1_HALL_INPUTU_AF					          GPIO_AF2_TIM3

/* TIM3 ͨ�� 2 ���� */
#define MOTOR1_HALL_INPUTV_PIN           		    GPIO_PIN_7
#define MOTOR1_HALL_INPUTV_GPIO_PORT     		    GPIOC
#define MOTOR1_HALL_INPUTV_GPIO_CLK_ENABLE() 	  __GPIOC_CLK_ENABLE()
#define MOTOR1_HALL_INPUTV_AF					          GPIO_AF2_TIM3

/* TIM3 ͨ�� 3 ���� */
#define MOTOR1_HALL_INPUTW_PIN           		    GPIO_PIN_8
#define MOTOR1_HALL_INPUTW_GPIO_PORT     		    GPIOC
#define MOTOR1_HALL_INPUTW_GPIO_CLK_ENABLE() 	  __GPIOC_CLK_ENABLE()
#define MOTOR1_HALL_INPUTW_AF					          GPIO_AF2_TIM3

#define MOTOR1_HALL_TIM_IRQn                    TIM3_IRQn
#define MOTOR1_HALL_TIM_IRQHandler              TIM3_IRQHandler


/***************************************************************************************/

/*****************************����ӿ�2�궨��*******************************************/
/* ������ƶ�ʱ�� */
#define MOTOR2_TIM           				      TIM8
#define MOTOR2_TIM_CLK_ENABLE()  			    __TIM8_CLK_ENABLE()
extern TIM_HandleTypeDef  motor2_htimx_bldcm;

/* �ۼ� TIM_Period�������һ�����»����ж�		
	����ʱ����0������5599����Ϊ5600�Σ�Ϊһ����ʱ���� */
#define MOTOR2_PWM_PERIOD_COUNT     (5600)

#define MOTOR2_PWM_MAX_PERIOD_COUNT    (MOTOR2_PWM_PERIOD_COUNT - 100)

/* �߼����ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK = 240MHz 
	 �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(PWM_PRESCALER_COUNT+1)/PWM_PERIOD_COUNT = 21.48KHz*/
#define MOTOR2_PWM_PRESCALER_COUNT     (2)

/* TIM8ͨ��1������� */
#define MOTOR2_OCPWM1_PIN           		    GPIO_PIN_5
#define MOTOR2_OCPWM1_GPIO_PORT     		    GPIOI
#define MOTOR2_OCPWM1_GPIO_CLK_ENABLE() 	  __GPIOI_CLK_ENABLE()
#define MOTOR2_OCPWM1_AF					          GPIO_AF3_TIM8

/* TIM8ͨ��2������� */
#define MOTOR2_OCPWM2_PIN           		    GPIO_PIN_6
#define MOTOR2_OCPWM2_GPIO_PORT     		    GPIOI
#define MOTOR2_OCPWM2_GPIO_CLK_ENABLE() 	  __GPIOI_CLK_ENABLE()
#define MOTOR2_OCPWM2_AF					          GPIO_AF3_TIM8

/* TIM8ͨ��3������� */
#define MOTOR2_OCPWM3_PIN           		    GPIO_PIN_7
#define MOTOR2_OCPWM3_GPIO_PORT     		    GPIOI
#define MOTOR2_OCPWM3_GPIO_CLK_ENABLE() 	  __GPIOI_CLK_ENABLE()
#define MOTOR2_OCPWM3_AF					          GPIO_AF3_TIM8

/* TIM8ͨ��1����������� */
#define MOTOR2_OCNPWM1_PIN            		  GPIO_PIN_13
#define MOTOR2_OCNPWM1_GPIO_PORT      		  GPIOH
#define MOTOR2_OCNPWM1_GPIO_CLK_ENABLE()	  __GPIOH_CLK_ENABLE()
#define MOTOR2_OCNPWM1_AF					        	GPIO_AF3_TIM8

/* TIM8ͨ��2����������� */
#define MOTOR2_OCNPWM2_PIN            		  GPIO_PIN_14
#define MOTOR2_OCNPWM2_GPIO_PORT      		  GPIOH
#define MOTOR2_OCNPWM2_GPIO_CLK_ENABLE()	  __GPIOH_CLK_ENABLE()
#define MOTOR2_OCNPWM2_AF					        	GPIO_AF3_TIM8

/* TIM8ͨ��3����������� */
#define MOTOR2_OCNPWM3_PIN            		  GPIO_PIN_15
#define MOTOR2_OCNPWM3_GPIO_PORT      		  GPIOH
#define MOTOR2_OCNPWM3_GPIO_CLK_ENABLE()	  __GPIOH_CLK_ENABLE()
#define MOTOR2_OCNPWM3_AF					        	GPIO_AF3_TIM8

#define MOTOR2_TIM_COM_TS_ITRx              TIM_TS_ITR3    // �ڲ���������(TIM8->ITR3->TIM5)

/* ������������ʱ�� */
#define MOTOR2_HALL_TIM           				  TIM5
#define MOTOR2_HALL_TIM_CLK_ENABLE()  			__TIM5_CLK_ENABLE()

extern TIM_HandleTypeDef motor2_htimx_hall;

/* �ۼ� TIM_Period�������һ�����»����ж�		
	����ʱ����0������9999����Ϊ10000�Σ�Ϊһ����ʱ���� */
#define MOTOR2_HALL_PERIOD_COUNT     (10000)

/* �߼����ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK / 2 = 240MHz
	 �趨��ʱ��Ƶ��Ϊ = TIMxCLK / (PWM_PRESCALER_COUNT) / PWM_PERIOD_COUNT = 28.6Hz
   ���� T = 34.96ms */
#define MOTOR2_HALL_PRESCALER_COUNT     (2400)

/* TIM5 ͨ�� 1 ���� */
#define MOTOR2_HALL_INPUTU_PIN           		    GPIO_PIN_10
#define MOTOR2_HALL_INPUTU_GPIO_PORT     		    GPIOH
#define MOTOR2_HALL_INPUTU_GPIO_CLK_ENABLE() 	  __GPIOH_CLK_ENABLE()
#define MOTOR2_HALL_INPUTU_AF					          GPIO_AF2_TIM5

/* TIM5 ͨ�� 2 ���� */
#define MOTOR2_HALL_INPUTV_PIN           		    GPIO_PIN_11
#define MOTOR2_HALL_INPUTV_GPIO_PORT     		    GPIOH
#define MOTOR2_HALL_INPUTV_GPIO_CLK_ENABLE() 	  __GPIOH_CLK_ENABLE()
#define MOTOR2_HALL_INPUTV_AF					          GPIO_AF2_TIM5

/* TIM5 ͨ�� 3 ���� */
#define MOTOR2_HALL_INPUTW_PIN           		    GPIO_PIN_12
#define MOTOR2_HALL_INPUTW_GPIO_PORT     		    GPIOH
#define MOTOR2_HALL_INPUTW_GPIO_CLK_ENABLE() 	  __GPIOH_CLK_ENABLE()
#define MOTOR2_HALL_INPUTW_AF					          GPIO_AF2_TIM5

#define MOTOR2_HALL_TIM_IRQn                    TIM5_IRQn
#define MOTOR2_HALL_TIM_IRQHandler              TIM5_IRQHandler


/***************************************************************************************/

extern TIM_HandleTypeDef motor1_TIM_TimeBaseStructure;
extern TIM_HandleTypeDef motor2_TIM_TimeBaseStructure;

void TIMx_Configuration(void);

void stop_motor1_pwm_output(void);
void set_motor1_pwm_pulse(uint16_t pulse);

void hall_motor1_enable(void);
void hall_motor1_disable(void);
void hall_motor1_tim_config(void);

void stop_motor2_pwm_output(void);
void set_motor2_pwm_pulse(uint16_t pulse);

void hall_motor2_enable(void);
void hall_motor2_disable(void);
void hall_motor2_tim_config(void);

#endif /* __BSP_MOTOR_TIM_H */

