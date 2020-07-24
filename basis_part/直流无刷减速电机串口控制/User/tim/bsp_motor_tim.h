#ifndef __BSP_MOTOR_TIM_H
#define	__BSP_MOTOR_TIM_H

#include "stm32h7xx.h"
#include ".\bldcm_control\bsp_bldcm_control.h"

/* 电机控制定时器 */
#define MOTOR_TIM           				      TIM8
#define MOTOR_TIM_CLK_ENABLE()  			    __TIM8_CLK_ENABLE()
extern TIM_HandleTypeDef  htimx_bldcm;

/* 累计 TIM_Period个后产生一个更新或者中断		
	当定时器从0计数到5599，即为5600次，为一个定时周期 */
#define PWM_PERIOD_COUNT     (1000)

/* 定时器时钟源TIMxCLK = 2 * PCLK1  
					PCLK1 = HCLK / 4 
					=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT+1)/PWM_PERIOD_COUNT = 15KHz*/
#define PWM_PRESCALER_COUNT     (16)

/* TIM8通道1输出引脚 */
#define MOTOR_OCPWM1_PIN           		    GPIO_PIN_5
#define MOTOR_OCPWM1_GPIO_PORT     		    GPIOI
#define MOTOR_OCPWM1_GPIO_CLK_ENABLE() 	  __GPIOI_CLK_ENABLE()
#define MOTOR_OCPWM1_AF					          GPIO_AF3_TIM8

/* TIM8通道2输出引脚 */
#define MOTOR_OCPWM2_PIN           		    GPIO_PIN_6
#define MOTOR_OCPWM2_GPIO_PORT     		    GPIOI
#define MOTOR_OCPWM2_GPIO_CLK_ENABLE() 	  __GPIOI_CLK_ENABLE()
#define MOTOR_OCPWM2_AF					          GPIO_AF3_TIM8

/* TIM8通道3输出引脚 */
#define MOTOR_OCPWM3_PIN           		    GPIO_PIN_7
#define MOTOR_OCPWM3_GPIO_PORT     		    GPIOI
#define MOTOR_OCPWM3_GPIO_CLK_ENABLE() 	  __GPIOI_CLK_ENABLE()
#define MOTOR_OCPWM3_AF					          GPIO_AF3_TIM8

/* TIM8通道1互补输出引脚 */
#define MOTOR_OCNPWM1_PIN            		  GPIO_PIN_13
#define MOTOR_OCNPWM1_GPIO_PORT      		  GPIOH
#define MOTOR_OCNPWM1_GPIO_CLK_ENABLE()	  __GPIOH_CLK_ENABLE()
#define MOTOR_OCNPWM1_AF					        GPIO_AF3_TIM8

/* TIM8通道2互补输出引脚 */
#define MOTOR_OCNPWM2_PIN            		  GPIO_PIN_14
#define MOTOR_OCNPWM2_GPIO_PORT      		  GPIOH
#define MOTOR_OCNPWM2_GPIO_CLK_ENABLE()	  __GPIOH_CLK_ENABLE()
#define MOTOR_OCNPWM2_AF					        GPIO_AF3_TIM8

/* TIM8通道3互补输出引脚 */
#define MOTOR_OCNPWM3_PIN            		  GPIO_PIN_15
#define MOTOR_OCNPWM3_GPIO_PORT      		  GPIOH
#define MOTOR_OCNPWM3_GPIO_CLK_ENABLE()	  __GPIOH_CLK_ENABLE()
#define MOTOR_OCNPWM3_AF					        GPIO_AF3_TIM8

/* 霍尔传感器定时器 */
#define HALL_TIM           				      TIM5
#define HALL_TIM_CLK_ENABLE()  			    __TIM5_CLK_ENABLE()

extern TIM_HandleTypeDef htimx_hall;

/* 累计 TIM_Period个后产生一个更新或者中断		
	当定时器从0计数到4999，即为5000次，为一个定时周期 */
#define HALL_PERIOD_COUNT     (10000)

/* 定时器时钟源TIMxCLK = 2 * PCLK1  
					PCLK1 = HCLK / 4 
					=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	 设定定时器频率为 = TIMxCLK / (PWM_PRESCALER_COUNT + 1) / PWM_PERIOD_COUNT = 10Hz
   周期 T = 100ms */
#define HALL_PRESCALER_COUNT     (2400)

/* TIM5 通道 1 引脚 */
#define HALL_INPUTU_PIN           		    GPIO_PIN_10
#define HALL_INPUTU_GPIO_PORT     		    GPIOH
#define HALL_INPUTU_GPIO_CLK_ENABLE() 	  __GPIOH_CLK_ENABLE()
#define HALL_INPUTU_AF					          GPIO_AF2_TIM5

/* TIM5 通道 2 引脚 */
#define HALL_INPUTV_PIN           		    GPIO_PIN_11
#define HALL_INPUTV_GPIO_PORT     		    GPIOH
#define HALL_INPUTV_GPIO_CLK_ENABLE() 	  __GPIOH_CLK_ENABLE()
#define HALL_INPUTV_AF					          GPIO_AF2_TIM5

/* TIM5 通道 3 引脚 */
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

