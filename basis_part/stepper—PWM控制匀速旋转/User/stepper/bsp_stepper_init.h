#ifndef __BSP_STEP_MOTOR_INIT_H
#define	__BSP_STEP_MOTOR_INIT_H

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

/*�궨��*/
/*******************************************************/
//Motor ���� 
#define MOTOR_DIR_PIN                  	GPIO_PIN_1   
#define MOTOR_DIR_GPIO_PORT            	GPIOE                    
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOE_CLK_ENABLE()

//Motor ʹ�� 
#define MOTOR_EN_PIN                  	GPIO_PIN_0
#define MOTOR_EN_GPIO_PORT            	GPIOE                       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOE_CLK_ENABLE()
	
//Motor ����		

#define MOTOR_PUL_TIM                   TIM8
#define MOTOR_PUL_CLK_ENABLE()  		__TIM8_CLK_ENABLE()

#define GENERAL_TIM_CH1_GPIO_PORT       GPIOI
#define GENERAL_TIM_CH1_PIN             GPIO_PIN_5
#define MOTOR_PUL_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOI_CLK_ENABLE()

#define MOTOR_PUL_GPIO_AF               GPIO_AF3_TIM8
#define MOTOR_PUL_CHANNEL_x             TIM_CHANNEL_1




extern void TIMx_Configuration(void);
extern void TIM2_SetPWM_pulse(int channel,int compare);

	
/************************************************************/
#define HIGH GPIO_PIN_SET       //�ߵ�ƽ
#define LOW  GPIO_PIN_RESET     //�͵�ƽ

#define ON  LOW                 //��
#define OFF HIGH                //��

#define CW  HIGH                //˳ʱ��
#define CCW LOW                 //��ʱ��




//����ʹ������
/* ���κ꣬��������������һ��ʹ�� */
#define MOTOR_EN(x)					HAL_GPIO_WritePin(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN,x)
#define MOTOR_PLU(x)				HAL_GPIO_WritePin(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN,x)
#define MOTOR_DIR(x)				HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT,MOTOR_DIR_PIN,x)

extern void stepper_Init(void);
extern void stepper_turn(int tim,float angle,float subdivide,uint8_t dir);
#endif /* __STEP_MOTOR_INIT_H */
