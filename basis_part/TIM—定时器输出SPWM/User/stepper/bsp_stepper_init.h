#ifndef __BSP_STEP_MOTOR_INIT_H
#define	__BSP_STEP_MOTOR_INIT_H

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#define GENERAL_TIM                     TIM2
#define GENERAL_TIM_CLK_ENABLE()  			__TIM2_CLK_ENABLE()

#define GENERAL_TIM_IRQ                  TIM2_IRQn
#define GENERAL_TIM_INT_IRQHandler       TIM2_IRQHandler
#define MOTOR_PUL_CHANNEL_x 						TIM_CHANNEL_1

#define MOTOR_PUL_GPIO_AF               GPIO_AF1_TIM2


extern TIM_HandleTypeDef TIM_TimeBaseStructure;

void TIMx_Configuration(void);


//���Ŷ���
/*******************************************************/
//Motor ���� 
#define MOTOR_DIR_PIN                  	GPIO_PIN_13   
#define MOTOR_DIR_GPIO_PORT            	GPIOB                     
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOB_CLK_ENABLE()

////Motor ����`
//#define MOTOR_PUL_PIN                  	GPIO_PIN_3            
//#define MOTOR_PUL_GPIO_PORT            	GPIOB                  
//#define MOTOR_PUL_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOB_CLK_ENABLE()

#define MOTOR_PUL_PIN                  	GPIO_PIN_15            
#define MOTOR_PUL_GPIO_PORT            	GPIOA
#define MOTOR_PUL_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOA_CLK_ENABLE()

//Motor ʹ�� 
#define MOTOR_EN_PIN                  	GPIO_PIN_6
#define MOTOR_EN_GPIO_PORT            	GPIOA                       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOA_CLK_ENABLE()
	
/************************************************************/
#define HIGH 1	//�ߵ�ƽ
#define LOW 0		//�͵�ƽ

#define ON 1	//��
#define OFF 0		//��

#define CLOCKWISE 			1//˳ʱ��
#define ANTI_CLOCKWISE	0//��ʱ��


//����ʹ������
/* ���κ꣬��������������һ��ʹ�� */
#define	GPIO_H(p,i)					{p->BSRR=i;}			  								//����Ϊ�ߵ�ƽ		
#define GPIO_L(p,i)					{p->BSRR=(uint32_t)i << 16;}				//����͵�ƽ
#define GPIO_T(p,i)					{p->ODR ^=i;}												//�����ת״̬

#define MOTOR_EN(x)					if(x)																			\
														{GPIO_H(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN);}\
														else																			\
														{GPIO_L(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN);}

#define MOTOR_PUL(x)				if(x)																				\
														{GPIO_H(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN);}\
														else																				\
														{GPIO_L(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN);}	

#define MOTOR_DIR(x)				if(x)																				\
														{GPIO_H(MOTOR_DIR_GPIO_PORT,MOTOR_DIR_PIN);}\
                            else																				\
                            {GPIO_L(MOTOR_DIR_GPIO_PORT,MOTOR_DIR_PIN);}	
														

#define MOTOR_PUL_T()				GPIO_T(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN);
	


extern void stepper_Init(void);
extern void stepper_turn(int tim,float angle,float subdivide,uint8_t dir);
#endif /* __STEP_MOTOR_INIT_H */
