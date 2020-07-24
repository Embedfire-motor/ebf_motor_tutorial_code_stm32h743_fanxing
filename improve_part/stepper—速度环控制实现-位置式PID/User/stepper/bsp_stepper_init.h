#ifndef __BSP_STEP_MOTOR_INIT_H
#define	__BSP_STEP_MOTOR_INIT_H

#include "stm32h7xx_hal.h"

/*�궨��*/
/*******************************************************/
//�궨���Ӧ������Ľӿ� 1 ��2 ��3 ��4
#define CHANNEL_SW 1

#if(CHANNEL_SW == 1)
//Motor ���� 
#define MOTOR_DIR_PIN                  	GPIO_PIN_1   
#define MOTOR_DIR_GPIO_PORT            	GPIOE                    
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOE_CLK_ENABLE()

//Motor ʹ�� 
#define MOTOR_EN_PIN                  	GPIO_PIN_0
#define MOTOR_EN_GPIO_PORT            	GPIOE                       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOE_CLK_ENABLE()
	
//Motor ����
#define MOTOR_PUL_IRQn                  TIM8_CC_IRQn
#define MOTOR_PUL_IRQHandler            TIM8_CC_IRQHandler

#define MOTOR_PUL_TIM                   TIM8
#define MOTOR_PUL_CLK_ENABLE()  		    __TIM8_CLK_ENABLE()

#define MOTOR_PUL_PORT       			      GPIOI
#define MOTOR_PUL_PIN             		  GPIO_PIN_5
#define MOTOR_PUL_GPIO_CLK_ENABLE()		  __HAL_RCC_GPIOI_CLK_ENABLE()

#define MOTOR_PUL_GPIO_AF               GPIO_AF3_TIM8
#define MOTOR_PUL_CHANNEL_x             TIM_CHANNEL_1

#elif(CHANNEL_SW == 2)

//Motor ���� 
#define MOTOR_DIR_PIN                  	GPIO_PIN_8
#define MOTOR_DIR_GPIO_PORT            	GPIOI          
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOI_CLK_ENABLE()

//Motor ʹ�� 
#define MOTOR_EN_PIN                  	GPIO_PIN_4
#define MOTOR_EN_GPIO_PORT            	GPIOE                       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOE_CLK_ENABLE()
	
//Motor ����
#define MOTOR_PUL_IRQn                  TIM8_CC_IRQn
#define MOTOR_PUL_IRQHandler            TIM8_CC_IRQHandler

#define MOTOR_PUL_TIM                   TIM8
#define MOTOR_PUL_CLK_ENABLE()  		    __TIM8_CLK_ENABLE()

#define MOTOR_PUL_PORT       			      GPIOI
#define MOTOR_PUL_PIN             	  	GPIO_PIN_6
#define MOTOR_PUL_GPIO_CLK_ENABLE()		  __HAL_RCC_GPIOI_CLK_ENABLE()

#define MOTOR_PUL_GPIO_AF               GPIO_AF3_TIM8
#define MOTOR_PUL_CHANNEL_x             TIM_CHANNEL_2

#elif(CHANNEL_SW == 3)

//Motor ���� 
#define MOTOR_DIR_PIN                  	GPIO_PIN_11
#define MOTOR_DIR_GPIO_PORT            	GPIOI          
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOI_CLK_ENABLE()

//Motor ʹ�� 
#define MOTOR_EN_PIN                  	GPIO_PIN_10
#define MOTOR_EN_GPIO_PORT            	GPIOI                 
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOI_CLK_ENABLE()
	
//Motor ����
#define MOTOR_PUL_IRQn                  TIM8_CC_IRQn
#define MOTOR_PUL_IRQHandler            TIM8_CC_IRQHandler

#define MOTOR_PUL_TIM                   TIM8
#define MOTOR_PUL_CLK_ENABLE()  		    __TIM8_CLK_ENABLE()

#define MOTOR_PUL_PORT       			      GPIOI
#define MOTOR_PUL_PIN             		  GPIO_PIN_7
#define MOTOR_PUL_GPIO_CLK_ENABLE()		  __HAL_RCC_GPIOI_CLK_ENABLE()

#define MOTOR_PUL_GPIO_AF               GPIO_AF3_TIM8
#define MOTOR_PUL_CHANNEL_x             TIM_CHANNEL_3

#elif(CHANNEL_SW == 4)

//Motor ���� 
#define MOTOR_DIR_PIN                  	GPIO_PIN_2
#define MOTOR_DIR_GPIO_PORT            	GPIOF
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOF_CLK_ENABLE()

//Motor ʹ�� 
#define MOTOR_EN_PIN                  	GPIO_PIN_1
#define MOTOR_EN_GPIO_PORT            	GPIOF       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOF_CLK_ENABLE()
	
//Motor ����
#define MOTOR_PUL_IRQn                  TIM8_CC_IRQn
#define MOTOR_PUL_IRQHandler            TIM8_CC_IRQHandler

#define MOTOR_PUL_TIM                   TIM8
#define MOTOR_PUL_CLK_ENABLE()  		    __TIM8_CLK_ENABLE()

#define MOTOR_PUL_PORT       			      GPIOC
#define MOTOR_PUL_PIN                   GPIO_PIN_9
#define MOTOR_PUL_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()

#define MOTOR_PUL_GPIO_AF               GPIO_AF3_TIM8
#define MOTOR_PUL_CHANNEL_x             TIM_CHANNEL_4

#endif



/*Ƶ����ز���*/
//��ʱ��ʵ��ʱ��Ƶ��Ϊ��240MHz/TIM_PRESCALER
//240/TIM_PRESCALER = 20MHz
//������Ҫ��Ƶ�ʿ����Լ�����
#define TIM_PRESCALER                240
// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define TIM_PERIOD                   0xFFFF

/************************************************************/
#define HIGH GPIO_PIN_SET	      //�ߵ�ƽ
#define LOW  GPIO_PIN_RESET	    //�͵�ƽ

#define ON  LOW	                //��
#define OFF HIGH	              //��

#define CW 	HIGH		            //˳ʱ��
#define CCW LOW      	          //��ʱ��



//����ʹ������
/* ���κ꣬��������������һ��ʹ�� */
#define MOTOR_EN(x)					HAL_GPIO_WritePin(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN,x)
#define MOTOR_PUL(x)				HAL_GPIO_WritePin(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN,x)
#define MOTOR_DIR(x)				HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT,MOTOR_DIR_PIN,x)

extern TIM_HandleTypeDef TIM_StepperHandle;
extern void stepper_Init(void);

#endif /* __STEP_MOTOR_INIT_H */
