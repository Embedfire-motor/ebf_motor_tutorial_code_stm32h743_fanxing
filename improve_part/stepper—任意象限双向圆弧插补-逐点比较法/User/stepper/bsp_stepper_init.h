#ifndef __BSP_STEPPER_INIT_H
#define	__BSP_STEPPER_INIT_H

#include "stm32h7xx_hal.h"

/* ��������ṹ�� */
typedef struct{
  uint16_t pul_pin;
  uint16_t dir_pin;
  uint16_t en_pin;
  uint32_t pul_channel;
  GPIO_TypeDef *pul_port;
  GPIO_TypeDef *dir_port;
  GPIO_TypeDef *en_port;
  uint16_t oc_pulse_num;
}Stepper_TypeDef;


/*�궨��*/
/*******************************************************/
#define MOTOR_PUL_TIM                        TIM8
#define MOTOR_PUL_IRQn                       TIM8_UP_TIM13_IRQn
#define MOTOR_PUL_IRQHandler                 TIM8_UP_TIM13_IRQHandler
#define MOTOR_PUL_CLK_ENABLE()               __TIM8_CLK_ENABLE()
#define MOTOR_PUL_GPIO_AF                    GPIO_AF3_TIM8

/*********************X�������Ŷ���*******************/
//Motor ����
#define X_MOTOR_DIR_PIN                      GPIO_PIN_1
#define X_MOTOR_DIR_GPIO_PORT                GPIOE
#define X_MOTOR_DIR_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOE_CLK_ENABLE()

//Motor ʹ��
#define X_MOTOR_EN_PIN                       GPIO_PIN_0
#define X_MOTOR_EN_GPIO_PORT                 GPIOE
#define X_MOTOR_EN_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOE_CLK_ENABLE()

//Motor ����
#define X_MOTOR_PUL_PORT                     GPIOI
#define X_MOTOR_PUL_PIN                      GPIO_PIN_5
#define X_MOTOR_PUL_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOI_CLK_ENABLE()

//��ʱ��ͨ��
#define X_MOTOR_PUL_CHANNEL                  TIM_CHANNEL_1

/*********************Y�������Ŷ���*******************/
//Motor ����
#define Y_MOTOR_DIR_PIN                      GPIO_PIN_8
#define Y_MOTOR_DIR_GPIO_PORT                GPIOI          
#define Y_MOTOR_DIR_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOI_CLK_ENABLE()

//Motor ʹ��
#define Y_MOTOR_EN_PIN                       GPIO_PIN_4
#define Y_MOTOR_EN_GPIO_PORT                 GPIOE                       
#define Y_MOTOR_EN_GPIO_CLK_ENABLE()      	 __HAL_RCC_GPIOE_CLK_ENABLE()

//Motor ����
#define Y_MOTOR_PUL_PORT       			         GPIOI
#define Y_MOTOR_PUL_PIN             		     GPIO_PIN_6
#define Y_MOTOR_PUL_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOI_CLK_ENABLE()

//��ʱ��ͨ��
#define Y_MOTOR_PUL_CHANNEL                  TIM_CHANNEL_2


//����ʹ������
/* ���κ꣬��������������һ��ʹ�� */
#define MOTOR_PUL(port, pin, x)              HAL_GPIO_WritePin(port, pin, x)
#define MOTOR_DIR(port, pin, x)              HAL_GPIO_WritePin(port, pin, x)
#define MOTOR_OFFLINE(port, pin, x)          HAL_GPIO_WritePin(port, pin, x)
#define MOTOR_START(tim, channel, status)    TIM_CCxChannelCmd(tim, channel, status)
#define MOTOR_STOP(tim, channel, status)     TIM_CCxChannelCmd(tim, channel, status)

/*Ƶ����ز���*/
//��ʱ��ʵ��ʱ��Ƶ��Ϊ��240MHz/TIM_PRESCALER
//240/TIM_PRESCALER=30MHz
//������Ҫ��Ƶ�ʿ����Լ�����
#define TIM_PRESCALER                12
// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define TIM_PERIOD                   0xFFFF

/************************************************************/
#define HIGH GPIO_PIN_SET	  //�ߵ�ƽ
#define LOW  GPIO_PIN_RESET	//�͵�ƽ

#define ON   LOW	          //��
#define OFF  HIGH	          //��

#define CW   HIGH		        //˳ʱ��
#define CCW  LOW      	    //��ʱ��


extern TIM_HandleTypeDef TIM_StepperHandle;
extern Stepper_TypeDef step_motor[2];

void stepper_Init(void);

#endif /* __BSP_STEPPER_INIT_H */
