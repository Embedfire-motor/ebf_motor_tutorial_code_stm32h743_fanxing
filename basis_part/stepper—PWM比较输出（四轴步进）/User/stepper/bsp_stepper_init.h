#ifndef __BSP_STEP_MOTOR_INIT_H
#define __BSP_STEP_MOTOR_INIT_H

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"


/* ��������ṹ�� */
typedef struct{
  uint16_t pul_pin;                     //�������������ź�
  uint16_t dir_pin;                     //����������ź�
  uint16_t en_pin;                      //���ʹ�����ź�
  uint32_t pul_channel;                 //����������ͨ��
  GPIO_TypeDef *pul_port;               //����������Ŷ˿�
  GPIO_TypeDef *dir_port;               //����������Ŷ˿�
  GPIO_TypeDef *en_port;                //���ʹ�����Ŷ˿�
  uint16_t oc_pulse_num;                //����Ƚϼ���ֵ��ֵԽС���ת��Խ��
}Stepper_TypeDef;


/*�궨��*/
/*******************************************************/
#define MOTOR_PUL_IRQn                   TIM8_CC_IRQn
#define MOTOR_PUL_IRQHandler             TIM8_CC_IRQHandler
#define MOTOR_PUL_TIM                    TIM8
#define MOTOR_PUL_CLK_ENABLE()  		     __TIM8_CLK_ENABLE()
#define MOTOR_PUL_GPIO_AF                GPIO_AF3_TIM8

//ͨ��1
//Motor ����
#define MOTOR_DIR1_PIN                  	GPIO_PIN_1   
#define MOTOR_DIR1_GPIO_PORT            	GPIOE
#define MOTOR_DIR1_GPIO_CLK_ENABLE()   	  __HAL_RCC_GPIOE_CLK_ENABLE()

//Motor ʹ��
#define MOTOR_EN1_PIN                  	  GPIO_PIN_0
#define MOTOR_EN1_GPIO_PORT            	  GPIOE
#define MOTOR_EN1_GPIO_CLK_ENABLE()    	  __HAL_RCC_GPIOE_CLK_ENABLE()

//Motor ����
#define MOTOR_PUL1_PORT       			      GPIOI
#define MOTOR_PUL1_PIN             		    GPIO_PIN_5
#define MOTOR_PUL1_GPIO_CLK_ENABLE()		  __HAL_RCC_GPIOI_CLK_ENABLE()

//��ʱ��ͨ��
#define MOTOR_PUL1_CHANNEL                TIM_CHANNEL_1


//ͨ��2
//Motor ����
#define MOTOR_DIR2_PIN                  	GPIO_PIN_8
#define MOTOR_DIR2_GPIO_PORT            	GPIOI          
#define MOTOR_DIR2_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOI_CLK_ENABLE()

//Motor ʹ��
#define MOTOR_EN2_PIN                     GPIO_PIN_4
#define MOTOR_EN2_GPIO_PORT               GPIOE                       
#define MOTOR_EN2_GPIO_CLK_ENABLE()    	  __HAL_RCC_GPIOE_CLK_ENABLE()

//Motor ����
#define MOTOR_PUL2_PORT       			      GPIOI
#define MOTOR_PUL2_PIN             		    GPIO_PIN_6
#define MOTOR_PUL2_GPIO_CLK_ENABLE()	  	__HAL_RCC_GPIOI_CLK_ENABLE()

//��ʱ��ͨ��
#define MOTOR_PUL2_CHANNEL                TIM_CHANNEL_2


//ͨ��3
//Motor ����
#define MOTOR_DIR3_PIN                  	GPIO_PIN_11
#define MOTOR_DIR3_GPIO_PORT            	GPIOI          
#define MOTOR_DIR3_GPIO_CLK_ENABLE()   	  __HAL_RCC_GPIOI_CLK_ENABLE()

//Motor ʹ��
#define MOTOR_EN3_PIN                  	  GPIO_PIN_10
#define MOTOR_EN3_GPIO_PORT            	  GPIOI                 
#define MOTOR_EN3_GPIO_CLK_ENABLE()    	  __HAL_RCC_GPIOI_CLK_ENABLE()

//Motor ����
#define MOTOR_PUL3_PORT       			      GPIOI
#define MOTOR_PUL3_PIN             		    GPIO_PIN_7
#define MOTOR_PUL3_GPIO_CLK_ENABLE()		  __HAL_RCC_GPIOI_CLK_ENABLE()

//��ʱ��ͨ��
#define MOTOR_PUL3_CHANNEL                TIM_CHANNEL_3


//ͨ��4
//Motor ����
#define MOTOR_DIR4_PIN                  	GPIO_PIN_2
#define MOTOR_DIR4_GPIO_PORT            	GPIOF
#define MOTOR_DIR4_GPIO_CLK_ENABLE()   	  __HAL_RCC_GPIOF_CLK_ENABLE()

//Motor ʹ��
#define MOTOR_EN4_PIN                  	  GPIO_PIN_1
#define MOTOR_EN4_GPIO_PORT            	  GPIOF       
#define MOTOR_EN4_GPIO_CLK_ENABLE()    	  __HAL_RCC_GPIOF_CLK_ENABLE()

//Motor ����
#define MOTOR_PUL4_PORT       			      GPIOC
#define MOTOR_PUL4_PIN             		    GPIO_PIN_9
#define MOTOR_PUL4_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()

//��ʱ��ͨ��
#define MOTOR_PUL4_CHANNEL                TIM_CHANNEL_4


//����ʹ������
/* ���κ꣬��������������һ��ʹ�� */
#define MOTOR_OFFLINE(port, pin, x)       HAL_GPIO_WritePin(port, pin, x)
#define MOTOR_PUL(port, pin, x)           HAL_GPIO_WritePin(port, pin, x)
#define MOTOR_DIR(port, pin, x)           HAL_GPIO_WritePin(port, pin, x)

/*Ƶ����ز���*/
//��ʱ��ʵ��ʱ��Ƶ��Ϊ��240MHz/TIM_PRESCALER
//240/TIM_PRESCALER=2MHz
//������Ҫ��Ƶ�ʿ����Լ�����
#define TIM_PRESCALER                120
// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define TIM_PERIOD                   0xFFFF

/************************************************************/
#define HIGH GPIO_PIN_SET	  //�ߵ�ƽ
#define LOW  GPIO_PIN_RESET	//�͵�ƽ

#define ON   LOW	          //��
#define OFF  HIGH	          //��

#define CW   HIGH		        //˳ʱ��
#define CCW  LOW      	    //��ʱ��

/* ����������� */
extern Stepper_TypeDef step_motor[4];

void stepper_Init(void);
void stepper_Start(uint32_t channel);
void stepper_Stop(uint32_t channel);

#endif /* __BSP_STEPPER_INIT_H */
