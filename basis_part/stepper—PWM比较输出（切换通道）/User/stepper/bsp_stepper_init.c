/**
  ******************************************************************************
  * @file    bsp_stepper_init.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   ���������ʼ��
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 H743 ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
#include "./stepper/bsp_stepper_init.h"
#include "./delay/core_delay.h"   
#include "stm32h7xx.h"

void TIM_SetTIMxCompare(TIM_TypeDef *TIMx,uint32_t channel,uint32_t compare);
void TIM_SetPWM_period(TIM_TypeDef* TIMx,uint32_t TIM_period);

TIM_HandleTypeDef TIM_TimeBaseStructure;
//__IO uint16_t OC_Pulse_num = 500;     //�Ƚ�����ļ���ֵ
__IO uint16_t OC_Pulse_num = 200;     //�Ƚ�����ļ���ֵ


 /**
  * @brief  �ж����ȼ�����
  * @param  ��
  * @retval ��
  */
static void TIMx_NVIC_Configuration(void)
{
  /* �����ж����� */
  HAL_NVIC_SetPriority(MOTOR_PUL_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MOTOR_PUL_IRQn);
}

/**
  * @brief  ����TIM�������PWMʱ�õ���I/O
  * @param  ��
  * @retval ��
  */
static void Stepper_GPIO_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*����Motor��ص�GPIO����ʱ��*/
	MOTOR_DIR_GPIO_CLK_ENABLE();
	MOTOR_PUL_GPIO_CLK_ENABLE();
	MOTOR_EN_GPIO_CLK_ENABLE();

	/*ѡ��Ҫ���Ƶ�GPIO����*/															   
	GPIO_InitStruct.Pin = MOTOR_DIR_PIN;	
	/*�������ŵ��������Ϊ�������*/
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;  
	GPIO_InitStruct.Pull =GPIO_PULLUP;
	/*������������Ϊ���� */   
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*Motor �������� ��ʼ��*/
	HAL_GPIO_Init(MOTOR_DIR_GPIO_PORT, &GPIO_InitStruct);	

	/*Motor ʹ������ ��ʼ��*/
	GPIO_InitStruct.Pin = MOTOR_EN_PIN;	
	HAL_GPIO_Init(MOTOR_EN_GPIO_PORT, &GPIO_InitStruct);	


	/* ��ʱ��ͨ��1��������IO��ʼ�� */
	/*�����������*/
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	/*������������ */ 
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*���ø���*/
	GPIO_InitStruct.Alternate = MOTOR_PUL_GPIO_AF;
	/*���ø���*/
	GPIO_InitStruct.Pull =GPIO_PULLUP;
	/*ѡ��Ҫ���Ƶ�GPIO����*/	
	GPIO_InitStruct.Pin = MOTOR_PUL_PIN;
	/*Motor �������� ��ʼ��*/
	HAL_GPIO_Init(MOTOR_PUL_PORT, &GPIO_InitStruct);			
}


/*
 * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
 * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
 * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         ����
 * TIM_CounterMode			 TIMx,x[6,7]û�У��������У�������ʱ����
 * TIM_Period            ����
 * TIM_ClockDivision     TIMx,x[6,7]û�У���������(������ʱ��)
 * TIM_RepetitionCounter TIMx,x[1,8]����(�߼���ʱ��)
 *-----------------------------------------------------------------------------
 */
void TIM_PWMOUTPUT_Config(void)
{
	TIM_OC_InitTypeDef  TIM_OCInitStructure;  	
	/*ʹ�ܶ�ʱ��*/
	MOTOR_PUL_CLK_ENABLE();

	TIM_TimeBaseStructure.Instance = MOTOR_PUL_TIM;    
	/* �ۼ� TIM_Period�������һ�����»����ж�*/		
	//����ʱ����0������10000����Ϊ10000�Σ�Ϊһ����ʱ����
	TIM_TimeBaseStructure.Init.Period = TIM_PERIOD; 
	//��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
	//				PCLK1 = HCLK / 4 
	//				=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=1MHz
	TIM_TimeBaseStructure.Init.Prescaler = 240-1;                

	/*������ʽ*/
	TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;            
	/*����ʱ�ӷ�Ƶ*/	
	TIM_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   
	TIM_TimeBaseStructure.Init.RepetitionCounter = 0 ;  		
	/*��ʼ����ʱ��*/
	HAL_TIM_OC_Init(&TIM_TimeBaseStructure);

	/*PWMģʽ����--��������Ϊ����Ƚ�ģʽ*/
	TIM_OCInitStructure.OCMode = TIM_OCMODE_TOGGLE; 
	/*�Ƚ�����ļ���ֵ*/
	TIM_OCInitStructure.Pulse = OC_Pulse_num;                    
	/*����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ*/
	TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;          
	/*���û���ͨ������ļ���*/
	TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_LOW; 
	/*����ģʽ����*/
	TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;   
	/*���е�ƽ*/
	TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;  
	/*����ͨ������*/
	TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET; 
	HAL_TIM_OC_ConfigChannel(&TIM_TimeBaseStructure, &TIM_OCInitStructure, MOTOR_PUL_CHANNEL_x);

	/* ȷ����ʱ�� */
	HAL_TIM_Base_Start(&TIM_TimeBaseStructure);
	/* �����Ƚ������ʹ���ж� */
	HAL_TIM_OC_Start_IT(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x);
	/*ʹ�ܱȽ�ͨ��*/
	TIM_CCxChannelCmd(MOTOR_PUL_TIM,MOTOR_PUL_CHANNEL_x,TIM_CCx_ENABLE);

}

/**
  * @brief  ��ʱ���жϺ���
	*	@note 		��
  * @retval ��
  */
void MOTOR_PUL_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TIM_TimeBaseStructure);
}

/**
  * @brief  ��ʱ���Ƚ��ж�
  * @param  htim����ʱ�����ָ��
	*	@note 		��
  * @retval ��
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t count;
  __IO uint32_t temp_val;
	/*��ȡ��ǰ����*/
  count=__HAL_TIM_GET_COUNTER(&TIM_TimeBaseStructure);
	/*����Ƚ���ֵ*/
  temp_val = TIM_PERIOD & (count+OC_Pulse_num); 
		
	/*���ñȽ���ֵ*/
  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x,temp_val);
	
}

/**
  * @brief  ����TIMͨ����ռ�ձ�
	* @param  channel		ͨ��	��1,2,3,4��
	* @param  compare		ռ�ձ�
	*	@note 	��
  * @retval ��
  */
void TIM8_SetPWM_pulse(int channel,int compare)
{
	switch(channel)
	{
		case 1:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_1,compare);break;
		case 2:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_2,compare);break;
		case 3:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_3,compare);break;
		case 4:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_4,compare);break;
	}
}



/**
  * @brief  ���ų�ʼ��
  * @retval ��
  */
void stepper_Init()
{
	/*���IO����*/
	Stepper_GPIO_Config();
	/*��ʱ��PWM�������*/
	TIM_PWMOUTPUT_Config();
	/*�ж�����*/
	TIMx_NVIC_Configuration();
}


