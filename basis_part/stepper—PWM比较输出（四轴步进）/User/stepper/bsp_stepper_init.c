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

/* ��ʱ����� */
TIM_HandleTypeDef TIM_StepperHandle;

/* ����������� */
Stepper_TypeDef step_motor[4] = 
{
  {MOTOR_PUL1_PIN, MOTOR_DIR1_PIN, MOTOR_EN1_PIN, MOTOR_PUL1_CHANNEL, MOTOR_PUL1_PORT, MOTOR_DIR1_GPIO_PORT, MOTOR_EN1_GPIO_PORT, 100},
  {MOTOR_PUL2_PIN, MOTOR_DIR2_PIN, MOTOR_EN2_PIN, MOTOR_PUL2_CHANNEL, MOTOR_PUL2_PORT, MOTOR_DIR2_GPIO_PORT, MOTOR_EN2_GPIO_PORT, 150},
  {MOTOR_PUL3_PIN, MOTOR_DIR3_PIN, MOTOR_EN3_PIN, MOTOR_PUL3_CHANNEL, MOTOR_PUL3_PORT, MOTOR_DIR3_GPIO_PORT, MOTOR_EN3_GPIO_PORT, 200},
  {MOTOR_PUL4_PIN, MOTOR_DIR4_PIN, MOTOR_EN4_PIN, MOTOR_PUL4_CHANNEL, MOTOR_PUL4_PORT, MOTOR_DIR4_GPIO_PORT, MOTOR_EN4_GPIO_PORT, 250},
};

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
  
  MOTOR_DIR1_GPIO_CLK_ENABLE();
  MOTOR_EN1_GPIO_CLK_ENABLE();
  MOTOR_PUL1_GPIO_CLK_ENABLE();

  MOTOR_DIR2_GPIO_CLK_ENABLE();
  MOTOR_EN2_GPIO_CLK_ENABLE();
  MOTOR_PUL2_GPIO_CLK_ENABLE();
  
  MOTOR_DIR3_GPIO_CLK_ENABLE();
  MOTOR_EN3_GPIO_CLK_ENABLE();
  MOTOR_PUL3_GPIO_CLK_ENABLE();
  
  MOTOR_DIR4_GPIO_CLK_ENABLE();
  MOTOR_EN4_GPIO_CLK_ENABLE();
  MOTOR_PUL4_GPIO_CLK_ENABLE();

  /* ��ȡ����Ԫ�ظ��� */
  uint8_t member_count = sizeof(step_motor)/sizeof(Stepper_TypeDef);
  
  for(uint8_t i = 0; i < member_count; i++)
  {
    /*ѡ��Ҫ���Ƶ�GPIO����*/															   
    GPIO_InitStruct.Pin = step_motor[i].dir_pin;	
    /*�������ŵ��������Ϊ�������*/
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;  
    GPIO_InitStruct.Pull =GPIO_PULLUP;
    /*������������Ϊ���� */   
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    /*Motor �������� ��ʼ��*/
    HAL_GPIO_Init(step_motor[i].dir_port, &GPIO_InitStruct);
    MOTOR_DIR(step_motor[i].dir_port, step_motor[i].dir_pin, CW);
    
    /*Motor ʹ������ ��ʼ��*/
    GPIO_InitStruct.Pin = step_motor[i].en_pin;
    HAL_GPIO_Init(step_motor[i].en_port, &GPIO_InitStruct);
    MOTOR_OFFLINE(step_motor[i].en_port, step_motor[i].en_pin, ON);
    
    /* ��ʱ�����ͨ����������IO��ʼ�� */
    /*�����������*/
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    /*������������ */ 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    /*���ø���*/
    GPIO_InitStruct.Alternate = MOTOR_PUL_GPIO_AF;
    /*���ø���*/
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    /*ѡ��Ҫ���Ƶ�GPIO����*/	
    GPIO_InitStruct.Pin = step_motor[i].pul_pin;
    /*Motor �������� ��ʼ��*/
    HAL_GPIO_Init(step_motor[i].pul_port, &GPIO_InitStruct);
  }
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
static void TIM_PWMOUTPUT_Config(void)
{
	TIM_OC_InitTypeDef  TIM_OCInitStructure;

  /* ��ȡ����Ԫ�ظ��� */
  uint8_t member_count = sizeof(step_motor)/sizeof(Stepper_TypeDef);
  
	/*ʹ�ܶ�ʱ��*/
	MOTOR_PUL_CLK_ENABLE();

	TIM_StepperHandle.Instance = MOTOR_PUL_TIM;    
	/* �ۼ� TIM_Period�������һ�����»����ж�*/		
	//����ʱ����0������TIM_PERIOD����ΪTIM_PERIOD�Σ�Ϊһ����ʱ����
	TIM_StepperHandle.Init.Period = TIM_PERIOD;
	// ͨ�ÿ��ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK/2=168MHz 
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=2MHz
	TIM_StepperHandle.Init.Prescaler = TIM_PRESCALER-1;                

	/*������ʽ*/
	TIM_StepperHandle.Init.CounterMode = TIM_COUNTERMODE_UP;            
	/*����ʱ�ӷ�Ƶ*/	
	TIM_StepperHandle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   
	TIM_StepperHandle.Init.RepetitionCounter = 0 ;  		
	/*��ʼ����ʱ��*/
	HAL_TIM_OC_Init(&TIM_StepperHandle);

	/*PWMģʽ����--��������Ϊ����Ƚ�ģʽ*/
	TIM_OCInitStructure.OCMode = TIM_OCMODE_TOGGLE; 
	/*�Ƚ�����ļ���ֵ*/
	TIM_OCInitStructure.Pulse = TIM_PERIOD;                    
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
  
  for(uint8_t i = 0; i < member_count; i++)
  {
    /* �����Ƚ������ʹ���ж� */
    HAL_TIM_OC_ConfigChannel(&TIM_StepperHandle, &TIM_OCInitStructure, step_motor[i].pul_channel);
    HAL_TIM_OC_Start_IT(&TIM_StepperHandle, step_motor[i].pul_channel);
  }

  /* ������ʱ�� */
	HAL_TIM_Base_Start(&TIM_StepperHandle);
}

/**
  * @brief  ��ʱ���жϷ�����
	*	@note   ��
  * @retval ��
  */
void MOTOR_PUL_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TIM_StepperHandle);
}

/**
  * @brief  ��ʱ���Ƚ��жϻص�����
  * @param  htim����ʱ�����ָ��
	*	@note   ��
  * @retval ��
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t channel = htim->Channel;
  
  /* ��ȡ��ǰ���� */
  uint32_t count = __HAL_TIM_GET_COUNTER(htim);
  
  switch(channel)
  {
    case HAL_TIM_ACTIVE_CHANNEL_1:
      /* ���ñȽ���ֵ */
      __HAL_TIM_SET_COMPARE(&TIM_StepperHandle, MOTOR_PUL1_CHANNEL, count + step_motor[0].oc_pulse_num);
      break;
    case HAL_TIM_ACTIVE_CHANNEL_2:
      /* ���ñȽ���ֵ */
      __HAL_TIM_SET_COMPARE(&TIM_StepperHandle, MOTOR_PUL2_CHANNEL, count + step_motor[1].oc_pulse_num);
      break;
    case HAL_TIM_ACTIVE_CHANNEL_3:
      /* ���ñȽ���ֵ */
      __HAL_TIM_SET_COMPARE(&TIM_StepperHandle, MOTOR_PUL3_CHANNEL, count + step_motor[2].oc_pulse_num);
      break;
    case HAL_TIM_ACTIVE_CHANNEL_4:
      /* ���ñȽ���ֵ */
      __HAL_TIM_SET_COMPARE(&TIM_StepperHandle, MOTOR_PUL4_CHANNEL, count + step_motor[3].oc_pulse_num);
      break;
  }
}


/**
  * @brief  ���������ʼ��
  * @param  *step_motor����������ṹ��ָ��
  * @param  member_count����Ҫ��ʼ���Ĳ����������
	*	@note   ��
  * @retval ��
  */
void stepper_Init(void)
{
	/*���IO����*/
	Stepper_GPIO_Config();
	/*��ʱ��PWM�������*/
	TIM_PWMOUTPUT_Config();
	/*�ж�����*/
	TIMx_NVIC_Configuration();
}

/**
  * @brief  �������
  * @param  channel�����ͨ��
	*	@note   ��
  * @retval ��
  */
void stepper_Start(uint32_t channel)
{
  TIM_CCxChannelCmd(MOTOR_PUL_TIM, channel, TIM_CCx_ENABLE);
}

/**
  * @brief  ֹͣ���
  * @param  channel�����ͨ��
	*	@note   ��
  * @retval ��
  */
void stepper_Stop(uint32_t channel)
{
  TIM_CCxChannelCmd(MOTOR_PUL_TIM, channel, TIM_CCx_DISABLE);
}
