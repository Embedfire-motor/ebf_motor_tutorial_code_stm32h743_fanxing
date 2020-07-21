/**
  ******************************************************************************
  * @file    bsp_basic_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   �ܻ��������Ʒ���
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 H743 ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./tim/bsp_general_tim.h"
#include "./usart/bsp_debug_usart.h"
#include <math.h>
#include <stdlib.h>

void TIM_SetTIMxCompare(TIM_TypeDef *TIMx,uint32_t channel,uint32_t compare);
void TIM_SetPWM_period(TIM_TypeDef* TIMx,uint32_t TIM_period);

/**
  * @brief  ����TIM�������PWMʱ�õ���I/O
  * @param  ��
  * @retval ��
  */
static void TIMx_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* ��ʱ��ͨ���������Ŷ˿�ʱ��ʹ�� */
	
	__HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* ��ʱ��ͨ��1��������IO��ʼ�� */
	/*�����������*/
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	/*������������ */ 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*���ø���*/
  GPIO_InitStruct.Alternate = GENERAL_TIM_GPIO_AF;
	
	/*ѡ��Ҫ���Ƶ�GPIO����*/	
	GPIO_InitStruct.Pin = GENERAL_TIM_CH1_PIN;
	/*���ÿ⺯����ʹ���������õ�GPIO_InitStructure��ʼ��GPIO*/
  HAL_GPIO_Init(GENERAL_TIM_CH1_GPIO_PORT, &GPIO_InitStruct);
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
TIM_HandleTypeDef  TIM_TimeBaseStructure;
static void TIM_PWMOUTPUT_Config(void)
{
  TIM_OC_InitTypeDef  TIM_OCInitStructure;  
	
  /*ʹ�ܶ�ʱ��*/
  GENERAL_TIM_CLK_ENABLE();
	
  TIM_TimeBaseStructure.Instance = GENERAL_TIM;
  /* �ۼ� TIM_Period�������һ�����»����ж�*/		
  //����ʱ����0������PWM_PERIOD_COUNT����ΪPWM_PERIOD_COUNT+1�Σ�Ϊһ����ʱ����
	TIM_TimeBaseStructure.Init.Period = PWM_PERIOD_COUNT;
    //��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
    //				PCLK1 = HCLK / 4 
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(PWM_PRESCALER_COUNT+1)
  TIM_TimeBaseStructure.Init.Prescaler = PWM_PRESCALER_COUNT;	
	
	/*������ʽ*/
  TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;
	/*����ʱ�ӷ�Ƶ*/
  TIM_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	/*��ʼ����ʱ��*/
  HAL_TIM_Base_Init(&TIM_TimeBaseStructure);
  
	/*PWMģʽ����*/
  TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM2;      // ����ΪPWMģʽ1
  TIM_OCInitStructure.Pulse = 0.5/20.0*PWM_PERIOD_COUNT;    // Ĭ��ռ�ձ�
  TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
	/*����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ*/
  TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;	
	
	/*����PWMͨ��*/
  HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure, &TIM_OCInitStructure, PWM_CHANNEL_1);
	/*��ʼ���PWM*/
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,PWM_CHANNEL_1);
}

/**
  * @brief  ����TIMͨ����ռ�ձ�
	* @param  channel		ͨ��	��1,2,3,4��
	* @param  compare		ռ�ձ�
	*	@note 	��
  * @retval ��
  */
void TIM2_SetPWM_pulse(uint32_t channel,int compare)
{
		switch(channel)
	{
		case TIM_CHANNEL_1:  	__HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_1,compare);break;
		case TIM_CHANNEL_2:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_2,compare);break;
		case TIM_CHANNEL_3:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_3,compare);break;
		case TIM_CHANNEL_4:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_4,compare);break;
	}
}


/**
  * @brief  ��ʼ������ͨ�ö�ʱ��
  * @param  ��
  * @retval ��
  */
void TIMx_Configuration(void)
{
	TIMx_GPIO_Config();
  
  TIM_PWMOUTPUT_Config();
}

/**
  * @brief  ���ö��ռ�ձ�
  * @param  angle: ռ�ձȣ���0.5/20.0*PWM_PERIOD_COUNT �� 2.5/20.0*PWM_PERIOD_COUNT��
  * @retval ��
  */
void set_steering_gear_dutyfactor(uint16_t dutyfactor)
{
  #if 1
  {
    /* �Գ�����Χ��ռ�ձȽ��б߽紦�� */
    dutyfactor = 0.5/20.0*PWM_PERIOD_COUNT > dutyfactor ? 0.5/20.0*PWM_PERIOD_COUNT : dutyfactor;
    dutyfactor = 2.5/20.0*PWM_PERIOD_COUNT < dutyfactor ? 2.5/20.0*PWM_PERIOD_COUNT : dutyfactor;
  }
  #endif
  
	TIM2_SetPWM_pulse(PWM_CHANNEL_1, dutyfactor);
}

/**
  * @brief  ���ö���Ƕ�
  * @param  angle: �Ƕȣ���0 �� 180�����Ϊ0��-180�㣩��
  * @retval ��
  */
void set_steering_gear_angle(uint16_t angle_temp)
{
  angle_temp = (0.5 + angle_temp / 180.0 * (2.5 - 0.5)) / 20.0 * PWM_PERIOD_COUNT;    // ����Ƕȶ�Ӧ��ռ�ձ�
  
  set_steering_gear_dutyfactor(angle_temp);    // ����ռ�ձ�
}

/**
  * @brief  ��ӡ��������
  * @param  ��
  * @retval ��
  */
void show_help(void)
{
    printf("����������������������������Ұ����������ʾ���򡪡�������������������������\n\r");
    printf("��������(�Իس�����)��\n\r");
    printf("< ? >     -�����˵�\n\r");
    printf("a[data]   -���ö���ĽǶȣ���Χ��%d��%d,�� a 90��\n\r", 0, 180);
}

extern uint16_t ChannelPulse;

/**
  * @brief  �����ڽ��յ�������
  * @param  ��
  * @retval ��
  */
void deal_serial_data(void)
{
    int angle_temp=0;
    
    //���յ���ȷ��ָ���Ϊ1
    char okCmd = 0;

    //����Ƿ���յ�ָ��
    if(receive_cmd == 1)
    {
      if(UART_RxBuffer[0] == 'a' || UART_RxBuffer[0] == 'A')
      {
        //�����ٶ�
        if(UART_RxBuffer[1] == ' ')
        {
          angle_temp = atoi((char const *)UART_RxBuffer+2);
          if(angle_temp>=0 && angle_temp <= 180)
          {
            printf("\n\r�Ƕ�: %d\n\r", angle_temp);
            ChannelPulse = (0.5 + angle_temp / 180.0 * (2.5 - 0.5)) / 20.0 * PWM_PERIOD_COUNT;    // ���°�ť���Ƶ�ռ�ձ�
            set_steering_gear_angle(angle_temp);
//            printf("\n\r�Ƕ�: %d\n\r", (uint16_t)(angle_temp/PWM_PERIOD_COUNT*20.0/(2.5-0.5)*180.0));
            okCmd = 1;
          }
        }
      }
      else if(UART_RxBuffer[0] == '?')
      {
        //��ӡ��������
        show_help();
        okCmd = 1;
      }
      //���ָ���������ӡ��������
      if(okCmd != 1)
      {
        printf("\n\r ������������������...\n\r");
        show_help();
      }

      //��մ��ڽ��ջ�������
      receive_cmd = 0;
      uart_FlushRxBuffer();

    }
}
