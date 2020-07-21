/**
  ******************************************************************************
  * @file    bsp_basic_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   ͨ�ö�ʱ����ʱ����
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
#include "./led/bsp_led.h" 
TIM_HandleTypeDef TIM_Base;


 /**
  * @brief  ͨ�ö�ʱ�� TIMx,x[2-5,12-14,15-17]�ж����ȼ�����
  * @param  ��
  * @retval ��
  */
static void TIMx_NVIC_Configuration(void)
{
    HAL_NVIC_SetPriority(GENERAL_TIM_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(GENERAL_TIM_IRQ);
}

                         
/*
 * ע�⣺TIM_Base_InitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
 * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
 * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
 *-----------------------------------------------------------------------------
 * Prescaler         ����
 * CounterMode			 TIMx,x[6,7]û�У��������У�������ʱ����
 * Period            ����
 * ClockDivision     TIMx,x[6,7]û�У���������(������ʱ��)
 * RepetitionCounter TIMx,x[1,8]����(�߼���ʱ��)
 *-----------------------------------------------------------------------------
 */
static void TIM_Mode_Config(void)
{
    GENERAL_TIM_CLK_ENABLE();
     
    TIM_Base.Instance = GENERAL_TIM;
    /* �ۼ� TIM_Period�������һ�����»����ж�*/		
    //����ʱ����0������10000-1����Ϊ10000�Σ�Ϊһ����ʱ����
    TIM_Base.Init.Period = 5000 - 1;
    //��ʱ��ʱ��ԴTIMxCLK = 2 * PCLK1  
    //				PCLK1 = HCLK / 4 
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
    // �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=10000Hz
    TIM_Base.Init.Prescaler =  24000 - 1;
		// ������ʽ
	  TIM_Base.Init.CounterMode=TIM_COUNTERMODE_UP;
	  // ����ʱ�ӷ�Ƶ
	  TIM_Base.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    // ��ʼ����ʱ��TIM
    HAL_TIM_Base_Init(&TIM_Base);
    // ������ʱ�������ж�
    HAL_TIM_Base_Start_IT(&TIM_Base);
}

/**
  * @brief  ��ʼ��������ʱ����ʱ��500ms����һ���ж�
  * @param  ��
  * @retval ��
  */
void TIMx_Configuration(void)
{
    TIMx_NVIC_Configuration(); 
	
  	TIM_Mode_Config();
}



/*********************************************END OF FILE**********************/

