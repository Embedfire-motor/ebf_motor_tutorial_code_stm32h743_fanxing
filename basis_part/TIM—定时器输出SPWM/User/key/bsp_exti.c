/**
  ******************************************************************************
  * @file    bsp_exti.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   I/O���ж�Ӧ��bsp
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 F407 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./key/bsp_exti.h"
#include "./led/bsp_led.h"   
#include "./stepper/bsp_stepper_init.h"

 /**
  * @brief  ���� PA0 Ϊ���жϿڣ��������ж����ȼ�
  * @param  ��
  * @retval ��
  */
void EXTI_Key_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; 

    /*��������GPIO�ڵ�ʱ��*/
    KEY1_INT_GPIO_CLK_ENABLE();
    KEY3_INT_GPIO_CLK_ENABLE();

    /* ѡ�񰴼�1������ */ 
    GPIO_InitStructure.Pin = KEY1_INT_GPIO_PIN;
    /* ��������Ϊ����ģʽ */ 
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;	    		
    /* �������Ų�����Ҳ������ */
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    /* ʹ������Ľṹ���ʼ������ */
    HAL_GPIO_Init(KEY1_INT_GPIO_PORT, &GPIO_InitStructure); 
    /* ���� EXTI �ж�Դ ��key1 ���š������ж����ȼ�*/
    HAL_NVIC_SetPriority(KEY1_INT_EXTI_IRQ, 0, 0);
    /* ʹ���ж� */
    HAL_NVIC_EnableIRQ(KEY1_INT_EXTI_IRQ);

    /* ѡ�񰴼�2������ */ 
    GPIO_InitStructure.Pin = KEY3_INT_GPIO_PIN;  
    /* ����������������ͬ */
    HAL_GPIO_Init(KEY3_INT_GPIO_PORT, &GPIO_InitStructure);      
    /* ���� EXTI �ж�Դ ��key1 ���š������ж����ȼ�*/
    HAL_NVIC_SetPriority(KEY3_INT_EXTI_IRQ, 0, 0);
    /* ʹ���ж� */
    HAL_NVIC_EnableIRQ(KEY3_INT_EXTI_IRQ);
}


int i=0,j=0;
int dir_val=0;
int en_val=0;
extern uint16_t pwm_index;	
void KEY1_IRQHandler(void)
{
  //ȷ���Ƿ������EXTI Line�ж�
	if(__HAL_GPIO_EXTI_GET_IT(KEY1_INT_GPIO_PIN) != RESET) 
	{
		// LED2 ȡ��		
		LED2_TOGGLE;
		// ���ݰ����ڶ�Ӧ360�������൱�� ÿ��������Ӧ 0.5��
		pwm_index=pwm_index+2*90;

    //����жϱ�־λ
		__HAL_GPIO_EXTI_CLEAR_IT(KEY1_INT_GPIO_PIN);     
	}  
}

void KEY3_IRQHandler(void)
{
  //ȷ���Ƿ������EXTI Line�ж�
	if(__HAL_GPIO_EXTI_GET_IT(KEY3_INT_GPIO_PIN) != RESET) 
	{
		// LED1 ȡ��		
		LED3_TOGGLE;

		
    //����жϱ�־λ
		__HAL_GPIO_EXTI_CLEAR_IT(KEY3_INT_GPIO_PIN);     
	}  
}

/*********************************************END OF FILE**********************/
