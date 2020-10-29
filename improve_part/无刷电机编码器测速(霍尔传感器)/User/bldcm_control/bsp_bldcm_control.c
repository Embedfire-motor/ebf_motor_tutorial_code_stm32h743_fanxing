/**
  ******************************************************************************
  * @file    bsp_bldcm_control.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   ��ˢ������ƽӿ�
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 H743 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "./bldcm_control/bsp_bldcm_control.h"
#include "./usart/bsp_debug_usart.h"
#include <math.h>
#include <stdlib.h>
#include "./tim/bsp_basic_tim.h"

/* ˽�б��� */
static bldcm_data_t bldcm_data;

/* �ֲ����� */
static void sd_gpio_config(void);

/**
  * @brief  �����ʼ��
  * @param  ��
  * @retval ��
  */
void bldcm_init(void)
{
  PWM_TIMx_Configuration();    // ������ƶ�ʱ�������ų�ʼ��
  hall_tim_config();       // ������������ʼ��
  sd_gpio_config();
}

static void sd_gpio_config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* ��ʱ��ͨ���������Ŷ˿�ʱ��ʹ�� */
	SHUTDOWN_GPIO_CLK_ENABLE();
  
  /* ����IO��ʼ�� */
	/*�����������*/
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	/*������������ */ 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*ѡ��Ҫ���Ƶ�GPIO����*/	
	GPIO_InitStruct.Pin = SHUTDOWN_PIN;
  
	/*���ÿ⺯����ʹ���������õ�GPIO_InitStructure��ʼ��GPIO*/
  HAL_GPIO_Init(SHUTDOWN_GPIO_PORT, &GPIO_InitStruct);
	/* �ӳ�50ms��H743��Cache���ٶȻ���죬δ����SD����ʱ�򡣼������ϵͳʱ�����ע�������ʱ�����Ƿ���Ч���������� */
	HAL_Delay(50);
  BLDCM_ENABLE_SD();     // Ĭ�Ͽ���
}

/**
  * @brief  ���õ���ٶ�
  * @param  v: �ٶȣ�ռ�ձȣ�
  * @retval ��
  */
void set_bldcm_speed(uint16_t v)
{
  bldcm_data.dutyfactor = v;
  
  set_pwm_pulse(v);     // �����ٶ�
}

/**
  * @brief  ���õ������
  * @param  ��
  * @retval ��
  */
void set_bldcm_direction(motor_dir_t dir)
{
  bldcm_data.direction = dir;
}

/**
  * @brief  ��ȡ�����ǰ����
  * @param  ��
  * @retval ��
  */
motor_dir_t get_bldcm_direction(void)
{
  return bldcm_data.direction;
}

/**
  * @brief  ʹ�ܵ��
  * @param  ��
  * @retval ��
  */
void set_bldcm_enable(void)
{
  bldcm_data.is_enable = 1;
  hall_enable();
}

/**
  * @brief  ���õ��
  * @param  ��
  * @retval ��
  */
void set_bldcm_disable(void)
{
  /* ���û����������ӿ� */
  hall_disable();
  
  /* ֹͣ PWM ��� */
  stop_pwm_output();
  
  bldcm_data.is_enable = 0;
}
 
///**
//  * @brief  ��ʱ��ÿ100ms����һ���жϻص�����
//  * @param  htim����ʱ�����
//  * @retval ��
//  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if(htim==(&TIM_TimeBaseStructure))
//    {
//        bldcm_pid_control();
//    }
//}

/*********************************************END OF FILE**********************/
