/**
  ******************************************************************************
  * @file    bsp_motor_control.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   ������ƽӿ�
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 F407 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 

#include ".\motor_control\bsp_motor_control.h"

/* ˽�б��� */
static motor_dir_t direction  = MOTOR_FWD;     // ��¼����
static uint16_t    dutyfactor = 0;             // ��¼ռ�ձ�

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
}

/**
  * @brief  �����ʼ��
  * @param  ��
  * @retval ��
  */
void motor_init(void)
{
  TIMx_Configuration();     // ��ʼ����� 1
  sd_gpio_config();
}

/**
  * @brief  ���õ���ٶ�
  * @param  v: �ٶȣ�ռ�ձȣ�
  * @retval ��
  */
void set_motor_speed(uint16_t v)
{
  dutyfactor = v;
  
  if (direction == MOTOR_FWD)
  {
    SET_FWD_COMPAER(dutyfactor);     // �����ٶ�
  }
  else
  {
    SET_REV_COMPAER(dutyfactor);     // �����ٶ�
  }
}

/**
  * @brief  ���õ������
  * @param  ��
  * @retval ��
  */
void set_motor_direction(motor_dir_t dir)
{
  direction = dir;
  
  if (direction == MOTOR_FWD)
  {
    SET_FWD_COMPAER(dutyfactor);     // �����ٶ�
    SET_REV_COMPAER(0);              // �����ٶ�
  }
  else
  {
    SET_FWD_COMPAER(0);              // �����ٶ�
    SET_REV_COMPAER(dutyfactor);     // �����ٶ�
  }
}

/**
  * @brief  ʹ�ܵ��
  * @param  ��
  * @retval ��
  */
void set_motor_enable(void)
{
  MOTOR_ENABLE_SD();
  MOTOR_FWD_ENABLE();
  MOTOR_REV_ENABLE();
}

/**
  * @brief  ���õ��
  * @param  ��
  * @retval ��
  */
void set_motor_disable(void)
{
  MOTOR_DISABLE_SD();
  MOTOR_FWD_DISABLE();
  MOTOR_REV_DISABLE();
}

/*********************************************END OF FILE**********************/

