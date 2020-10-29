/**
  ******************************************************************************
  * @file    bsp_bldcm_control.c
  * @author  fire
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   ��ˢ���-��������-˫���
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 H743 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 

#include ".\bldcm_control\bsp_bldcm_control.h"

/* ˽�б��� */
static bldcm_data_t motor1_bldcm_data;
static bldcm_data_t motor2_bldcm_data;

/* �ֲ����� */
static void sd_gpio_config(void);

/**
  * @brief  �����ʼ��
  * @param  ��
  * @retval ��
  */
void bldcm_init(void)
{
  TIMx_Configuration();    // ������ƶ�ʱ�������ų�ʼ��
  hall_motor1_tim_config();       // ������������ʼ��
	hall_motor2_tim_config();       // ������������ʼ��
  sd_gpio_config();        // sd ���ų�ʼ��
}

/**
  * @brief  ��� SD �������ų�ʼ��
  * @param  ��
  * @retval ��
  */
static void sd_gpio_config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* ��ʱ��ͨ���������Ŷ˿�ʱ��ʹ�� */
	MOTOR1_SHUTDOWN_GPIO_CLK_ENABLE();
 	MOTOR2_SHUTDOWN_GPIO_CLK_ENABLE(); 
	
  /* ����IO��ʼ�� */
	/*�����������*/
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	/*������������ */ 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*ѡ��Ҫ���Ƶ�GPIO����*/	
	GPIO_InitStruct.Pin = MOTOR1_SHUTDOWN_PIN;
  
	/*���ÿ⺯����ʹ���������õ�GPIO_InitStructure��ʼ��GPIO*/
  HAL_GPIO_Init(MOTOR1_SHUTDOWN_GPIO_PORT, &GPIO_InitStruct);
	/*ѡ��Ҫ���Ƶ�GPIO����*/	
	GPIO_InitStruct.Pin = MOTOR2_SHUTDOWN_PIN;
	HAL_GPIO_Init(MOTOR2_SHUTDOWN_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  ���õ���ٶ�
  * @param  v: �ٶȣ�ռ�ձȣ�
  * @retval ��
  */
void set_motor1_bldcm_speed(uint16_t v)
{
  motor1_bldcm_data.dutyfactor = v;
  
  set_motor1_pwm_pulse(v);     // �����ٶ�
}

void set_motor2_bldcm_speed(uint16_t v)
{
  motor2_bldcm_data.dutyfactor = v;
  
  set_motor2_pwm_pulse(v);     // �����ٶ�
}

/**
  * @brief  ���õ������
  * @param  ��
  * @retval ��
  */
void set_motor1_bldcm_direction(motor_dir_t dir)
{
  motor1_bldcm_data.direction = dir;
}

void set_motor2_bldcm_direction(motor_dir_t dir)
{
  motor2_bldcm_data.direction = dir;
}

/**
  * @brief  ��ȡ�����ǰ����
  * @param  ��
  * @retval ��
  */
motor_dir_t get_motor1_bldcm_direction(void)
{
  return motor1_bldcm_data.direction;
}

motor_dir_t get_motor2_bldcm_direction(void)
{
  return motor2_bldcm_data.direction;
}

/**
  * @brief  ʹ�ܵ��
  * @param  ��
  * @retval ��
  */
void set_motor1_bldcm_enable(void)
{
  MOTOR1_BLDCM_ENABLE_SD();
  hall_motor1_enable();
}

void set_motor2_bldcm_enable(void)
{
  MOTOR2_BLDCM_ENABLE_SD();
  hall_motor2_enable();
}

/**
  * @brief  ���õ��
  * @param  ��
  * @retval ��
  */
void set_motor1_bldcm_disable(void)
{
  /* ���û����������ӿ� */
  hall_motor1_disable();
  
  /* ֹͣ PWM ��� */
  stop_motor1_pwm_output();
  
  /* �ر� MOS �� */
  MOTOR1_BLDCM_DISABLE_SD();
}

void set_motor2_bldcm_disable(void)
{
  /* ���û����������ӿ� */
  hall_motor2_disable();
  
  /* ֹͣ PWM ��� */
  stop_motor2_pwm_output();
  
  /* �ر� MOS �� */
  MOTOR2_BLDCM_DISABLE_SD();
}

/*********************************************END OF FILE**********************/
