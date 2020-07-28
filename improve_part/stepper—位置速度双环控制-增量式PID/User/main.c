/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   �������-GPIO�ж�ģ���������
  ******************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32H743 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************
  */  
#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx.h"
#include "main.h"
#include "./led/bsp_led.h"
#include "./delay/core_delay.h" 
#include "./usart/bsp_debug_usart.h"
#include "./stepper/bsp_stepper_init.h"
#include "./key/bsp_key.h" 
#include "./Encoder/bsp_encoder.h"
#include "./pid/bsp_pid.h"
#include "./tim/bsp_basic_tim.h"
#include "./stepper/bsp_stepper_ctrl.h"

extern _pid move_pid,speed_pid;
extern int pid_status;
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void) 
{
	/* ��ʼ��ϵͳʱ��Ϊ480MHz */
	SystemClock_Config();
	/*��ʼ��USART ����ģʽΪ 115200 8-N-1���жϽ���*/
	DEBUG_USART_Config();
	LED_GPIO_Config();
	printf("��ӭʹ��Ұ�� ��������� ������� �ٶȱջ����� λ��ʽPID����\r\n");
	printf("���°���1�������������2ֹͣ������3����Ŀ��ֵ������4����Ŀ��ֵ\r\n");	
  /* ��ʼ��ʱ��� */
  HAL_InitTick(5);
	/*������ʼ��*/
	Key_GPIO_Config();	
	/*led��ʼ��*/
	LED_GPIO_Config();
  /* ��ʼ��������ʱ����ʱ��20ms����һ���ж� */
	TIMx_Configuration();
  /* �������ӿڳ�ʼ�� */
	Encoder_Init();
	/*���������ʼ��*/
	stepper_Init();
  /* �ϵ�Ĭ��ֹͣ��� */
  Set_Stepper_Stop();
  /* PID�㷨������ʼ�� */
  PID_param_init();
  
  /* Ŀ���ٶ�ת��Ϊ����������������ΪpidĿ��ֵ */
  move_pid.target_val = TARGET_DISP * ENCODER_TOTAL_RESOLUTION;
	int32_t Temp = TARGET_DISP * ENCODER_TOTAL_RESOLUTION;
#if PID_ASSISTANT_EN
  set_computer_value(SEED_STOP_CMD, CURVES_CH1, NULL, 0);    // ͬ����λ����������ť״̬
  set_computer_value(SEED_TARGET_CMD, CURVES_CH1, &Temp, 1);// ��ͨ�� 1 ����Ŀ��ֵ
#endif

	while(1)
	{
    /* ɨ��KEY1��������� */
    if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON  )
		{
    #if PID_ASSISTANT_EN
      Set_Stepper_Start();
      set_computer_value(SEED_START_CMD, CURVES_CH1, NULL, 0);// ͬ����λ����������ť״̬
    #else
      Set_Stepper_Start();
    #endif
		}
    /* ɨ��KEY2��ֹͣ��� */
    if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON  )
		{
    #if PID_ASSISTANT_EN
      Set_Stepper_Stop();
      set_computer_value(SEED_STOP_CMD, CURVES_CH1, NULL, 0);// ͬ����λ����������ť״̬
    #else
      Set_Stepper_Stop();     
    #endif
		}
    /* ɨ��KEY3������Ŀ��λ��*/
    if( Key_Scan(KEY3_GPIO_PORT,KEY3_PIN) == KEY_ON  )
		{
      /* Ŀ��λ������48000����Ӧ���λ������20Ȧ */
      move_pid.target_val += 48000;
      
    #if PID_ASSISTANT_EN
      int temp = move_pid.target_val;
      set_computer_value(SEED_TARGET_CMD, CURVES_CH1, &temp, 1);// ��ͨ�� 1 ����Ŀ��ֵ
    #endif
		}
    /* ɨ��KEY4����СĿ��λ�� */
    if( Key_Scan(KEY4_GPIO_PORT,KEY4_PIN) == KEY_ON  )
		{
      /* Ŀ��λ�ü�С48000����Ӧ���λ�ü���20Ȧ */
      move_pid.target_val -= 48000;
      
    #if PID_ASSISTANT_EN
      int temp = move_pid.target_val;
      set_computer_value(SEED_TARGET_CMD, CURVES_CH1, &temp, 1);// ��ͨ�� 1 ����Ŀ��ֵ
    #endif
		}
	}
} 	

/**
  * @brief  ��ʱ�������¼��ص�����
  * @param  ��
  * @retval ��
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* �жϴ����жϵĶ�ʱ�� */
  if(htim->Instance == BASIC_TIM)
  {
    Stepper_Ctrl();
  }
  else if(htim->Instance == ENCODER_TIM)
  {  
    /* �жϵ�ǰ�������� */
    if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
      /* ���� */
      encoder_overflow_count--;
    else
      /* ���� */
      encoder_overflow_count++;
  }
}


/**
  * @brief  System Clock ����
  *         system Clock ��������: 
	*            System Clock source  = PLL (HSE)
	*            SYSCLK(Hz)           = 480000000 (CPU Clock)
	*            HCLK(Hz)             = 240000000 (AXI and AHBs Clock)
	*            AHB Prescaler        = 2
	*            D1 APB3 Prescaler    = 2 (APB3 Clock  120MHz)
	*            D2 APB1 Prescaler    = 2 (APB1 Clock  120MHz)
	*            D2 APB2 Prescaler    = 2 (APB2 Clock  120MHz)
	*            D3 APB4 Prescaler    = 2 (APB4 Clock  120MHz)
	*            HSE Frequency(Hz)    = 25000000
	*            PLL_M                = 5
	*            PLL_N                = 192
	*            PLL_P                = 2
	*            PLL_Q                = 4
	*            PLL_R                = 2
	*            VDD(V)               = 3.3
	*            Flash Latency(WS)    = 4
  * @param  None
  * @retval None
  */
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** ���õ�Դ���ø���
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** ����������ѹ�������ѹ
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** ��ʼ��CPU��AHB��APB����ʱ��
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
		while(1);
  }
  /** ��ʼ��CPU��AHB��APB����ʱ��
  */
	/* ѡ��PLL��Ϊϵͳʱ��Դ����������ʱ�ӷ�Ƶ�� */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK  | \
																 RCC_CLOCKTYPE_HCLK    | \
																 RCC_CLOCKTYPE_D1PCLK1 | \
																 RCC_CLOCKTYPE_PCLK1   | \
                                 RCC_CLOCKTYPE_PCLK2   | \
																 RCC_CLOCKTYPE_D3PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;  
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2; 
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2; 
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2; 
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4)!= HAL_OK)
  {
    while(1) { ; }
  }
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
		while(1);
  }
}
/****************************END OF FILE***************************/
