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

/* �����ת���� */
__IO int8_t motor_direction = 0;
/* ��ǰʱ���ܼ���ֵ */
__IO int32_t capture_count = 0;
/* ��һʱ���ܼ���ֵ */
__IO int32_t last_count = 0;
/* ��λʱ�����ܼ���ֵ */
__IO int32_t count_per_unit = 0;
/* ���ת��ת�� */
__IO float shaft_speed = 0.0f;
/* �ۻ�Ȧ�� */
__IO float number_of_rotations = 0.0f;
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void) 
{
  int i = 0;
	/* ��ʼ��ϵͳʱ��Ϊ480MHz */
	SystemClock_Config();
	/*��ʼ��USART ����ģʽΪ 115200 8-N-1���жϽ���*/
	DEBUG_USART_Config();
	LED_GPIO_Config();
	printf("��ӭʹ��Ұ�� ��������� ������� ���������� ����\r\n");
	printf("���°���1�������������2ֹͣ������3�ı䷽��\r\n");	
  /* ��ʼ��ʱ��� */
  HAL_InitTick(5);
	/*������ʼ��*/
	Key_GPIO_Config();	
	/*led��ʼ��*/
	LED_GPIO_Config();
	/*���������ʼ��*/
	stepper_Init();
  /* �ϵ�Ĭ��ֹͣ���������1���� */
  MOTOR_EN(OFF);
  /* �������ӿڳ�ʼ�� */
	Encoder_Init();
	
	while(1)
	{
    /* ɨ��KEY1��������� */
    if(Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON)
    {
      MOTOR_EN(ON);
    }
    /* ɨ��KEY2��ֹͣ��� */
    if(Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON)
    {
      MOTOR_EN(OFF);
    }
    /* ɨ��KEY3���ı䷽�� */
    if(Key_Scan(KEY3_GPIO_PORT,KEY3_PIN) == KEY_ON)
    {
      static int j = 0;
      j > 0 ? MOTOR_DIR(CCW) : MOTOR_DIR(CW);
      j=!j;
    }
    
    /* 20ms����һ�� */
    /* �����ת���� = �������������� */
    motor_direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&TIM_EncoderHandle);
    
    /* ��ǰʱ���ܼ���ֵ = ������ֵ + ����������� * ENCODER_TIM_PERIOD  */
    capture_count =__HAL_TIM_GET_COUNTER(&TIM_EncoderHandle) + (Encoder_Overflow_Count * ENCODER_TIM_PERIOD);
    
    /* ��λʱ�����ܼ���ֵ = ��ǰʱ���ܼ���ֵ - ��һʱ���ܼ���ֵ */
    count_per_unit = capture_count - last_count;
    
    /* ת��ת�� = ��λʱ���ڵļ���ֵ / �������ֱܷ��� * ʱ��ϵ��  */
    shaft_speed = (float)count_per_unit / ENCODER_TOTAL_RESOLUTION * 50 ;
    
    /* �ۻ�Ȧ�� = ��ǰʱ���ܼ���ֵ / �������ֱܷ���  */
    number_of_rotations = (float)capture_count / ENCODER_TOTAL_RESOLUTION;

    /* ��¼��ǰ�ܼ���ֵ������һʱ�̼���ʹ�� */
    last_count = capture_count;
    
    if(i == 50)/* 1s����һ�� */
    {
      printf("\r\n�������%d\r\n", motor_direction);
      printf("��λʱ������Ч����ֵ��%d\r\n", (count_per_unit<0 ? abs(count_per_unit) : count_per_unit));
      printf("�������ת�٣�%.2f ת/��\r\n", shaft_speed);
      printf("�ۼ�Ȧ����%.2f Ȧ\r\n", number_of_rotations);
      i = 0;
    }
    HAL_Delay(20);
    i++;
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
