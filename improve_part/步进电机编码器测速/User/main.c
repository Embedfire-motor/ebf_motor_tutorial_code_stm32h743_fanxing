/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   步进电机-GPIO中断模拟脉冲控制
  ******************************************************************
  * @attention
  *
  * 实验平台:野火 STM32H743 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
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

/* 电机旋转方向 */
__IO int8_t motor_direction = 0;
/* 当前时刻总计数值 */
__IO int32_t capture_count = 0;
/* 上一时刻总计数值 */
__IO int32_t last_count = 0;
/* 单位时间内总计数值 */
__IO int32_t count_per_unit = 0;
/* 电机转轴转速 */
__IO float shaft_speed = 0.0f;
/* 累积圈数 */
__IO float number_of_rotations = 0.0f;
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void) 
{
  int i = 0;
	/* 初始化系统时钟为480MHz */
	SystemClock_Config();
	/*初始化USART 配置模式为 115200 8-N-1，中断接收*/
	DEBUG_USART_Config();
	LED_GPIO_Config();
	printf("欢迎使用野火 电机开发板 步进电机 编码器测速 例程\r\n");
	printf("按下按键1启动电机、按键2停止、按键3改变方向\r\n");	
  /* 初始化时间戳 */
  HAL_InitTick(5);
	/*按键初始化*/
	Key_GPIO_Config();	
	/*led初始化*/
	LED_GPIO_Config();
	/*步进电机初始化*/
	stepper_Init();
  /* 上电默认停止电机，按键1启动 */
  MOTOR_EN(OFF);
  /* 编码器接口初始化 */
	Encoder_Init();
	
	while(1)
	{
    /* 扫描KEY1，启动电机 */
    if(Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON)
    {
      MOTOR_EN(ON);
    }
    /* 扫描KEY2，停止电机 */
    if(Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON)
    {
      MOTOR_EN(OFF);
    }
    /* 扫描KEY3，改变方向 */
    if(Key_Scan(KEY3_GPIO_PORT,KEY3_PIN) == KEY_ON)
    {
      static int j = 0;
      j > 0 ? MOTOR_DIR(CCW) : MOTOR_DIR(CW);
      j=!j;
    }
    
    /* 20ms计算一次 */
    /* 电机旋转方向 = 计数器计数方向 */
    motor_direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&TIM_EncoderHandle);
    
    /* 当前时刻总计数值 = 计数器值 + 计数溢出次数 * ENCODER_TIM_PERIOD  */
    capture_count =__HAL_TIM_GET_COUNTER(&TIM_EncoderHandle) + (Encoder_Overflow_Count * ENCODER_TIM_PERIOD);
    
    /* 单位时间内总计数值 = 当前时刻总计数值 - 上一时刻总计数值 */
    count_per_unit = capture_count - last_count;
    
    /* 转轴转速 = 单位时间内的计数值 / 编码器总分辨率 * 时间系数  */
    shaft_speed = (float)count_per_unit / ENCODER_TOTAL_RESOLUTION * 50 ;
    
    /* 累积圈数 = 当前时刻总计数值 / 编码器总分辨率  */
    number_of_rotations = (float)capture_count / ENCODER_TOTAL_RESOLUTION;

    /* 记录当前总计数值，供下一时刻计算使用 */
    last_count = capture_count;
    
    if(i == 50)/* 1s报告一次 */
    {
      printf("\r\n电机方向：%d\r\n", motor_direction);
      printf("单位时间内有效计数值：%d\r\n", (count_per_unit<0 ? abs(count_per_unit) : count_per_unit));
      printf("步进电机转速：%.2f 转/秒\r\n", shaft_speed);
      printf("累计圈数：%.2f 圈\r\n", number_of_rotations);
      i = 0;
    }
    HAL_Delay(20);
    i++;
	}
} 	


/**
  * @brief  System Clock 配置
  *         system Clock 配置如下: 
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

  /** 启用电源配置更新
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** 配置主内稳压器输出电压
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** 初始化CPU、AHB和APB总线时钟
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
  /** 初始化CPU、AHB和APB总线时钟
  */
	/* 选择PLL作为系统时钟源并配置总线时钟分频器 */
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
