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
#include "./pid/bsp_pid.h"
#include "./tim/bsp_basic_tim.h"
#include "./stepper/bsp_stepper_ctrl.h"

extern _pid move_pid,speed_pid;
extern int pid_status;
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void) 
{
	/* 初始化系统时钟为480MHz */
	SystemClock_Config();
	/*初始化USART 配置模式为 115200 8-N-1，中断接收*/
	DEBUG_USART_Config();
	LED_GPIO_Config();
	printf("欢迎使用野火 电机开发板 步进电机 速度闭环控制 位置式PID例程\r\n");
	printf("按下按键1启动电机、按键2停止、按键3增加目标值、按键4减少目标值\r\n");	
  /* 初始化时间戳 */
  HAL_InitTick(5);
	/*按键初始化*/
	Key_GPIO_Config();	
	/*led初始化*/
	LED_GPIO_Config();
  /* 初始化基本定时器定时，20ms产生一次中断 */
	TIMx_Configuration();
  /* 编码器接口初始化 */
	Encoder_Init();
	/*步进电机初始化*/
	stepper_Init();
  /* 上电默认停止电机 */
  Set_Stepper_Stop();
  /* PID算法参数初始化 */
  PID_param_init();
  
  /* 目标速度转换为编码器的脉冲数作为pid目标值 */
  move_pid.target_val = TARGET_DISP * ENCODER_TOTAL_RESOLUTION;
	int32_t Temp = TARGET_DISP * ENCODER_TOTAL_RESOLUTION;
#if PID_ASSISTANT_EN
  set_computer_value(SEED_STOP_CMD, CURVES_CH1, NULL, 0);    // 同步上位机的启动按钮状态
  set_computer_value(SEED_TARGET_CMD, CURVES_CH1, &Temp, 1);// 给通道 1 发送目标值
#endif

	while(1)
	{
    /* 扫描KEY1，启动电机 */
    if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON  )
		{
    #if PID_ASSISTANT_EN
      Set_Stepper_Start();
      set_computer_value(SEED_START_CMD, CURVES_CH1, NULL, 0);// 同步上位机的启动按钮状态
    #else
      Set_Stepper_Start();
    #endif
		}
    /* 扫描KEY2，停止电机 */
    if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON  )
		{
    #if PID_ASSISTANT_EN
      Set_Stepper_Stop();
      set_computer_value(SEED_STOP_CMD, CURVES_CH1, NULL, 0);// 同步上位机的启动按钮状态
    #else
      Set_Stepper_Stop();     
    #endif
		}
    /* 扫描KEY3，增大目标位置*/
    if( Key_Scan(KEY3_GPIO_PORT,KEY3_PIN) == KEY_ON  )
		{
      /* 目标位置增加48000，对应电机位置增加20圈 */
      move_pid.target_val += 48000;
      
    #if PID_ASSISTANT_EN
      int temp = move_pid.target_val;
      set_computer_value(SEED_TARGET_CMD, CURVES_CH1, &temp, 1);// 给通道 1 发送目标值
    #endif
		}
    /* 扫描KEY4，减小目标位置 */
    if( Key_Scan(KEY4_GPIO_PORT,KEY4_PIN) == KEY_ON  )
		{
      /* 目标位置减小48000，对应电机位置减少20圈 */
      move_pid.target_val -= 48000;
      
    #if PID_ASSISTANT_EN
      int temp = move_pid.target_val;
      set_computer_value(SEED_TARGET_CMD, CURVES_CH1, &temp, 1);// 给通道 1 发送目标值
    #endif
		}
	}
} 	

/**
  * @brief  定时器更新事件回调函数
  * @param  无
  * @retval 无
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* 判断触发中断的定时器 */
  if(htim->Instance == BASIC_TIM)
  {
    Stepper_Ctrl();
  }
  else if(htim->Instance == ENCODER_TIM)
  {  
    /* 判断当前计数方向 */
    if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
      /* 下溢 */
      encoder_overflow_count--;
    else
      /* 上溢 */
      encoder_overflow_count++;
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
