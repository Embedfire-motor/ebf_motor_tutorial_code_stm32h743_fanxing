/**
  ******************************************************************************
  * @file    bsp_motor_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   �����ض�ʱ������
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 H743 ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./tim/bsp_motor_tim.h"
#include "./led/bsp_led.h"
#include "./usart/bsp_debug_usart.h"

TIM_HandleTypeDef  htimx_bldcm;
TIM_OC_InitTypeDef TIM_OCInitStructure;

/* ������������ض�ʱ����ʼ�� */
TIM_HandleTypeDef htimx_hall;

/**
  * @brief  ����TIM�������PWMʱ�õ���I/O
  * @param  ��
  * @retval ��
  */
static void TIMx_GPIO_Config(void) 
{
  /*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
  GPIO_InitTypeDef GPIO_InitStructure;

  /*������ʱ����ص�GPIO����ʱ��*/
  MOTOR_OCPWM1_GPIO_CLK_ENABLE();
  MOTOR_OCNPWM1_GPIO_CLK_ENABLE();
  MOTOR_OCPWM2_GPIO_CLK_ENABLE();
  MOTOR_OCNPWM2_GPIO_CLK_ENABLE();
  MOTOR_OCPWM3_GPIO_CLK_ENABLE();
  MOTOR_OCNPWM3_GPIO_CLK_ENABLE();

  /* ��ʱ���������ų�ʼ�� */															   
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;   // �������ģʽ

  GPIO_InitStructure.Pin = MOTOR_OCNPWM1_PIN;
  HAL_GPIO_Init(MOTOR_OCNPWM1_GPIO_PORT, &GPIO_InitStructure);	

  GPIO_InitStructure.Pin = MOTOR_OCNPWM2_PIN;	
  HAL_GPIO_Init(MOTOR_OCNPWM2_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = MOTOR_OCNPWM3_PIN;	
  HAL_GPIO_Init(MOTOR_OCNPWM3_GPIO_PORT, &GPIO_InitStructure);	

  /* ͨ�� 2 */
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;  

  GPIO_InitStructure.Pin = MOTOR_OCPWM1_PIN;
  GPIO_InitStructure.Alternate = MOTOR_OCPWM1_AF;	
  HAL_GPIO_Init(MOTOR_OCPWM1_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = MOTOR_OCPWM2_PIN;	
  GPIO_InitStructure.Alternate = MOTOR_OCPWM2_AF;	
  HAL_GPIO_Init(MOTOR_OCPWM2_GPIO_PORT, &GPIO_InitStructure);

  /* ͨ�� 3 */
  GPIO_InitStructure.Pin = MOTOR_OCPWM3_PIN;	
  GPIO_InitStructure.Alternate = MOTOR_OCPWM3_AF;	
  HAL_GPIO_Init(MOTOR_OCPWM3_GPIO_PORT, &GPIO_InitStructure);
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
static void TIM_Mode_Config(void)
{
  // ����TIMx_CLK,x[1,8] 
  MOTOR_TIM_CLK_ENABLE(); 
  /* ���嶨ʱ���ľ����ȷ����ʱ���Ĵ����Ļ���ַ*/
  htimx_bldcm.Instance = MOTOR_TIM;
  /* �ۼ� TIM_Period�������һ�����»����ж�*/		
  //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
  htimx_bldcm.Init.Period =  PWM_PERIOD_COUNT - 1;
  // �߼����ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK=216MHz 
  // �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=1MHz
  htimx_bldcm.Init.Prescaler = PWM_PRESCALER_COUNT - 1;	
  // ����ʱ�ӷ�Ƶ
  htimx_bldcm.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  // ������ʽ
  htimx_bldcm.Init.CounterMode=TIM_COUNTERMODE_UP;
  // �ظ�������
  htimx_bldcm.Init.RepetitionCounter=0;	
  // ��ʼ����ʱ��TIMx, x[1,8]
  HAL_TIM_PWM_Init(&htimx_bldcm);

  /*PWMģʽ����*/
  //����ΪPWMģʽ1
  TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
  TIM_OCInitStructure.Pulse = 0;
  TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
  TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
  TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  HAL_TIM_PWM_ConfigChannel(&htimx_bldcm,&TIM_OCInitStructure,TIM_CHANNEL_1);    // ��ʼ��ͨ�� 1 ��� PWM 
  HAL_TIM_PWM_ConfigChannel(&htimx_bldcm,&TIM_OCInitStructure,TIM_CHANNEL_2);    // ��ʼ��ͨ�� 2 ��� PWM
  HAL_TIM_PWM_ConfigChannel(&htimx_bldcm,&TIM_OCInitStructure,TIM_CHANNEL_3);    // ��ʼ��ͨ�� 3 ��� PWM

  set_pwm_pulse(0);
}

/**
  * @brief  ��ʼpwm���
  * @param  ��
  * @retval ��
  */
void start_pwm_output(void)
{
  /* ������ʱ��ͨ��1���PWM */
  HAL_TIM_PWM_Start(&htimx_bldcm,TIM_CHANNEL_1);

  /* ������ʱ��ͨ��2���PWM */
  HAL_TIM_PWM_Start(&htimx_bldcm,TIM_CHANNEL_2);

  /* ������ʱ��ͨ��3���PWM */
  HAL_TIM_PWM_Start(&htimx_bldcm,TIM_CHANNEL_3);
  
}

/**
  * @brief  ֹͣpwm���
  * @param  ��
  * @retval ��
  */
void stop_pwm_output(void)
{
  /* �رն�ʱ��ͨ��1���PWM */
	HAL_TIM_PWM_Stop(&htimx_bldcm,TIM_CHANNEL_1);

  /* �رն�ʱ��ͨ��2���PWM */
	HAL_TIM_PWM_Stop(&htimx_bldcm,TIM_CHANNEL_2);
  
  /* �رն�ʱ��ͨ��3���PWM */
	HAL_TIM_PWM_Stop(&htimx_bldcm,TIM_CHANNEL_3);
  
  HAL_GPIO_WritePin(MOTOR_OCNPWM1_GPIO_PORT, MOTOR_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�
  HAL_GPIO_WritePin(MOTOR_OCNPWM2_GPIO_PORT, MOTOR_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
  HAL_GPIO_WritePin(MOTOR_OCNPWM3_GPIO_PORT, MOTOR_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
}

/**
  * @brief  ����pwm�����ռ�ձ�
  * @param  pulse:Ҫ���õ�ռ�ձ�
  * @retval ��
  */
void set_pwm_pulse(uint16_t pulse)
{
  /* ���ö�ʱ��ͨ����� PWM ��ռ�ձ� */
	__HAL_TIM_SET_COMPARE(&htimx_bldcm,TIM_CHANNEL_1,pulse);
  __HAL_TIM_SET_COMPARE(&htimx_bldcm,TIM_CHANNEL_2,pulse);
  __HAL_TIM_SET_COMPARE(&htimx_bldcm,TIM_CHANNEL_3,pulse);
}

/**
  * @brief  ��ʼ���߼����ƶ�ʱ��
  * @param  ��
  * @retval ��
  */
void TIMx_Configuration(void)
{
	TIMx_GPIO_Config();
	TIM_Mode_Config();
}

/**
  * @brief  �������������ų�ʼ��
  * @param  ��
  * @retval ��
  */
static void hall_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  HALL_INPUTU_GPIO_CLK_ENABLE();
  HALL_INPUTV_GPIO_CLK_ENABLE();
  HALL_INPUTW_GPIO_CLK_ENABLE();
  
  /* ��ʱ��ͨ�� 1 ���ų�ʼ�� */
  GPIO_InitStruct.Pin = HALL_INPUTU_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = HALL_INPUTU_AF;
  HAL_GPIO_Init(HALL_INPUTU_GPIO_PORT, &GPIO_InitStruct);
  
  /* ��ʱ��ͨ�� 2 ���ų�ʼ�� */
  GPIO_InitStruct.Pin = HALL_INPUTV_PIN;
  HAL_GPIO_Init(HALL_INPUTV_GPIO_PORT, &GPIO_InitStruct);
  
  /* ��ʱ��ͨ�� 3 ���ų�ʼ�� */
  GPIO_InitStruct.Pin = HALL_INPUTW_PIN;
  HAL_GPIO_Init(HALL_INPUTW_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  ������������ʱ����ʼ��
  * @param  ��
  * @retval ��
  */
static void hall_tim_init(void)
{
  TIM_HallSensor_InitTypeDef  hall_sensor_onfig;  
  
  /* ������ʱ������ʱ��ʹ�� */
  HALL_TIM_CLK_ENABLE();
  
  /* ��ʱ�������������� */
  htimx_hall.Instance = HALL_TIM;
  htimx_hall.Init.Prescaler = HALL_PRESCALER_COUNT - 1;       // Ԥ��Ƶ
  htimx_hall.Init.CounterMode = TIM_COUNTERMODE_UP;           // ���ϼ���
  htimx_hall.Init.Period = HALL_PERIOD_COUNT - 1;             // ��������
  htimx_hall.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;     // ʱ�ӷ�Ƶ
  
  hall_sensor_onfig.IC1Prescaler = TIM_ICPSC_DIV1;            // ���벶���Ƶ
  hall_sensor_onfig.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;    // ���벶����
  hall_sensor_onfig.IC1Filter = 10;                           // �����˲�
  hall_sensor_onfig.Commutation_Delay = 0U;                   // ��ʹ���ӳٴ���
  HAL_TIMEx_HallSensor_Init(&htimx_hall,&hall_sensor_onfig);
  
  HAL_NVIC_SetPriority(HALL_TIM_IRQn, 0, 0);    // �����ж����ȼ�
  HAL_NVIC_EnableIRQ(HALL_TIM_IRQn);            // ʹ���ж�
}

/**
  * @brief  ʹ�ܻ���������
  * @param  ��
  * @retval ��
  */
void hall_enable(void)
{
  /* ʹ�ܻ����������ӿ� */
  __HAL_TIM_ENABLE_IT(&htimx_hall, TIM_IT_TRIGGER);
  __HAL_TIM_ENABLE_IT(&htimx_hall, TIM_IT_UPDATE);
  
  HAL_TIMEx_HallSensor_Start(&htimx_hall);

  LED1_OFF;
  
  HAL_TIM_TriggerCallback(&htimx_hall);   // ִ��һ�λ���
}

/**
  * @brief  ���û���������
  * @param  ��
  * @retval ��
  */
void hall_disable(void)
{
  /* ���û����������ӿ� */
  __HAL_TIM_DISABLE_IT(&htimx_hall, TIM_IT_TRIGGER);
  __HAL_TIM_DISABLE_IT(&htimx_hall, TIM_IT_UPDATE);
  HAL_TIMEx_HallSensor_Stop(&htimx_hall);
}

uint8_t get_hall_state(void)
{
  uint8_t state = 0;
  
#if 1
  /* ��ȡ���������� U ��״̬ */
  if(HAL_GPIO_ReadPin(HALL_INPUTU_GPIO_PORT, HALL_INPUTU_PIN) != GPIO_PIN_RESET)
  {
    state |= 0x01U << 0;
  }
  
  /* ��ȡ���������� V ��״̬ */
  if(HAL_GPIO_ReadPin(HALL_INPUTV_GPIO_PORT, HALL_INPUTV_PIN) != GPIO_PIN_RESET)
  {
    state |= 0x01U << 1;
  }
  
  /* ��ȡ���������� W ��״̬ */
  if(HAL_GPIO_ReadPin(HALL_INPUTW_GPIO_PORT, HALL_INPUTW_PIN) != GPIO_PIN_RESET)
  {
    state |= 0x01U << 2;
  }
#else
  state = (GPIOH->IDR >> 10) & 7;    // �� 3 ��������������״̬
#endif

  return state;    // ���ش�����״̬
}

/**
  * @brief  ��ʼ��������������ʱ��
  * @param  ��
  * @retval ��
  */
void hall_tim_config(void)
{
	hall_gpio_init();	    // ��ʼ������
	hall_tim_init();      // ��ʼ����ʱ��
}

int update = 0;     // ��ʱ�����¼���

/**
  * @brief  ���������������ص�����
  * @param  htim:��ʱ�����
  * @retval ��
  */
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim)
{
  /* ��ȡ��������������״̬,��Ϊ��������� */
  uint8_t step = 0;
  step = get_hall_state();

  if(get_bldcm_direction() != MOTOR_FWD)
  {
    step = 7 - step;        // ����˳���Ĺ��ɿ�֪�� CW = 7 - CCW;
  }

  switch(step)
  {
    case 1://W+ U-
      /*  Channe2 configuration  */ 
      __HAL_TIM_SET_COMPARE(&htimx_bldcm,TIM_CHANNEL_2,0);
//      HAL_TIM_PWM_Stop(&htimx_bldcm, TIM_CHANNEL_2);     // ֹͣ���ű� PWM ���
      HAL_GPIO_WritePin(MOTOR_OCNPWM2_GPIO_PORT, MOTOR_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
    
      /*  Channe3 configuration */
//      HAL_TIM_PWM_Start(&htimx_bldcm, TIM_CHANNEL_3);    // ��ʼ���ű� PWM ���
    __HAL_TIM_SET_COMPARE(&htimx_bldcm,TIM_CHANNEL_3,PWM_PERIOD_COUNT/10);
      HAL_GPIO_WritePin(MOTOR_OCNPWM1_GPIO_PORT, MOTOR_OCNPWM1_PIN, GPIO_PIN_SET);      // �������ű�
      break;
    
    case 2: //U+  V-
      /*  Channe3 configuration */ 
    __HAL_TIM_SET_COMPARE(&htimx_bldcm,TIM_CHANNEL_3,0);
//      HAL_TIM_PWM_Stop(&htimx_bldcm, TIM_CHANNEL_3);
      HAL_GPIO_WritePin(MOTOR_OCNPWM3_GPIO_PORT, MOTOR_OCNPWM3_PIN, GPIO_PIN_RESET);
    
      /*  Channel configuration  */
//      HAL_TIM_PWM_Start(&htimx_bldcm, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htimx_bldcm,TIM_CHANNEL_1,PWM_PERIOD_COUNT/10);
      HAL_GPIO_WritePin(MOTOR_OCNPWM2_GPIO_PORT, MOTOR_OCNPWM2_PIN, GPIO_PIN_SET);
      break;
    
    case 3:// W+ V-
      /*  Channel configuration */ 
    __HAL_TIM_SET_COMPARE(&htimx_bldcm,TIM_CHANNEL_1,0);
//      HAL_TIM_PWM_Stop(&htimx_bldcm, TIM_CHANNEL_1);
      HAL_GPIO_WritePin(MOTOR_OCNPWM1_GPIO_PORT, MOTOR_OCNPWM1_PIN, GPIO_PIN_RESET);
 
      /*  Channe3 configuration  */
//      HAL_TIM_PWM_Start(&htimx_bldcm, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htimx_bldcm,TIM_CHANNEL_3,PWM_PERIOD_COUNT/10);
      HAL_GPIO_WritePin(MOTOR_OCNPWM2_GPIO_PORT, MOTOR_OCNPWM2_PIN, GPIO_PIN_SET);
      break;
    
    case 4:// V+ W-
      /*  Channel configuration */ 
    __HAL_TIM_SET_COMPARE(&htimx_bldcm,TIM_CHANNEL_1,0);
//      HAL_TIM_PWM_Stop(&htimx_bldcm, TIM_CHANNEL_1);
      HAL_GPIO_WritePin(MOTOR_OCNPWM1_GPIO_PORT, MOTOR_OCNPWM1_PIN, GPIO_PIN_RESET);

      /*  Channe2 configuration */
//      HAL_TIM_PWM_Start(&htimx_bldcm, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htimx_bldcm,TIM_CHANNEL_2,PWM_PERIOD_COUNT/10);
      HAL_GPIO_WritePin(MOTOR_OCNPWM3_GPIO_PORT, MOTOR_OCNPWM3_PIN, GPIO_PIN_SET);    
      break;
    
    case 5: // V+ U-
      /*  Channe3 configuration */       
    __HAL_TIM_SET_COMPARE(&htimx_bldcm,TIM_CHANNEL_3,0);
//      HAL_TIM_PWM_Stop(&htimx_bldcm, TIM_CHANNEL_3);
      HAL_GPIO_WritePin(MOTOR_OCNPWM3_GPIO_PORT, MOTOR_OCNPWM3_PIN, GPIO_PIN_RESET);
    
      /*  Channe2 configuration */
//      HAL_TIM_PWM_Start(&htimx_bldcm, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htimx_bldcm,TIM_CHANNEL_2,PWM_PERIOD_COUNT/10);
    
      HAL_GPIO_WritePin(MOTOR_OCNPWM1_GPIO_PORT, MOTOR_OCNPWM1_PIN, GPIO_PIN_SET);
      break;
    
    case 6: // U+ W-
      /*  Channe2 configuration */ 
    __HAL_TIM_SET_COMPARE(&htimx_bldcm,TIM_CHANNEL_2,0);
//      HAL_TIM_PWM_Stop(&htimx_bldcm, TIM_CHANNEL_2);
      HAL_GPIO_WritePin(MOTOR_OCNPWM2_GPIO_PORT, MOTOR_OCNPWM2_PIN, GPIO_PIN_RESET);
    
      /*  Channel configuration */
//      HAL_TIM_PWM_Start(&htimx_bldcm, TIM_CHANNEL_1); 
    __HAL_TIM_SET_COMPARE(&htimx_bldcm,TIM_CHANNEL_1,PWM_PERIOD_COUNT/10);
      HAL_GPIO_WritePin(MOTOR_OCNPWM3_GPIO_PORT, MOTOR_OCNPWM3_PIN, GPIO_PIN_SET);
      break;
  }

  update = 0;
}

/**
  * @brief  ��ʱ�������жϻص�����
  * @param  htim:��ʱ�����
  * @retval ��
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (update++ > 1)    // ��һ���ڲ��������ж�ǰ����������û�в���ֵ
  {
    printf("��ת��ʱ\r\n");
    update = 0;
    
    LED1_ON;     // ����LED1��ʾ��ת��ʱֹͣ
    
    /* ��ת��ʱֹͣ PWM ��� */
//    hall_disable();       // ���û����������ӿ�
//    stop_pwm_output();    // ֹͣ PWM ���
  }
}

/*********************************************END OF FILE**********************/
