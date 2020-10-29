/**
  ******************************************************************************
  * @file    bsp_motor_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2020-xx-xx
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

TIM_HandleTypeDef  motor1_htimx_bldcm;
TIM_OC_InitTypeDef MOTOR1_TIM_OCInitStructure;

TIM_HandleTypeDef  motor2_htimx_bldcm;
TIM_OC_InitTypeDef MOTOR2_TIM_OCInitStructure;

/* ������������ض�ʱ����ʼ�� */
TIM_HandleTypeDef motor1_htimx_hall;
TIM_HandleTypeDef motor2_htimx_hall;

static uint16_t motor1_bldcm_pulse = 0;
static uint16_t motor2_bldcm_pulse = 0;

/**
  * @brief  ����TIM�������PWMʱ�õ���I/O
  * @param  ��
  * @retval ��
  */
static void TIMx_GPIO_Config(void) 
{
  /*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
  GPIO_InitTypeDef GPIO_InitStructure;

  /*�������1��ʱ����ص�GPIO����ʱ��*/
  MOTOR1_OCPWM1_GPIO_CLK_ENABLE();
  MOTOR1_OCNPWM1_GPIO_CLK_ENABLE();
  MOTOR1_OCPWM2_GPIO_CLK_ENABLE();
  MOTOR1_OCNPWM2_GPIO_CLK_ENABLE();
  MOTOR1_OCPWM3_GPIO_CLK_ENABLE();
  MOTOR1_OCNPWM3_GPIO_CLK_ENABLE();
	
	/*�������2��ʱ����ص�GPIO����ʱ��*/
	MOTOR2_OCPWM1_GPIO_CLK_ENABLE();
  MOTOR2_OCNPWM1_GPIO_CLK_ENABLE();
  MOTOR2_OCPWM2_GPIO_CLK_ENABLE();
  MOTOR2_OCNPWM2_GPIO_CLK_ENABLE();
  MOTOR2_OCPWM3_GPIO_CLK_ENABLE();
  MOTOR2_OCNPWM3_GPIO_CLK_ENABLE();

  /* ��ʱ���������ų�ʼ�� */															   
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;   // �������ģʽ

  GPIO_InitStructure.Pin = MOTOR1_OCNPWM1_PIN;
  HAL_GPIO_Init(MOTOR1_OCNPWM1_GPIO_PORT, &GPIO_InitStructure);	

  GPIO_InitStructure.Pin = MOTOR1_OCNPWM2_PIN;	
  HAL_GPIO_Init(MOTOR1_OCNPWM2_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = MOTOR1_OCNPWM3_PIN;	
  HAL_GPIO_Init(MOTOR1_OCNPWM3_GPIO_PORT, &GPIO_InitStructure);	
	
	GPIO_InitStructure.Pin = MOTOR2_OCNPWM1_PIN;
  HAL_GPIO_Init(MOTOR2_OCNPWM1_GPIO_PORT, &GPIO_InitStructure);	

  GPIO_InitStructure.Pin = MOTOR2_OCNPWM2_PIN;	
  HAL_GPIO_Init(MOTOR2_OCNPWM2_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = MOTOR2_OCNPWM3_PIN;	
  HAL_GPIO_Init(MOTOR2_OCNPWM3_GPIO_PORT, &GPIO_InitStructure);	

  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;  
  /* ͨ�� 1 */
  GPIO_InitStructure.Pin = MOTOR1_OCPWM1_PIN;
  GPIO_InitStructure.Alternate = MOTOR1_OCPWM1_AF;	
  HAL_GPIO_Init(MOTOR1_OCPWM1_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = MOTOR2_OCPWM1_PIN;
  GPIO_InitStructure.Alternate = MOTOR2_OCPWM1_AF;	
  HAL_GPIO_Init(MOTOR2_OCPWM1_GPIO_PORT, &GPIO_InitStructure);
	
	/* ͨ�� 2 */
  GPIO_InitStructure.Pin = MOTOR1_OCPWM2_PIN;	
  GPIO_InitStructure.Alternate = MOTOR1_OCPWM2_AF;	
  HAL_GPIO_Init(MOTOR1_OCPWM2_GPIO_PORT, &GPIO_InitStructure);
	
  GPIO_InitStructure.Pin = MOTOR2_OCPWM2_PIN;	
  GPIO_InitStructure.Alternate = MOTOR2_OCPWM2_AF;	
  HAL_GPIO_Init(MOTOR2_OCPWM2_GPIO_PORT, &GPIO_InitStructure);

  /* ͨ�� 3 */
  GPIO_InitStructure.Pin = MOTOR1_OCPWM3_PIN;	
  GPIO_InitStructure.Alternate = MOTOR1_OCPWM3_AF;	
  HAL_GPIO_Init(MOTOR1_OCPWM3_GPIO_PORT, &GPIO_InitStructure);
	
  GPIO_InitStructure.Pin = MOTOR2_OCPWM3_PIN;	
  GPIO_InitStructure.Alternate = MOTOR2_OCPWM3_AF;	
  HAL_GPIO_Init(MOTOR2_OCPWM3_GPIO_PORT, &GPIO_InitStructure);
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
  MOTOR1_TIM_CLK_ENABLE(); 
  MOTOR2_TIM_CLK_ENABLE(); 
  /* ���嶨ʱ���ľ����ȷ����ʱ���Ĵ����Ļ���ַ*/
  motor1_htimx_bldcm.Instance = MOTOR1_TIM;
  motor2_htimx_bldcm.Instance = MOTOR2_TIM;
  /* �ۼ� TIM_Period�������һ�����»����ж�*/		
  //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
  motor1_htimx_bldcm.Init.Period = MOTOR1_PWM_PERIOD_COUNT - 1;
	motor2_htimx_bldcm.Init.Period = MOTOR2_PWM_PERIOD_COUNT - 1;
	
  // �߼����ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK=216MHz 
  // �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=1MHz
  motor1_htimx_bldcm.Init.Prescaler = MOTOR1_PWM_PRESCALER_COUNT - 1;	
  motor2_htimx_bldcm.Init.Prescaler = MOTOR2_PWM_PRESCALER_COUNT - 1;	
	
  // ����ʱ�ӷ�Ƶ
  motor1_htimx_bldcm.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  motor2_htimx_bldcm.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  // ������ʽ
  motor1_htimx_bldcm.Init.CounterMode=TIM_COUNTERMODE_UP;
  motor2_htimx_bldcm.Init.CounterMode=TIM_COUNTERMODE_UP;
	
  // �ظ�������
  motor1_htimx_bldcm.Init.RepetitionCounter=0;	
  motor2_htimx_bldcm.Init.RepetitionCounter=0;
	
  // ��ʼ����ʱ��TIMx, x[1,8]
  HAL_TIM_PWM_Init(&motor1_htimx_bldcm);
	HAL_TIM_PWM_Init(&motor2_htimx_bldcm);

  /*PWMģʽ����*/
  //����ΪPWMģʽ1
  MOTOR1_TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
  MOTOR1_TIM_OCInitStructure.Pulse = 0;                         // Ĭ�ϱ���Ҫ��ʼΪ0
  MOTOR1_TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
  MOTOR1_TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  MOTOR1_TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
  MOTOR1_TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  HAL_TIM_PWM_ConfigChannel(&motor1_htimx_bldcm,&MOTOR1_TIM_OCInitStructure,TIM_CHANNEL_1);    // ��ʼ��ͨ�� 1 ��� PWM 
  HAL_TIM_PWM_ConfigChannel(&motor1_htimx_bldcm,&MOTOR1_TIM_OCInitStructure,TIM_CHANNEL_2);    // ��ʼ��ͨ�� 2 ��� PWM
  HAL_TIM_PWM_ConfigChannel(&motor1_htimx_bldcm,&MOTOR1_TIM_OCInitStructure,TIM_CHANNEL_3);    // ��ʼ��ͨ�� 3 ��� PWM
	
	MOTOR2_TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
  MOTOR2_TIM_OCInitStructure.Pulse = 0;                         // Ĭ�ϱ���Ҫ��ʼΪ0
  MOTOR2_TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
  MOTOR2_TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  MOTOR2_TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
  MOTOR2_TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  HAL_TIM_PWM_ConfigChannel(&motor2_htimx_bldcm,&MOTOR2_TIM_OCInitStructure,TIM_CHANNEL_1);    // ��ʼ��ͨ�� 1 ��� PWM 
  HAL_TIM_PWM_ConfigChannel(&motor2_htimx_bldcm,&MOTOR2_TIM_OCInitStructure,TIM_CHANNEL_2);    // ��ʼ��ͨ�� 2 ��� PWM
  HAL_TIM_PWM_ConfigChannel(&motor2_htimx_bldcm,&MOTOR2_TIM_OCInitStructure,TIM_CHANNEL_3);    // ��ʼ��ͨ�� 3 ��� PWM
  
  /* ���ô���Դ */
  HAL_TIMEx_ConfigCommutationEvent(&motor1_htimx_bldcm, MOTOR1_TIM_COM_TS_ITRx, TIM_COMMUTATION_SOFTWARE);
  HAL_TIMEx_ConfigCommutationEvent(&motor2_htimx_bldcm, MOTOR2_TIM_COM_TS_ITRx, TIM_COMMUTATION_SOFTWARE);
	
  /* ������ʱ��ͨ��1���PWM */
  HAL_TIM_PWM_Start(&motor1_htimx_bldcm,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&motor2_htimx_bldcm,TIM_CHANNEL_1);

  /* ������ʱ��ͨ��2���PWM */
  HAL_TIM_PWM_Start(&motor1_htimx_bldcm,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&motor2_htimx_bldcm,TIM_CHANNEL_2);
	
  /* ������ʱ��ͨ��3���PWM */
  HAL_TIM_PWM_Start(&motor1_htimx_bldcm,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&motor2_htimx_bldcm,TIM_CHANNEL_3);
}

/**
  * @brief  ֹͣpwm���
  * @param  ��
  * @retval ��
  */
void stop_motor1_pwm_output(void)
{
	/* �رն�ʱ��ͨ��1���PWM */
  __HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_1, 0);

  /* �رն�ʱ��ͨ��2���PWM */
  __HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_2, 0);
	
  /* �رն�ʱ��ͨ��3���PWM */
  __HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_3, 0);
  
  HAL_GPIO_WritePin(MOTOR1_OCNPWM1_GPIO_PORT, MOTOR1_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�
  HAL_GPIO_WritePin(MOTOR1_OCNPWM2_GPIO_PORT, MOTOR1_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
  HAL_GPIO_WritePin(MOTOR1_OCNPWM3_GPIO_PORT, MOTOR1_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
	
}

void stop_motor2_pwm_output(void)
{
	/* �رն�ʱ��ͨ��1���PWM */
  __HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_1, 0);
	
  /* �رն�ʱ��ͨ��2���PWM */
  __HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_2, 0);
	
  /* �رն�ʱ��ͨ��3���PWM */
  __HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_3, 0);
  
  HAL_GPIO_WritePin(MOTOR2_OCNPWM1_GPIO_PORT, MOTOR2_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�
  HAL_GPIO_WritePin(MOTOR2_OCNPWM2_GPIO_PORT, MOTOR2_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
  HAL_GPIO_WritePin(MOTOR2_OCNPWM3_GPIO_PORT, MOTOR2_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
		
}

/**
  * @brief  ����pwm�����ռ�ձ�
  * @param  pulse:Ҫ���õ�ռ�ձ�
  * @retval ��
  */
void set_motor1_pwm_pulse(uint16_t pulse)
{
  /* ���ö�ʱ��ͨ����� PWM ��ռ�ձ� */
	motor1_bldcm_pulse = pulse;
}

void set_motor2_pwm_pulse(uint16_t pulse)
{
  /* ���ö�ʱ��ͨ����� PWM ��ռ�ձ� */
	motor2_bldcm_pulse = pulse;
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
static void hall_motor1_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  MOTOR1_HALL_INPUTU_GPIO_CLK_ENABLE();
  MOTOR1_HALL_INPUTV_GPIO_CLK_ENABLE();
  MOTOR1_HALL_INPUTW_GPIO_CLK_ENABLE();
  
  /* ��ʱ��ͨ�� 1 ���ų�ʼ�� */
  GPIO_InitStruct.Pin = MOTOR1_HALL_INPUTU_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = MOTOR1_HALL_INPUTU_AF;
  HAL_GPIO_Init(MOTOR1_HALL_INPUTU_GPIO_PORT, &GPIO_InitStruct);
  
  /* ��ʱ��ͨ�� 2 ���ų�ʼ�� */
  GPIO_InitStruct.Pin = MOTOR1_HALL_INPUTV_PIN;
  HAL_GPIO_Init(MOTOR1_HALL_INPUTV_GPIO_PORT, &GPIO_InitStruct);

  /* ��ʱ��ͨ�� 3 ���ų�ʼ�� */
  GPIO_InitStruct.Pin = MOTOR1_HALL_INPUTW_PIN;
  HAL_GPIO_Init(MOTOR1_HALL_INPUTW_GPIO_PORT, &GPIO_InitStruct);
	
}

static void hall_motor2_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  MOTOR2_HALL_INPUTU_GPIO_CLK_ENABLE();
  MOTOR2_HALL_INPUTV_GPIO_CLK_ENABLE();
  MOTOR2_HALL_INPUTW_GPIO_CLK_ENABLE();
  
  /* ��ʱ��ͨ�� 1 ���ų�ʼ�� */
  GPIO_InitStruct.Pin = MOTOR2_HALL_INPUTU_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = MOTOR2_HALL_INPUTU_AF;
  HAL_GPIO_Init(MOTOR2_HALL_INPUTU_GPIO_PORT, &GPIO_InitStruct);
  
  /* ��ʱ��ͨ�� 2 ���ų�ʼ�� */
  GPIO_InitStruct.Pin = MOTOR2_HALL_INPUTV_PIN;
  HAL_GPIO_Init(MOTOR2_HALL_INPUTV_GPIO_PORT, &GPIO_InitStruct);

  /* ��ʱ��ͨ�� 3 ���ų�ʼ�� */
  GPIO_InitStruct.Pin = MOTOR2_HALL_INPUTW_PIN;
  HAL_GPIO_Init(MOTOR2_HALL_INPUTW_GPIO_PORT, &GPIO_InitStruct);
	
}
/**
  * @brief  ������������ʱ����ʼ��
  * @param  ��
  * @retval ��
  */
static void hall_motor1_tim_init(void)
{
  TIM_HallSensor_InitTypeDef  hall_sensor_cfg;  
  
  /* ������ʱ������ʱ��ʹ�� */
  MOTOR1_HALL_TIM_CLK_ENABLE();
  
  /* ��ʱ�������������� */
  motor1_htimx_hall.Instance = MOTOR1_HALL_TIM;
  motor1_htimx_hall.Init.Prescaler = MOTOR1_HALL_PRESCALER_COUNT - 1;       // Ԥ��Ƶ
  motor1_htimx_hall.Init.CounterMode = TIM_COUNTERMODE_UP;           // ���ϼ���
  motor1_htimx_hall.Init.Period = MOTOR1_HALL_PERIOD_COUNT - 1;             // ��������
  motor1_htimx_hall.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;     // ʱ�ӷ�Ƶ
  
  hall_sensor_cfg.IC1Prescaler = TIM_ICPSC_DIV1;            // ���벶���Ƶ
  hall_sensor_cfg.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;    // ���벶����
  hall_sensor_cfg.IC1Filter = 10;                           // �����˲�
  hall_sensor_cfg.Commutation_Delay = 0U;                   // ��ʹ���ӳٴ���
  HAL_TIMEx_HallSensor_Init(&motor1_htimx_hall, &hall_sensor_cfg);
  
  HAL_NVIC_SetPriority(MOTOR1_HALL_TIM_IRQn, 0, 0);    // �����ж����ȼ�
  HAL_NVIC_EnableIRQ(MOTOR1_HALL_TIM_IRQn);            // ʹ���ж�
}

static void hall_motor2_tim_init(void)
{
  TIM_HallSensor_InitTypeDef  hall_sensor_cfg;  
  
  /* ������ʱ������ʱ��ʹ�� */
  MOTOR2_HALL_TIM_CLK_ENABLE();
  
  /* ��ʱ�������������� */
  motor2_htimx_hall.Instance = MOTOR2_HALL_TIM;
  motor2_htimx_hall.Init.Prescaler = MOTOR2_HALL_PRESCALER_COUNT - 1;       // Ԥ��Ƶ
  motor2_htimx_hall.Init.CounterMode = TIM_COUNTERMODE_UP;           // ���ϼ���
  motor2_htimx_hall.Init.Period = MOTOR2_HALL_PERIOD_COUNT - 1;             // ��������
  motor2_htimx_hall.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;     // ʱ�ӷ�Ƶ
  
  hall_sensor_cfg.IC1Prescaler = TIM_ICPSC_DIV1;            // ���벶���Ƶ
  hall_sensor_cfg.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;    // ���벶����
  hall_sensor_cfg.IC1Filter = 10;                           // �����˲�
  hall_sensor_cfg.Commutation_Delay = 0U;                   // ��ʹ���ӳٴ���
  HAL_TIMEx_HallSensor_Init(&motor2_htimx_hall, &hall_sensor_cfg);
  
  HAL_NVIC_SetPriority(MOTOR2_HALL_TIM_IRQn, 0, 1);    // �����ж����ȼ�
  HAL_NVIC_EnableIRQ(MOTOR2_HALL_TIM_IRQn);            // ʹ���ж�
}

/**
  * @brief  ʹ�ܻ���������
  * @param  ��
  * @retval ��
  */
void hall_motor1_enable(void)
{
  /* ʹ�ܻ����������ӿ� */
  __HAL_TIM_ENABLE_IT(&motor1_htimx_hall, TIM_IT_TRIGGER);
  __HAL_TIM_ENABLE_IT(&motor1_htimx_hall, TIM_IT_UPDATE);
  
  HAL_TIMEx_HallSensor_Start(&motor1_htimx_hall);

  LED1_OFF;
  
  HAL_TIM_TriggerCallback(&motor1_htimx_hall);   // ִ��һ�λ���
}

void hall_motor2_enable(void)
{
  /* ʹ�ܻ����������ӿ� */
  __HAL_TIM_ENABLE_IT(&motor2_htimx_hall, TIM_IT_TRIGGER);
  __HAL_TIM_ENABLE_IT(&motor2_htimx_hall, TIM_IT_UPDATE);
  
  HAL_TIMEx_HallSensor_Start(&motor2_htimx_hall);

  LED2_OFF;
  
  HAL_TIM_TriggerCallback(&motor2_htimx_hall);   // ִ��һ�λ���
}

/**
  * @brief  ���û���������
  * @param  ��
  * @retval ��
  */
void hall_motor1_disable(void)
{
  /* ���û����������ӿ� */
  __HAL_TIM_DISABLE_IT(&motor1_htimx_hall, TIM_IT_TRIGGER);
  __HAL_TIM_DISABLE_IT(&motor1_htimx_hall, TIM_IT_UPDATE);
  HAL_TIMEx_HallSensor_Stop(&motor1_htimx_hall);
}

void hall_motor2_disable(void)
{
  /* ���û����������ӿ� */
  __HAL_TIM_DISABLE_IT(&motor2_htimx_hall, TIM_IT_TRIGGER);
  __HAL_TIM_DISABLE_IT(&motor2_htimx_hall, TIM_IT_UPDATE);
  HAL_TIMEx_HallSensor_Stop(&motor2_htimx_hall);
}


uint8_t get_hall_motor1_state(void)
{
  uint8_t state = 0;
  
#if 1
  /* ��ȡ���������� U ��״̬ */
  if(HAL_GPIO_ReadPin(MOTOR1_HALL_INPUTU_GPIO_PORT, MOTOR1_HALL_INPUTU_PIN) != GPIO_PIN_RESET)
  {
    state |= 0x01U << 0;
  }
  
  /* ��ȡ���������� V ��״̬ */
  if(HAL_GPIO_ReadPin(MOTOR1_HALL_INPUTV_GPIO_PORT, MOTOR1_HALL_INPUTV_PIN) != GPIO_PIN_RESET)
  {
    state |= 0x01U << 1;
  }
  
  /* ��ȡ���������� W ��״̬ */
  if(HAL_GPIO_ReadPin(MOTOR1_HALL_INPUTW_GPIO_PORT, MOTOR1_HALL_INPUTW_PIN) != GPIO_PIN_RESET)
  {
    state |= 0x01U << 2;
  }
	

#else
  state = (GPIOH->IDR >> 10) & 7;    // ������ӿ�һ 3 ��������������״̬

#endif

  return state;    // ���ش�����״̬
}


uint8_t get_hall_motor2_state(void)
{
  uint8_t state = 0;
  
#if 1
  /* ��ȡ���������� U ��״̬ */
  if(HAL_GPIO_ReadPin(MOTOR2_HALL_INPUTU_GPIO_PORT, MOTOR2_HALL_INPUTU_PIN) != GPIO_PIN_RESET)
  {
    state |= 0x01U << 0;
  }
  
  /* ��ȡ���������� V ��״̬ */
  if(HAL_GPIO_ReadPin(MOTOR2_HALL_INPUTV_GPIO_PORT, MOTOR2_HALL_INPUTV_PIN) != GPIO_PIN_RESET)
  {
    state |= 0x01U << 1;
  }
  
  /* ��ȡ���������� W ��״̬ */
  if(HAL_GPIO_ReadPin(MOTOR2_HALL_INPUTW_GPIO_PORT, MOTOR2_HALL_INPUTW_PIN) != GPIO_PIN_RESET)
  {
    state |= 0x01U << 2;
  }
	

#else
  state = (GPIOH->IDR >> 10) & 7;    // ������ӿ�һ 3 ��������������״̬

#endif

  return state;    // ���ش�����״̬
}

/**
  * @brief  ��ʼ��������������ʱ��
  * @param  ��
  * @retval ��
  */
void hall_motor1_tim_config(void)
{
	hall_motor1_gpio_init();	    // ��ʼ������
	hall_motor1_tim_init();      // ��ʼ����ʱ��
}

void hall_motor2_tim_config(void)
{
	hall_motor2_gpio_init();	    // ��ʼ������
	hall_motor2_tim_init();      // ��ʼ����ʱ��
}

int motor1_update = 0;     // ��ʱ�����¼���
int motor2_update = 0;     // ��ʱ�����¼���
/**
  * @brief  ���������������ص�����
  * @param  htim:��ʱ�����
  * @retval ��
  */
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim)
{
  /* ��ȡ��������������״̬,��Ϊ��������� */

	if (htim == &motor1_htimx_hall)
	{  
			uint8_t step = 0;
			step = get_hall_motor1_state();
			if(get_motor1_bldcm_direction() == MOTOR_FWD)
				{
					switch(step)
					{
						case 1:    /* U+ W- */
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 2 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM2_GPIO_PORT, MOTOR1_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM1_GPIO_PORT, MOTOR1_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_1, motor1_bldcm_pulse);      // ͨ�� 1 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR1_OCNPWM3_GPIO_PORT, MOTOR1_OCNPWM3_PIN, GPIO_PIN_SET);      // �������ű�
							break;
						
						case 2:     /* V+ U- */
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 3 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM3_GPIO_PORT, MOTOR1_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM2_GPIO_PORT, MOTOR1_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_2, motor1_bldcm_pulse);      // ͨ�� 2 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR1_OCNPWM1_GPIO_PORT, MOTOR1_OCNPWM1_PIN, GPIO_PIN_SET);      // �������ű�
						
							break;
						
						case 3:    /* V+ W- */
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM1_GPIO_PORT, MOTOR1_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM2_GPIO_PORT, MOTOR1_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
							
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_2, motor1_bldcm_pulse);      // ͨ�� 2 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR1_OCNPWM3_GPIO_PORT, MOTOR1_OCNPWM3_PIN, GPIO_PIN_SET);      // �������ű�
							break;
						
						case 4:     /* W+ V- */
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM1_GPIO_PORT, MOTOR1_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM3_GPIO_PORT, MOTOR1_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
				 
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_3, motor1_bldcm_pulse);      // ͨ�� 3 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR1_OCNPWM2_GPIO_PORT, MOTOR1_OCNPWM2_PIN, GPIO_PIN_SET);      // �������ű� 
							break;
						
						case 5:     /* U+  V -*/
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 3 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM3_GPIO_PORT, MOTOR1_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM1_GPIO_PORT, MOTOR1_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_1, motor1_bldcm_pulse);      // ͨ�� 1 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR1_OCNPWM2_GPIO_PORT, MOTOR1_OCNPWM2_PIN, GPIO_PIN_SET);      // �������ű�
							break;
						
						case 6:     /* W+ U- */
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 2 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM2_GPIO_PORT, MOTOR1_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM3_GPIO_PORT, MOTOR1_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_3, motor1_bldcm_pulse);      // ͨ�� 3 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR1_OCNPWM1_GPIO_PORT, MOTOR1_OCNPWM1_PIN, GPIO_PIN_SET);      // �������ű�
							break;
					}
				}
				else
				{
					switch(step)
					{
						case 1:   /* W+ U- */
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 2 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM2_GPIO_PORT, MOTOR1_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM3_GPIO_PORT, MOTOR1_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_3, motor1_bldcm_pulse);      // ͨ�� 3 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR1_OCNPWM1_GPIO_PORT, MOTOR1_OCNPWM1_PIN, GPIO_PIN_SET);      // �������ű�
							break;
						
						case 2:    /* U+  V -*/
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 3 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM3_GPIO_PORT, MOTOR1_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM1_GPIO_PORT, MOTOR1_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_1, motor1_bldcm_pulse);      // ͨ�� 1 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR1_OCNPWM2_GPIO_PORT, MOTOR1_OCNPWM2_PIN, GPIO_PIN_SET);      // �������ű�
							break;
						
						case 3:   /* W+ V- */
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM1_GPIO_PORT, MOTOR1_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM3_GPIO_PORT, MOTOR1_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
				 
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_3, motor1_bldcm_pulse);      // ͨ�� 3 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR1_OCNPWM2_GPIO_PORT, MOTOR1_OCNPWM2_PIN, GPIO_PIN_SET);      // �������ű�        

							break;
						
						case 4:    /* V+ W- */
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM1_GPIO_PORT, MOTOR1_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM2_GPIO_PORT, MOTOR1_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
							
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_2, motor1_bldcm_pulse);      // ͨ�� 2 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR1_OCNPWM3_GPIO_PORT, MOTOR1_OCNPWM3_PIN, GPIO_PIN_SET);      // �������ű�
							break;
						
						case 5:    /* V+ U- */
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 3 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM3_GPIO_PORT, MOTOR1_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM2_GPIO_PORT, MOTOR1_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_2, motor1_bldcm_pulse);      // ͨ�� 2 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR1_OCNPWM1_GPIO_PORT, MOTOR1_OCNPWM1_PIN, GPIO_PIN_SET);      // �������ű�
							break;
						
						case 6:    /* U+ W- */
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 2 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM2_GPIO_PORT, MOTOR1_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR1_OCNPWM1_GPIO_PORT, MOTOR1_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor1_htimx_bldcm, TIM_CHANNEL_1, motor1_bldcm_pulse);      // ͨ�� 1 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR1_OCNPWM3_GPIO_PORT, MOTOR1_OCNPWM3_PIN, GPIO_PIN_SET);      // �������ű�
							break;
					}
				}			
			HAL_TIM_GenerateEvent(&motor1_htimx_bldcm, TIM_EVENTSOURCE_COM);    // ������������¼�����ʱ�Ž�����д��
			motor1_update = 0;
	}
	else if (htim == &motor2_htimx_hall)
	{
			uint8_t step = 0;
			step = get_hall_motor2_state();
			if(get_motor2_bldcm_direction() == MOTOR_FWD)
				{
					switch(step)
					{
						case 1:    /* U+ W- */
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 2 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM2_GPIO_PORT, MOTOR2_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM1_GPIO_PORT, MOTOR2_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_1, motor2_bldcm_pulse);      // ͨ�� 1 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR2_OCNPWM3_GPIO_PORT, MOTOR2_OCNPWM3_PIN, GPIO_PIN_SET);      // �������ű�
							break;
						
						case 2:     /* V+ U- */
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 3 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM3_GPIO_PORT, MOTOR2_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM2_GPIO_PORT, MOTOR2_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_2, motor2_bldcm_pulse);      // ͨ�� 2 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR2_OCNPWM1_GPIO_PORT, MOTOR2_OCNPWM1_PIN, GPIO_PIN_SET);      // �������ű�
						
							break;
						
						case 3:    /* V+ W- */
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM1_GPIO_PORT, MOTOR2_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM2_GPIO_PORT, MOTOR2_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
							
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_2, motor2_bldcm_pulse);      // ͨ�� 2 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR2_OCNPWM3_GPIO_PORT, MOTOR2_OCNPWM3_PIN, GPIO_PIN_SET);      // �������ű�
							break;
						
						case 4:     /* W+ V- */
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM1_GPIO_PORT, MOTOR2_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM3_GPIO_PORT, MOTOR2_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
				 
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_3, motor2_bldcm_pulse);      // ͨ�� 3 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR2_OCNPWM2_GPIO_PORT, MOTOR2_OCNPWM2_PIN, GPIO_PIN_SET);      // �������ű� 
							break;
						
						case 5:     /* U+  V -*/
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 3 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM3_GPIO_PORT, MOTOR2_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM1_GPIO_PORT, MOTOR2_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_1, motor2_bldcm_pulse);      // ͨ�� 1 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR2_OCNPWM2_GPIO_PORT, MOTOR2_OCNPWM2_PIN, GPIO_PIN_SET);      // �������ű�
							break;
						
						case 6:     /* W+ U- */
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 2 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM2_GPIO_PORT, MOTOR2_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM3_GPIO_PORT, MOTOR2_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_3, motor2_bldcm_pulse);      // ͨ�� 3 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR2_OCNPWM1_GPIO_PORT, MOTOR2_OCNPWM1_PIN, GPIO_PIN_SET);      // �������ű�
							break;
					}
				}
				else
				{
					switch(step)
					{
						case 1:   /* W+ U- */
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 2 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM2_GPIO_PORT, MOTOR2_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM3_GPIO_PORT, MOTOR2_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_3, motor2_bldcm_pulse);      // ͨ�� 3 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR2_OCNPWM1_GPIO_PORT, MOTOR2_OCNPWM1_PIN, GPIO_PIN_SET);      // �������ű�
							break;
						
						case 2:    /* U+  V -*/
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 3 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM3_GPIO_PORT, MOTOR2_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM1_GPIO_PORT, MOTOR2_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_1, motor2_bldcm_pulse);      // ͨ�� 1 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR2_OCNPWM2_GPIO_PORT, MOTOR2_OCNPWM2_PIN, GPIO_PIN_SET);      // �������ű�
							break;
						
						case 3:   /* W+ V- */
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM1_GPIO_PORT, MOTOR2_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM3_GPIO_PORT, MOTOR2_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�
				 
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_3, motor2_bldcm_pulse);      // ͨ�� 3 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR2_OCNPWM2_GPIO_PORT, MOTOR2_OCNPWM2_PIN, GPIO_PIN_SET);      // �������ű�        

							break;
						
						case 4:    /* V+ W- */
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM1_GPIO_PORT, MOTOR2_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM2_GPIO_PORT, MOTOR2_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
							
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_2, motor2_bldcm_pulse);      // ͨ�� 2 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR2_OCNPWM3_GPIO_PORT, MOTOR2_OCNPWM3_PIN, GPIO_PIN_SET);      // �������ű�
							break;
						
						case 5:    /* V+ U- */
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 3 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM3_GPIO_PORT, MOTOR2_OCNPWM3_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_1, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM2_GPIO_PORT, MOTOR2_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_2, motor2_bldcm_pulse);      // ͨ�� 2 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR2_OCNPWM1_GPIO_PORT, MOTOR2_OCNPWM1_PIN, GPIO_PIN_SET);      // �������ű�
							break;
						
						case 6:    /* U+ W- */
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_2, 0);                       // ͨ�� 2 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM2_GPIO_PORT, MOTOR2_OCNPWM2_PIN, GPIO_PIN_RESET);    // �ر����ű�
						
							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_3, 0);                       // ͨ�� 1 ����Ϊ 0
							HAL_GPIO_WritePin(MOTOR2_OCNPWM1_GPIO_PORT, MOTOR2_OCNPWM1_PIN, GPIO_PIN_RESET);    // �ر����ű�

							__HAL_TIM_SET_COMPARE(&motor2_htimx_bldcm, TIM_CHANNEL_1, motor2_bldcm_pulse);      // ͨ�� 1 ���õ�ռ�ձ�
							HAL_GPIO_WritePin(MOTOR2_OCNPWM3_GPIO_PORT, MOTOR2_OCNPWM3_PIN, GPIO_PIN_SET);      // �������ű�
							break;
					}
			}
			HAL_TIM_GenerateEvent(&motor2_htimx_bldcm, TIM_EVENTSOURCE_COM);    // ������������¼�����ʱ�Ž�����д��	
			motor2_update = 0;
	}
}

/**
  * @brief  ��ʱ�������жϻص�����
  * @param  htim:��ʱ�����
  * @retval ��
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* ��ת��ʱֹͣ PWM ��� */
		if (htim == &motor1_htimx_bldcm)
		{
			if (motor1_update++ > 1)    // ��һ���ڲ��������ж�ǰ����������û�в���ֵ
			{
				printf("��ת��ʱ\r\n");
				motor1_update = 0;
				
				LED1_ON;     // ����LED1��ʾ��ת��ʱֹͣ
			}
			hall_motor1_disable();       // ���û����������ӿ�
			stop_motor1_pwm_output();    // ֹͣ PWM ���		
		}
		else if (htim == &motor2_htimx_bldcm)
		{
			if (motor2_update++ > 1)    // ��һ���ڲ��������ж�ǰ����������û�в���ֵ
			{
				printf("��ת��ʱ\r\n");
				motor2_update = 0;
				
				LED2_ON;     // ����LED1��ʾ��ת��ʱֹͣ
			}			
			hall_motor2_disable();       // ���û����������ӿ�
			stop_motor2_pwm_output();    // ֹͣ PWM ���		
		}
  
}

/*********************************************END OF FILE**********************/
