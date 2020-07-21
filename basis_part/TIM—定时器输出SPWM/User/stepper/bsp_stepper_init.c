#include "./stepper/bsp_stepper_init.h"
#include "./led/bsp_led.h"   
#include "./delay/core_delay.h"   
#include "stm32h7xx.h"


TIM_HandleTypeDef TIM_TimeBaseStructure;
 /**
  * @brief  ͨ�ö�ʱ�� TIMx,x[6,7]�ж����ȼ�����
  * @param  ��
  * @retval ��
  */
static void TIMx_NVIC_Configuration(void)
{
    /* �����ж����� */
    HAL_NVIC_SetPriority(GENERAL_TIM_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(GENERAL_TIM_IRQ);
}

/*
 * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
 * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
 * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         ����
 * TIM_CounterMode			 TIMx,x[6,7]û�У��������У�ͨ�ö�ʱ����
 * TIM_Period            ����
 * TIM_ClockDivision     TIMx,x[6,7]û�У���������(ͨ�ö�ʱ��)
 * TIM_RepetitionCounter TIMx,x[1,8]����(�߼���ʱ��)
 *-----------------------------------------------------------------------------
 */
static void TIM_Mode_Config(void)
{

	GENERAL_TIM_CLK_ENABLE();
  TIM_OC_InitTypeDef  TIM_OCInitStructure;  
	
	int tim_per=512;
	
	/*��ʱ������*/
  TIM_TimeBaseStructure.Instance = GENERAL_TIM;
  TIM_TimeBaseStructure.Init.Period = tim_per-1;									//����
  TIM_TimeBaseStructure.Init.Prescaler = 9-1;								//��Ƶϵ��
	TIM_TimeBaseStructure.Init.CounterMode=TIM_COUNTERMODE_UP;	//���ϼ���
	TIM_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&TIM_TimeBaseStructure);
	
	/*pwm����*/

	TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1; 
	TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_LOW;          
	TIM_OCInitStructure.Pulse = 0;  
	TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;   
	TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;  


	HAL_TIM_OC_ConfigChannel(&TIM_TimeBaseStructure, &TIM_OCInitStructure, MOTOR_PUL_CHANNEL_x);

	/* ȷ����ʱ�� */
	HAL_TIM_Base_Start(&TIM_TimeBaseStructure);
	/* �����Ƚ������ʹ���ж� */
	HAL_TIM_OC_Start_IT(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x);
	/*ʹ�ܱȽ�ͨ��*/
	TIM_CCxChannelCmd(GENERAL_TIM,MOTOR_PUL_CHANNEL_x,TIM_CCx_ENABLE);

	// ������ʱ�������ж�
	HAL_TIM_Base_Start_IT(&TIM_TimeBaseStructure);	
}

/**
  * @brief  ��ʼ��ͨ�ö�ʱ����ʱ
  * @param  ��
  * @retval ��
  */
void TIMx_Configuration(void)
{
	TIMx_NVIC_Configuration();	
  
	TIM_Mode_Config();
}

/**
  * @brief  ����TIM�������PWMʱ�õ���I/O
  * @param  ��
  * @retval ��
  */
static void Stepper_GPIO_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*����Motor��ص�GPIO����ʱ��*/
	MOTOR_PUL_GPIO_CLK_ENABLE();

  /* ��ʱ��ͨ��1��������IO��ʼ�� */
	/*�����������*/
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	/*������������ */ 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*���ø���*/
  GPIO_InitStruct.Alternate = MOTOR_PUL_GPIO_AF;
	/*���ø���*/
	GPIO_InitStruct.Pull =GPIO_PULLUP;
	/*ѡ��Ҫ���Ƶ�GPIO����*/	
	GPIO_InitStruct.Pin = MOTOR_PUL_PIN;
	/*Motor �������� ��ʼ��*/
  HAL_GPIO_Init(MOTOR_PUL_GPIO_PORT, &GPIO_InitStruct);			
	
}


/**
  * @brief  ����TIMͨ����ռ�ձ�
	* @param  channel		ͨ��	��1,2,3,4��
	* @param  compare		ռ�ձ�
	*	@note 	��
  * @retval ��
  */
void TIM2_SetPWM_pulse(int channel,int compare)
{
		switch(channel)
	{
		case 1:  	__HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_1,compare);break;
		case 2:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_2,compare);break;
		case 3:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_3,compare);break;
		case 4:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_4,compare);break;
	}
}

/**
  * @brief  ���ų�ʼ��
  * @retval ��
  */
void stepper_Init()
{
		Stepper_GPIO_Config();

		TIMx_Configuration();
}


/**
  * @brief  ��ʱ���жϺ���
	*	@note 		��
  * @retval ��
  */
void  GENERAL_TIM_INT_IRQHandler (void)
{
	HAL_TIM_IRQHandler(&TIM_TimeBaseStructure);	 	
}


/* SPWM��,�������ߣ��˱�ʹ�ù���Ŀ¼�µ�python�ű�sin_wave.py����*/
const uint16_t indexWave[] = {
0, 9, 18, 27, 36, 45, 54, 63, 72, 81, 89, 98,
	107, 116, 125, 133, 142, 151, 159, 168, 176,
	184, 193, 201, 209, 218, 226, 234, 242, 249,
	257, 265, 273, 280, 288, 295, 302, 310, 317, 
	324, 331, 337, 344, 351, 357, 364, 370, 376, 
	382, 388, 394, 399, 405, 410, 416, 421, 426, 
	431, 436, 440, 445, 449, 454, 458, 462, 465, 
	469, 473, 476, 479, 482, 485, 488, 491, 493, 
	496, 498, 500, 502, 503, 505, 506, 508, 509, 
	510, 510, 511, 512, 512, 512, 512, 512, 512,
	511, 510, 510, 509, 508, 506, 505, 503, 502,
	500, 498, 496, 493, 491, 488, 485, 482, 479,
	476, 473, 469, 465, 462, 458, 454, 449, 445, 
	440, 436, 431, 426, 421, 416, 410, 405, 399, 
	394, 388, 382, 376, 370, 364, 357, 351, 344, 
	337, 331, 324, 	317, 310, 302, 295, 288, 280, 
	273, 265, 257, 249, 242, 234, 226, 218, 209, 
	201, 193, 184, 176, 168, 159, 151, 142, 133, 
125, 116, 107, 98, 89, 81, 72, 63, 54, 45, 36,
27, 18, 9, 0
	
};
/*
�����ڵ����ұ�
ϸ����Ϊ360����0~�з�Ϊ360��
*/
float wave_ary[] = { 
0.0,0.00875,0.0175,0.02625,0.035,0.04374,0.05248,0.06122,0.06995,0.07868,0.0874,
0.09611,0.10482,0.11352,0.12221,0.13089,0.13956,0.14822,0.15687,0.1655,0.17413,
0.18274,0.19133,0.19992,0.20848,0.21703,0.22557,0.23408,0.24258,0.25106,0.25952,
0.26796,0.27639,0.28478,0.29316,0.30152,0.30985,0.31816,0.32644,0.3347,0.34293,
0.35114,0.35932,0.36747,0.3756,0.38369,0.39176,0.3998,0.4078,0.41578,0.42372,
0.43163,0.43951,0.44735,0.45516,0.46293,0.47067,0.47838,0.48604,0.49367,0.50126,
0.50882,0.51633,0.5238,0.53124,0.53863,0.54598,0.55329,0.56056,0.56779,0.57497,
0.58211,0.5892,0.59625,0.60325,0.61021,0.61712,0.62398,0.63079,0.63756,0.64428,
0.65094,0.65756,0.66413,0.67065,0.67711,0.68353,0.68989,0.6962,0.70245,0.70865,
0.7148,0.72089,0.72693,0.73291,0.73884,0.7447,0.75052,0.75627,0.76197,0.7676,
0.77318,0.7787,0.78416,0.78956,0.7949,0.80018,0.8054,0.81056,0.81565,0.82068,
0.82565,0.83056,0.8354,0.84018,0.84489,0.84954,0.85412,0.85864,0.86309,0.86748,
0.8718,0.87605,0.88024,0.88436,0.88841,0.89239,0.89631,0.90015,0.90393,0.90764,
0.91128,0.91485,0.91834,0.92177,0.92513,0.92842,0.93163,0.93478,0.93785,0.94085,
0.94378,0.94664,0.94942,0.95213,0.95477,0.95734,0.95983,0.96225,0.96459,0.96686,
0.96906,0.97118,0.97323,0.97521,0.9771,0.97893,0.98068,0.98235,0.98395,0.98548,
0.98692,0.9883,0.98959,0.99082,0.99196,0.99303,0.99402,0.99494,0.99578,0.99655,
0.99723,0.99785,0.99838,0.99884,0.99922,0.99953,0.99976,0.99991,0.99999,0.99999,
0.99991,0.99976,0.99953,0.99922,0.99884,0.99838,0.99785,0.99723,0.99655,0.99578,
0.99494,0.99402,0.99303,0.99196,0.99082,0.98959,0.9883,0.98692,0.98548,0.98395,
0.98235,0.98068,0.97893,0.9771,0.97521,0.97323,0.97118,0.96906,0.96686,0.96459,
0.96225,0.95983,0.95734,0.95477,0.95213,0.94942,0.94664,0.94378,0.94085,0.93785,
0.93478,0.93163,0.92842,0.92513,0.92177,0.91834,0.91485,0.91128,0.90764,0.90393,
0.90015,0.89631,0.89239,0.88841,0.88436,0.88024,0.87605,0.8718,0.86748,0.86309,
0.85864,0.85412,0.84954,0.84489,0.84018,0.8354,0.83056,0.82565,0.82068,0.81565,
0.81056,0.8054,0.80018,0.7949,0.78956,0.78416,0.7787,0.77318,0.7676,0.76197,
0.75627,0.75052,0.7447,0.73884,0.73291,0.72693,0.72089,0.7148,0.70865,0.70245,
0.6962,0.68989,0.68353,0.67711,0.67065,0.66413,0.65756,0.65094,0.64428,0.63756,
0.63079,0.62398,0.61712,0.61021,0.60325,0.59625,0.5892,0.58211,0.57497,0.56779,
0.56056,0.55329,0.54598,0.53863,0.53124,0.5238,0.51633,0.50882,0.50126,0.49367,
0.48604,0.47838,0.47067,0.46293,0.45516,0.44735,0.43951,0.43163,0.42372,0.41578,
0.4078,0.3998,0.39176,0.38369,0.3756,0.36747,0.35932,0.35114,0.34293,0.3347,
0.32644,0.31816,0.30985,0.30152,0.29316,0.28478,0.27639,0.26796,0.25952,0.25106,
0.24258,0.23408,0.22557,0.21703,0.20848,0.19992,0.19133,0.18274,0.17413,0.1655,
0.15687,0.14822,0.13956,0.13089,0.12221,0.11352,0.10482,0.09611,0.0874,0.07868,
0.06995,0.06122,0.05248,0.04374,0.035,0.02625,0.0175,0.00875,0.0};

//����PWM���ж��ٸ�Ԫ��
uint16_t POINT_NUM = sizeof(wave_ary)/sizeof(wave_ary[0]); 
/*��ѹ��ֵ�ȼ���*/
#define AMPLITUDE_CLASS 1024

//����������ε�Ƶ��
__IO uint16_t period_class = 1;

#define INIT_PHASE 0 * 2				//��ʼ��λ
uint16_t pwm_index = INIT_PHASE;//����PWM���  ����������
uint16_t amplitude_cnt = 0;			//���ڼ����ֵ�ȼ�

/**
  * @brief  �ص�����
	*	@note 		��
  * @retval ��
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	float *p = wave_ary;
	if(htim==(&TIM_TimeBaseStructure))
	{
			amplitude_cnt++;	//����
						
			//ÿ��PWM���е�ÿ��Ԫ����AMPLITUDE_CLASS���ȼ���
		  //ÿ����һ�������һ�����壬��PWM���е�Ԫ�ض�ʹ��һ��
		  //ʹ��256��
			if(amplitude_cnt > (500))		 						
			{		
					pwm_index++;												//��־PWM��ָ����һ��Ԫ��
					//��PWM���ѵ����β������ָ���ͷ
					if( pwm_index >=  POINT_NUM)			
					{
							pwm_index=0;						//�ع��ʼ��λ
					}

					amplitude_cnt=0;											//���÷�ֵ+������־
			}
			else
			{	
					__IO int val=*(p+pwm_index)*512;	
				
					TIM2_SetPWM_pulse(1,val);				
			}						
	}
}

/****************************************************************************/
////����PWM���ж��ٸ�Ԫ��
//uint16_t POINT_NUM = sizeof(indexWave)/sizeof(indexWave[0]); 
///*��ѹ��ֵ�ȼ���*/
//#define AMPLITUDE_CLASS 1024

////����������ε�Ƶ��
//__IO uint16_t period_class = 1;


///**
//  * @brief  �ص�����
//	*	@note 		��
//  * @retval ��
//  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	static uint16_t pwm_index = 0;			//����PWM���  ��������
//	static uint16_t amplitude_cnt = 0;	//���ڼ����ֵ�ȼ�
//	if(htim==(&TIM_TimeBaseStructure))
//	{
//			amplitude_cnt++;	//����
//				
//			//ÿ��PWM���е�ÿ��Ԫ����AMPLITUDE_CLASS���ȼ���
//		  //ÿ����һ�������һ�����壬��PWM���е�Ԫ�ض�ʹ��һ��
//		  //ʹ��256��
//			if(amplitude_cnt > (511))		 						
//			{		
//		
//					pwm_index++;												//��־PWM��ָ����һ��Ԫ��
//					//��PWM���ѵ����β������ָ���ͷ
//					if( pwm_index >=  POINT_NUM)			
//					{
//							pwm_index=0;								
//					}

//					amplitude_cnt=0;											//���÷�ֵ+������־
//			}
//			else
//			{	
//					//ÿ��PWM���е�ÿ��Ԫ����AMPLITUDE_CLASS���ȼ���
//					//ÿ����һ�������һ�����壬��PWM���е�Ԫ�ض�ʹ��һ��
//					//����RGB��ɫ����ֵ�����ø���ͨ���Ƿ������ǰ��PWM��Ԫ�ر�ʾ������

//					TIM2_SetPWM_pulse(1,indexWave[pwm_index]);//����PWM���޸Ķ�ʱ���ıȽϼĴ���ֵ
//			}						
//	}
//}

/****************************************************************************/

///**
//  * @brief  �ص�����
//	*	@note 		��
//  * @retval ��
//  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	static uint16_t pwm_index = 0;			//����PWM���  ��������
//	static uint16_t period_cnt = 0;			//���ڼ���������
//	static uint16_t amplitude_cnt = 0;	//���ڼ����ֵ�ȼ�
//	if(htim==(&TIM_TimeBaseStructure))
//	{
//			amplitude_cnt++;
//				
//			//ÿ��PWM���е�ÿ��Ԫ����AMPLITUDE_CLASS���ȼ���
//		  //ÿ����һ�������һ�����壬��PWM���е�Ԫ�ض�ʹ��һ��
//		  //ʹ��256��
//			if(amplitude_cnt > (AMPLITUDE_CLASS-1))		 						
//			{		
//					period_cnt++;
//					//ÿ��PWM���е�ÿ��Ԫ��ʹ��period_class��
//					if(period_cnt > period_class)
//					{				
//							pwm_index++;												//��־PWM��ָ����һ��Ԫ��
//							//��PWM���ѵ����β������ָ���ͷ
//							if( pwm_index >=  POINT_NUM)			
//							{
//									pwm_index=0;								
//							}
//							period_cnt = 0;											//�������ڼ�����־
//					}
//					amplitude_cnt=0;											//���÷�ֵ+������־
//			}
//			else
//			{	
//					//ÿ��PWM���е�ÿ��Ԫ����AMPLITUDE_CLASS���ȼ���
//					//ÿ����һ�������һ�����壬��PWM���е�Ԫ�ض�ʹ��һ��
//					//����RGB��ɫ����ֵ�����ø���ͨ���Ƿ������ǰ��PWM��Ԫ�ر�ʾ������

//					TIM2_SetPWM_pulse(1,indexWave[pwm_index]);//����PWM���޸Ķ�ʱ���ıȽϼĴ���ֵ
//			}						
//	}
//}

