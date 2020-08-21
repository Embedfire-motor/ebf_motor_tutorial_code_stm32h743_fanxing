#include "./adc/bsp_adc.h"
#include ".\bldcm_control\bsp_bldcm_control.h"
#include "./led/bsp_led.h" 
#include "./usart/bsp_debug_usart.h"

__IO uint16_t ADC_ConvertedValue;
DMA_HandleTypeDef DMA_Init_Handle;
ADC_HandleTypeDef ADC_Handle;

static uint32_t adc_buff[ADC_NUM_MAX];    // ��ѹ�ɼ�������
static uint32_t vbus_adc_mean = 0;         // ��Դ��ѹ ACD �������ƽ��ֵ
static uint32_t adc_mean_t = 0;           // ƽ��ֵ�ۼ�

/**
  * @brief  ADC ͨ�����ų�ʼ��
  * @param  ��
  * @retval ��
  */
static void ADC_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  // ʹ�� GPIO ʱ��
  TEMP_ADC_GPIO_CLK_ENABLE();
  VBUS_GPIO_CLK_ENABLE();
  // ���� IO
  GPIO_InitStructure.Pin = TEMP_ADC_GPIO_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
  GPIO_InitStructure.Pull = GPIO_NOPULL ; //������������
  HAL_GPIO_Init(TEMP_ADC_GPIO_PORT, &GPIO_InitStructure);	

  GPIO_InitStructure.Pin = VBUS_GPIO_PIN;
  HAL_GPIO_Init(VBUS_GPIO_PORT, &GPIO_InitStructure);	
}

void adc_dma_init(void)
{
  // ------------------DMA Init �ṹ����� ��ʼ��--------------------------
  // ����DMAʱ��
  ADC_DMA_CLK_ENABLE();
  // ���ݴ���ͨ��
  DMA_Init_Handle.Instance = ADC_DMA_STREAM;
  // ���ݴ��䷽��Ϊ���赽�洢��	
  DMA_Init_Handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
  // ����Ĵ���ֻ��һ������ַ���õ���
  DMA_Init_Handle.Init.PeriphInc = DMA_PINC_DISABLE;
  // �洢����ַ�̶�
  DMA_Init_Handle.Init.MemInc = DMA_MINC_ENABLE;
  // �������ݴ�СΪ�֣����ĸ��ֽ�,H7�ϵ�DR�Ĵ���Ϊ32λ,���������ݻ��쳣,���ֶ�Ӧ�Ĵ�����С����
  DMA_Init_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  // �洢�����ݴ�СҲΪ�֣����������ݴ�С��ͬ
  DMA_Init_Handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;	
  // ѭ������ģʽ
  DMA_Init_Handle.Init.Mode = DMA_NORMAL;
  // DMA ����ͨ�����ȼ�Ϊ�ߣ���ʹ��һ��DMAͨ��ʱ�����ȼ����ò�Ӱ��
  DMA_Init_Handle.Init.Priority = DMA_PRIORITY_HIGH;
  // ��ֹDMA FIFO	��ʹ��ֱ��ģʽ
  DMA_Init_Handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;  
  // FIFO ��С��FIFOģʽ��ֹʱ�������������
  DMA_Init_Handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
  DMA_Init_Handle.Init.MemBurst = DMA_MBURST_SINGLE;
  DMA_Init_Handle.Init.PeriphBurst = DMA_PBURST_SINGLE;  
  // ѡ�� DMA ͨ����ͨ������������
  DMA_Init_Handle.Init.Request = DMA_REQUEST_ADCx; 
  //��ʼ��DMA�������൱��һ����Ĺܵ����ܵ������кܶ�ͨ��
  HAL_DMA_Init(&DMA_Init_Handle); 

  __HAL_LINKDMA(&ADC_Handle,DMA_Handle,DMA_Init_Handle);
}

/**
  * @brief  ADC �� DMA ��ʼ��
  * @param  ��
  * @retval ��
  */
static void ADC_Mode_Config(void)
{
  // ����ADCʱ��
  ADC_CLK_ENABLE();
  // -------------------ADC Init �ṹ�� ���� ��ʼ��------------------------
  // ADC1
  ADC_Handle.Instance = ADCx;
  // ʱ��Ϊfpclk 4��Ƶ,ADʱ��60Mhz
  ADC_Handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  // ADC �ֱ���
  ADC_Handle.Init.Resolution = ADC_RESOLUTION_16B;
  // ɨ��ģʽ	Z
  ADC_Handle.Init.ScanConvMode = ADC_SCAN_ENABLE; 
  // ����ת��	
  ADC_Handle.Init.ContinuousConvMode = ENABLE;
  // ������ת��	
  ADC_Handle.Init.DiscontinuousConvMode = DISABLE;
  // ������ת������
  ADC_Handle.Init.NbrOfDiscConversion   = 0;
  //��ֹ�ⲿ���ش���    
  ADC_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  //ʹ���������
  ADC_Handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  //��������,������
  ADC_Handle.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  //ת��ͨ�� 2��
  ADC_Handle.Init.NbrOfConversion = 2;
  //ת�����ݹ���:DMA���δ���
  ADC_Handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  //������
  ADC_Handle.Init.OversamplingMode = DISABLE;
  //�������,��ԭ�������ݴ���ʽ,��д
  ADC_Handle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  //ת����ɱ�־
  ADC_Handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;    
  //��ʼ��ADC
  HAL_ADC_Init(&ADC_Handle);

  /* У׼ ADC������ƫ��У׼ */
  if (HAL_ADCEx_Calibration_Start(&ADC_Handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    printf("%s,%d\r\n",__FILE__, __LINE__);
    while(1);
  }

  //---------------------------------------------------------------------------
  ADC_ChannelConfTypeDef ADC_Config;
  
  ADC_Config.SamplingTime           = ADC_SAMPLETIME_2CYCLES_5;   // ����ʱ����	
  ADC_Config.SingleDiff             = ADC_SINGLE_ENDED;           // �������뷽ʽ,��������
  ADC_Config.Offset                 = 0;                          // ���������Զ���ƫ��,��ƫ��
  ADC_Config.OffsetNumber           = ADC_OFFSET_NONE;
  ADC_Config.OffsetRightShift       = DISABLE;
  ADC_Config.OffsetSignedSaturation = DISABLE;

  /** Configure for the selected ADC regular channel its corresponding 
  rank in the sequencer and its sample time. */
  ADC_Config.Channel = TEMP_ADC_CHANNEL;
  ADC_Config.Rank    = ADC_REGULAR_RANK_1;    // ���� ADC ͨ��ת��˳��Ϊ1
  if (HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config) != HAL_OK)
  {
    while(1);
  }

  /** Configure for the selected ADC regular channel its corresponding 
  rank in the sequencer and its sample time. */
  ADC_Config.Channel = VBUS_ADC_CHANNEL;
  ADC_Config.Rank    = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config) != HAL_OK)
  {
    while(1);
  }
  
  // �����ж����ȼ����ú�ʹ���ж�����
  HAL_NVIC_SetPriority(ADC_DMA_IRQ, 1, 1);
  HAL_NVIC_EnableIRQ(ADC_DMA_IRQ);

  HAL_ADC_Start_DMA(&ADC_Handle, (uint32_t*)adc_buff, ADC_NUM_MAX);
}

/**
  * @brief  �����ɼ���ʼ��
  * @param  ��
  * @retval ��
  */
void ADC_Init(void)
{
	ADC_GPIO_Config();
  adc_dma_init();
	ADC_Mode_Config();
}

/**
  * @brief  ����ת���ڷ�����ģʽ����ɻص�
  * @param  hadc: ADC  ���.
  * @retval ��
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  uint32_t adc_mean = 0;

  /* �����¶�ͨ��������ƽ��ֵ */
  for(uint32_t count = 0; count < ADC_NUM_MAX; count+=2)
  {
    adc_mean += (int32_t)adc_buff[count];
  }
  
  adc_mean_t = adc_mean / (ADC_NUM_MAX / 2);    // ����ƽ��ֵ
  
#if 1
  
  adc_mean = 0;
  
  /* �����ѹͨ��������ƽ��ֵ */
  for(uint32_t count = 1; count < ADC_NUM_MAX; count+=2)
  {
    adc_mean += (int32_t)adc_buff[count];
  }
  
  vbus_adc_mean = adc_mean / (ADC_NUM_MAX / 2);    // ����ƽ��ֵ
  
#else
  vbus_adc_mean = adc_buff[1];
#endif
  
  HAL_ADC_Start_DMA(&ADC_Handle, (uint32_t*)adc_buff, ADC_NUM_MAX);    // ��ʼ ADC ����
}
/**
  * @brief  ��ȡ�¶ȴ������˵ĵ�ѹֵ
  * @param  ��
  * @retval ת���õ��ĵ���ֵ
  */
float get_ntc_v_val(void)
{
  float vdc = GET_ADC_VDC_VAL(adc_mean_t);      // ��ȡ��ѹֵ
  
  return vdc;
}

/**
  * @brief  ��ȡ�¶ȴ������˵ĵ���ֵ
  * @param  ��
  * @retval ת���õ��ĵ���ֵ
  */
float get_ntc_r_val(void)
{
  float r = 0;
  float vdc = get_ntc_v_val();
  
  r = (VREF - vdc) / (vdc / (float)4700.0);
  
  return r;
}

/**
  * @brief  ��ȡ�¶ȴ��������¶�
  * @param  ��
  * @retval ת���õ����¶ȣ���λ�����棩
  */
float get_ntc_t_val(void)
{
  float t = 0;             // �����¶�
  float Rt = 0;            // ��������
  float Ka = 273.15;       // 0�� ʱ��Ӧ���¶ȣ������ģ�
  float R25 = 10000.0;     // 25�� ����ֵ
  float T25 = Ka + 25;     // 25�� ʱ��Ӧ���¶ȣ������ģ�
  float B = 3950.0;        /* B-������B = ln(R25 / Rt) / (1 / T �C 1 / T25)��
                             ���� T = 25 + 273.15 */

  Rt = get_ntc_r_val();    // ��ȡ��ǰ����ֵ

  t = B * T25 / (B + log(Rt / R25) * T25) - Ka ;    // ʹ�ù�ʽ����

  return t;
}

/**
  * @brief  ��ȡ��Դ��ѹֵ
  * @param  ��
  * @retval ת���õ��ĵ���ֵ
  */
float get_vbus_val(void)
{
  float vdc = GET_ADC_VDC_VAL(vbus_adc_mean);      // ��ȡ��ѹֵ
  
  return GET_VBUS_VAL(vdc);
}

/*********************************** END OF FILE *********************************************/
