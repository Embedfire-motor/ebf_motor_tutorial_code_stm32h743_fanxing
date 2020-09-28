#include "./adc/bsp_adc.h"
#include ".\bldcm_control\bsp_bldcm_control.h"
#include "./led/bsp_led.h" 
#include "./usart/bsp_debug_usart.h"

__IO uint16_t ADC_ConvertedValue;
DMA_HandleTypeDef DMA_Init_Handle;
ADC_HandleTypeDef ADC_Handle;

static uint32_t adc_buff[ADC_NUM_MAX];    // 电压采集缓冲区
static uint32_t vbus_adc_mean = 0;         // 电源电压 ACD 采样结果平均值
static uint32_t adc_mean_t = 0;           // 平均值累加

/**
  * @brief  ADC 通道引脚初始化
  * @param  无
  * @retval 无
  */
static void ADC_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  // 使能 GPIO 时钟
  TEMP_ADC_GPIO_CLK_ENABLE();
  VBUS_GPIO_CLK_ENABLE();
  // 配置 IO
  GPIO_InitStructure.Pin = TEMP_ADC_GPIO_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
  GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
  HAL_GPIO_Init(TEMP_ADC_GPIO_PORT, &GPIO_InitStructure);	

  GPIO_InitStructure.Pin = VBUS_GPIO_PIN;
  HAL_GPIO_Init(VBUS_GPIO_PORT, &GPIO_InitStructure);	
}

void adc_dma_init(void)
{
  // ------------------DMA Init 结构体参数 初始化--------------------------
  // 开启DMA时钟
  ADC_DMA_CLK_ENABLE();
  // 数据传输通道
  DMA_Init_Handle.Instance = ADC_DMA_STREAM;
  // 数据传输方向为外设到存储器	
  DMA_Init_Handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
  // 外设寄存器只有一个，地址不用递增
  DMA_Init_Handle.Init.PeriphInc = DMA_PINC_DISABLE;
  // 存储器地址固定
  DMA_Init_Handle.Init.MemInc = DMA_MINC_ENABLE;
  // 外设数据大小为字，即四个字节,H7上的DR寄存器为32位,处理不好数据会异常,用字对应寄存器大小即可
  DMA_Init_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  // 存储器数据大小也为字，跟外设数据大小相同
  DMA_Init_Handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;	
  // 循环传输模式
  DMA_Init_Handle.Init.Mode = DMA_NORMAL;
  // DMA 传输通道优先级为高，当使用一个DMA通道时，优先级设置不影响
  DMA_Init_Handle.Init.Priority = DMA_PRIORITY_HIGH;
  // 禁止DMA FIFO	，使用直连模式
  DMA_Init_Handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;  
  // FIFO 大小，FIFO模式禁止时，这个不用配置
  DMA_Init_Handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
  DMA_Init_Handle.Init.MemBurst = DMA_MBURST_SINGLE;
  DMA_Init_Handle.Init.PeriphBurst = DMA_PBURST_SINGLE;  
  // 选择 DMA 通道，通道存在于流中
  DMA_Init_Handle.Init.Request = DMA_REQUEST_ADCx; 
  //初始化DMA流，流相当于一个大的管道，管道里面有很多通道
  HAL_DMA_Init(&DMA_Init_Handle); 

  __HAL_LINKDMA(&ADC_Handle,DMA_Handle,DMA_Init_Handle);
}

/**
  * @brief  ADC 和 DMA 初始化
  * @param  无
  * @retval 无
  */
static void ADC_Mode_Config(void)
{
  // 开启ADC时钟
  ADC_CLK_ENABLE();
  // -------------------ADC Init 结构体 参数 初始化------------------------
  // ADC1
  ADC_Handle.Instance = ADCx;
  // 时钟为fpclk 4分频,AD时钟60Mhz
  ADC_Handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  // ADC 分辨率
  ADC_Handle.Init.Resolution = ADC_RESOLUTION_16B;
  // 扫描模式	Z
  ADC_Handle.Init.ScanConvMode = ADC_SCAN_ENABLE; 
  // 连续转换	
  ADC_Handle.Init.ContinuousConvMode = ENABLE;
  // 非连续转换	
  ADC_Handle.Init.DiscontinuousConvMode = DISABLE;
  // 非连续转换个数
  ADC_Handle.Init.NbrOfDiscConversion   = 0;
  //禁止外部边沿触发    
  ADC_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  //使用软件触发
  ADC_Handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  //数据左移,不左移
  ADC_Handle.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  //转换通道 2个
  ADC_Handle.Init.NbrOfConversion = 2;
  //转换数据管理:DMA单次传输
  ADC_Handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  //过采样
  ADC_Handle.Init.OversamplingMode = DISABLE;
  //数据溢出,对原数据数据处理方式,覆写
  ADC_Handle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  //转换完成标志
  ADC_Handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;    
  //初始化ADC
  HAL_ADC_Init(&ADC_Handle);

  /* 校准 ADC，采用偏移校准 */
  if (HAL_ADCEx_Calibration_Start(&ADC_Handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    printf("%s,%d\r\n",__FILE__, __LINE__);
    while(1);
  }

  //---------------------------------------------------------------------------
  ADC_ChannelConfTypeDef ADC_Config;
  
  ADC_Config.SamplingTime           = ADC_SAMPLETIME_2CYCLES_5;   // 采样时间间隔	
  ADC_Config.SingleDiff             = ADC_SINGLE_ENDED;           // 采样输入方式,单端输入
  ADC_Config.Offset                 = 0;                          // 采样数据自定义偏移,不偏移
  ADC_Config.OffsetNumber           = ADC_OFFSET_NONE;
  ADC_Config.OffsetRightShift       = DISABLE;
  ADC_Config.OffsetSignedSaturation = DISABLE;

  /** Configure for the selected ADC regular channel its corresponding 
  rank in the sequencer and its sample time. */
  ADC_Config.Channel = TEMP_ADC_CHANNEL;
  ADC_Config.Rank    = ADC_REGULAR_RANK_1;    // 配置 ADC 通道转换顺序为1
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
  
  // 外设中断优先级配置和使能中断配置
  HAL_NVIC_SetPriority(ADC_DMA_IRQ, 1, 1);
  HAL_NVIC_EnableIRQ(ADC_DMA_IRQ);

  HAL_ADC_Start_DMA(&ADC_Handle, (uint32_t*)adc_buff, ADC_NUM_MAX);
}

/**
  * @brief  电流采集初始化
  * @param  无
  * @retval 无
  */
void ADC_Init(void)
{
	ADC_GPIO_Config();
  adc_dma_init();
	ADC_Mode_Config();
}

/**
  * @brief  常规转换在非阻塞模式下完成回调
  * @param  hadc: ADC  句柄.
  * @retval 无
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  uint32_t adc_mean = 0;

  /* 计算温度通道采样的平均值 */
  for(uint32_t count = 0; count < ADC_NUM_MAX; count+=2)
  {
    adc_mean += (int32_t)adc_buff[count];
  }
  
  adc_mean_t = adc_mean / (ADC_NUM_MAX / 2);    // 保存平均值
  
#if 1
  
  adc_mean = 0;
  
  /* 计算电压通道采样的平均值 */
  for(uint32_t count = 1; count < ADC_NUM_MAX; count+=2)
  {
    adc_mean += (int32_t)adc_buff[count];
  }
  
  vbus_adc_mean = adc_mean / (ADC_NUM_MAX / 2);    // 保存平均值
  
#else
  vbus_adc_mean = adc_buff[1];
#endif
  
  HAL_ADC_Start_DMA(&ADC_Handle, (uint32_t*)adc_buff, ADC_NUM_MAX);    // 开始 ADC 采样
}
/**
  * @brief  获取温度传感器端的电压值
  * @param  无
  * @retval 转换得到的电流值
  */
float get_ntc_v_val(void)
{
  float vdc = GET_ADC_VDC_VAL(adc_mean_t);      // 获取电压值
  
  return vdc;
}

/**
  * @brief  获取温度传感器端的电阻值
  * @param  无
  * @retval 转换得到的电流值
  */
float get_ntc_r_val(void)
{
  float r = 0;
  float vdc = get_ntc_v_val();
  
  r = (VREF - vdc) / (vdc / (float)4700.0);
  
  return r;
}

/**
  * @brief  获取温度传感器的温度
  * @param  无
  * @retval 转换得到的温度，单位：（℃）
  */
float get_ntc_t_val(void)
{
  float t = 0;             // 测量温度
  float Rt = 0;            // 测量电阻
  float Ka = 273.15;       // 0℃ 时对应的温度（开尔文）
  float R25 = 10000.0;     // 25℃ 电阻值
  float T25 = Ka + 25;     // 25℃ 时对应的温度（开尔文）
  float B = 3950.0;        /* B-常数：B = ln(R25 / Rt) / (1 / T – 1 / T25)，
                             其中 T = 25 + 273.15 */

  Rt = get_ntc_r_val();    // 获取当前电阻值

  t = B * T25 / (B + log(Rt / R25) * T25) - Ka ;    // 使用公式计算

  return t;
}

/**
  * @brief  获取电源电压值
  * @param  无
  * @retval 转换得到的电流值
  */
float get_vbus_val(void)
{
  float vdc = GET_ADC_VDC_VAL(vbus_adc_mean);      // 获取电压值
  
  return GET_VBUS_VAL(vdc);
}

/*********************************** END OF FILE *********************************************/
