#include "./adc/bsp_adc.h"
#include ".\motor_control\bsp_motor_control.h"
#include "./led/bsp_led.h" 
#include "./usart/bsp_debug_usart.h"

__IO uint16_t ADC_ConvertedValue;
DMA_HandleTypeDef DMA_Init_Handle;
ADC_HandleTypeDef ADC_Handle;

static uint32_t adc_buff[ADC_NUM_MAX];    // 电压采集缓冲区
static uint32_t vbus_adc_mean = 0;        // 电源电压 ACD 采样结果平均值
static uint32_t adc_mean_sum = 0;        // 平均值累加
static uint32_t adc_mean_count = 0;      // 累加计数

/**
  * @brief  ADC 通道引脚初始化
  * @param  无
  * @retval 无
  */
static void ADC_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // 使能 GPIO 时钟
    CURR_ADC_GPIO_CLK_ENABLE();
    VBUS_GPIO_CLK_ENABLE();
    // 配置 IO
    GPIO_InitStructure.Pin = CURR_ADC_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;	    
    GPIO_InitStructure.Pull = GPIO_NOPULL ; //不上拉不下拉
    HAL_GPIO_Init(CURR_ADC_GPIO_PORT, &GPIO_InitStructure);	

    GPIO_InitStructure.Pin = VBUS_GPIO_PIN;
    HAL_GPIO_Init(VBUS_GPIO_PORT, &GPIO_InitStructure);	
}

void adc_dma_init(void)
{
    // ------------------DMA Init 结构体参数 初始化--------------------------
    // ADC1使用DMA2，数据流0，通道0，这个是手册固定死的
    // 开启DMA时钟
    CURR_ADC_DMA_CLK_ENABLE();
    // 数据传输通道
    DMA_Init_Handle.Instance = CURR_ADC_DMA_STREAM;
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
    DMA_Init_Handle.Init.Mode = DMA_CIRCULAR;
    // DMA 传输通道优先级为高，当使用一个DMA通道时，优先级设置不影响
    DMA_Init_Handle.Init.Priority = DMA_PRIORITY_HIGH;
    // 禁止DMA FIFO	，使用直连模式
    DMA_Init_Handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;  
    // FIFO 大小，FIFO模式禁止时，这个不用配置
    DMA_Init_Handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    DMA_Init_Handle.Init.MemBurst = DMA_MBURST_SINGLE;
    DMA_Init_Handle.Init.PeriphBurst = DMA_PBURST_SINGLE;  
    // 选择 DMA 通道，通道存在于流中
    DMA_Init_Handle.Init.Request = DMA_REQUEST_ADC1; 
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
    CURR_ADC_CLK_ENABLE();
    // -------------------ADC Init 结构体 参数 初始化------------------------
    // ADC1
    ADC_Handle.Instance = CURR_ADC;
    // 时钟为fpclk 4分频	
    ADC_Handle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
    // ADC 分辨率
    ADC_Handle.Init.Resolution = ADC_RESOLUTION_16B;
    // 禁止扫描模式，多通道采集才需要	
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
		
    /** 配置AD模拟看门狗
    */
    ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
    /* 使用AD模拟看门狗一 */
		AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
		/* 只监测一个规则通道 */
    AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
		/* 阈值上限 */
    AnalogWDGConfig.HighThreshold = VBUS_HEX_MAX;
		/* 阈值下限 */
    AnalogWDGConfig.LowThreshold = VBUS_HEX_MIN;
		/* 检测通道 */
    AnalogWDGConfig.Channel = VBUS_ADC_CHANNEL;
		/* 使能模拟看门狗中断,溢出会产生中断回调 */
    AnalogWDGConfig.ITMode = ENABLE;
    if (HAL_ADC_AnalogWDGConfig(&ADC_Handle, &AnalogWDGConfig) != HAL_OK)
    {
      while(1);
    }
		
    //---------------------------------------------------------------------------
    ADC_ChannelConfTypeDef ADC_Config;
    
    ADC_Config.Channel      = CURR_ADC_CHANNEL;
    // 配置 ADC 通道转换顺序为1，第一个转换，采样时间为3个时钟周期
    ADC_Config.Rank         = ADC_REGULAR_RANK_1;
    // 采样时间间隔	
    ADC_Config.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
		// 采样输入方式,单端输入
		ADC_Config.SingleDiff = ADC_SINGLE_ENDED;
		// 采样数据自定义偏移,不偏移
    ADC_Config.Offset       = 0;
		ADC_Config.OffsetNumber = ADC_OFFSET_NONE;
		ADC_Config.OffsetRightShift = DISABLE;
		ADC_Config.OffsetSignedSaturation = DISABLE;

    HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config);
    
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
    ADC_Config.Channel = VBUS_ADC_CHANNEL;
    ADC_Config.Rank = ADC_REGULAR_RANK_2;
		ADC_Config.SingleDiff = ADC_SINGLE_ENDED;
    // 采样时间间隔	
    ADC_Config.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    ADC_Config.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&ADC_Handle, &ADC_Config) != HAL_OK)
    {
      while(1);
    }
		
    // 外设中断优先级配置和使能中断配置
    HAL_NVIC_SetPriority(ADC_DMA_IRQ, 1, 1);
    HAL_NVIC_EnableIRQ(ADC_DMA_IRQ);
		
		HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(ADC_IRQn);
		
    HAL_ADC_Start_DMA(&ADC_Handle, (uint32_t*)&adc_buff, ADC_NUM_MAX);
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
  int32_t adc_mean = 0;

//  HAL_ADC_Stop_DMA(hadc);       // 停止 ADC 采样，处理完一次数据在继续采样,使用DMA单次模式不停止

  /* 计算电流通道采样的平均值 */
  for(uint32_t count = 0; count < ADC_NUM_MAX; count+=2)
  {
    adc_mean += adc_buff[count];
  }
  
  adc_mean_sum += adc_mean / (ADC_NUM_MAX / 2);    // 累加电压
  adc_mean_count++;
  
#if 1
  
  adc_mean = 0;
  
  /* 计算电压通道采样的平均值 */
  for(uint32_t count = 1; count < ADC_NUM_MAX; count+=2)
  {
    adc_mean += adc_buff[count];
  }
  
  vbus_adc_mean = adc_mean / (ADC_NUM_MAX / 2);    // 保存平均值

#else
  vbus_adc_mean = adc_buff[1];
#endif
  HAL_ADC_Start_DMA(&ADC_Handle, (uint32_t*)&adc_buff, ADC_NUM_MAX);    // 重新开始 ADC 采样

}

static uint16_t flag_num = 0;
/**
  * @brief  在非阻塞模式模拟看门狗回调
  * @param  hadc: ADC  句柄.
  * @retval 无
  */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
	float temp_adc;
	
  flag_num++;     // 电源电压超过阈值电压
	
  temp_adc = get_vbus_val();
	
  if (temp_adc > VBUS_MIN && temp_adc < VBUS_MAX)
    flag_num = 0;
  
  if (flag_num > ADC_NUM_MAX)      // 电源电压超过阈值电压10次
  {
    set_motor_disable();
    flag_num = 0;
    LED1_ON;
    printf("电源电压超过限制！请检查原因，复位开发板在试！\r\n");
    while(1);
  }
}

/**
  * @brief  获取电流值
  * @param  无
  * @retval 转换得到的电流值
  */
int32_t get_curr_val(void)
{
  static uint8_t flag = 0;
  static uint32_t adc_offset = 0;    // 偏置电压
  int16_t curr_adc_mean = 0;         // 电流 ACD 采样结果平均值
  
  curr_adc_mean = adc_mean_sum / adc_mean_count;    // 保存平均值

	adc_mean_count = 0;
	adc_mean_sum = 0;
	
	if (flag < 10 && is_motor_en ==0)	//	仅在电机未启动时记录
	{
		adc_offset = curr_adc_mean;    // 多次记录偏置电压，待系统稳定偏置电压才为有效值
		flag += 1;
	}
	if(curr_adc_mean>=adc_offset)
	{
		curr_adc_mean -= adc_offset;                     // 减去偏置电压
	}else
	{
		curr_adc_mean=0;
	}


  float vdc = GET_ADC_VDC_VAL(curr_adc_mean);      // 获取电压值
  
  return GET_ADC_CURR_VAL(vdc);
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
