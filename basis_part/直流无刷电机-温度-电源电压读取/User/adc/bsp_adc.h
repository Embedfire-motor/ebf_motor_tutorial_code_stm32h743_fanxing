#ifndef __BSP_ADC_H
#define	__BSP_ADC_H

#include "stm32h7xx.h"

// ADC 序号宏定义
#define ADCx                            ADC3
#define ADC_CLK_ENABLE()                __HAL_RCC_ADC3_CLK_ENABLE()

#define VREF                            3.3f     // 参考电压，理论上是3.3，可通过实际测量得3.258
#define ADC_NUM_MAX                     40       // ADC 转换结果缓冲区最大值

#define GET_ADC_VDC_VAL(val)            ((float)val/(float)65536.0*VREF)          // 得到电压值
  
/*********************** 温度传感器电压采集 ******************/
// ADC GPIO 宏定义
#define TEMP_ADC_GPIO_PORT              GPIOF
#define TEMP_ADC_GPIO_PIN               GPIO_PIN_10
#define TEMP_ADC_GPIO_CLK_ENABLE()      __GPIOF_CLK_ENABLE()

#define TEMP_ADC_CHANNEL                ADC_CHANNEL_6

// ADC DMA 通道宏定义，这里我们使用DMA传输
#define ADC_DMA_CLK_ENABLE()            __DMA2_CLK_ENABLE()
#define ADC_DMA_STREAM                  DMA2_Stream0
#define DMA_REQUEST_ADCx                DMA_REQUEST_ADC3

#define ADC_DMA_IRQ                     DMA2_Stream0_IRQn
#define ADC_DMA_IRQ_Handler             DMA2_Stream0_IRQHandler

/*********************** 电源电压采集 ******************/

#define VBUS_GPIO_PORT                  GPIOF
#define VBUS_GPIO_PIN                   GPIO_PIN_9
#define VBUS_GPIO_CLK_ENABLE()          __GPIOF_CLK_ENABLE()

#define VBUS_ADC_CHANNEL                ADC_CHANNEL_2
#define GET_VBUS_VAL(val)               (((float)val - 1.24f) * 37.0f)      // 测量电压是电源电压的1/37

extern DMA_HandleTypeDef DMA_Init_Handle;
extern ADC_HandleTypeDef ADC_Handle;

void ADC_Init(void);
float get_ntc_v_val(void);
float get_ntc_r_val(void);
float get_ntc_t_val(void);
float get_vbus_val(void);
 
#endif /* __BSP_ADC_H */
