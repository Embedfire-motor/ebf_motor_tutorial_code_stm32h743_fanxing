#ifndef __BSP_ADC_H
#define	__BSP_ADC_H

#include "stm32h7xx.h"

// ADC ��ź궨��
#define CURR_ADC                        ADC1
#define CURR_ADC_CLK_ENABLE()           __HAL_RCC_ADC12_CLK_ENABLE()

#define ADC_VBUS_IRQ                    ADC_IRQn
#define ADC_VBUS_IRQHandler             ADC_IRQHandler

#define VREF                            3.3f     // �ο���ѹ����������3.3����ͨ��ʵ�ʲ�����3.258
#define ADC_NUM_MAX                     1024       // ADC ת��������������ֵ

#define GET_ADC_VDC_VAL(val)            ((float)val/(float)65536.0*VREF)          // �õ���ѹֵ
  
/*********************** �����ɼ� ******************/
// ADC GPIO �궨��
#define CURR_ADC_GPIO_PORT              GPIOB
#define CURR_ADC_GPIO_PIN               GPIO_PIN_1
#define CURR_ADC_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()

#define CURR_ADC_CHANNEL                ADC_CHANNEL_5

// ADC DR�Ĵ����궨�壬ADCת���������ֵ����������
#define CURR_ADC_DR_ADDR                ((uint32_t)ADC1+0x4c)

// ADC DMA ͨ���궨�壬��������ʹ��DMA����
#define CURR_ADC_DMA_CLK_ENABLE()       __DMA2_CLK_ENABLE()
#define CURR_ADC_DMA_CHANNEL            DMA_CHANNEL_0
#define CURR_ADC_DMA_STREAM             DMA2_Stream0

#define ADC_DMA_IRQ                     DMA2_Stream0_IRQn
#define ADC_DMA_IRQ_Handler             DMA2_Stream0_IRQHandler

#define GET_ADC_CURR_VAL(val)           (((float)val)/(float)8.0/(float)0.02*(float)1000.0)          // �õ�����ֵ����ѹ�Ŵ�8����0.02�ǲ������裬��λmA��

/*********************** ��Դ��ѹ�ɼ� ******************/

#define VBUS_GPIO_PORT                  GPIOB
#define VBUS_GPIO_PIN                   GPIO_PIN_0
#define VBUS_GPIO_CLK_ENABLE()          __GPIOB_CLK_ENABLE()

#define VBUS_ADC_CHANNEL                ADC_CHANNEL_9

#define GET_VBUS_VAL(val)               (((float)val-(float)0.5) / (float)8.0 * (float)301.0)

/*********************** ADģ�⿴�Ź����޵�ѹ ******************/
#define VBUS_MAX                        35    // ��ѹ���ֵ
#define VBUS_MIN                        5    // ��ѹ��Сֵ

#define VBUS_HEX_MAX                    ((VBUS_MAX/311.88*8.0+0.5)/VREF*65536)    // ��ѹ���ֵ��������ѹ�ǵ�Դ��ѹ��1/31��31.0
#define VBUS_HEX_MIN                    ((VBUS_MIN/311.88*8.0+0.5)/VREF*65536)    // ��ѹ��Сֵ��������ѹ�ǵ�Դ��ѹ��1/31��31.0

extern DMA_HandleTypeDef DMA_Init_Handle;
extern ADC_HandleTypeDef ADC_Handle;

void ADC_Init(void);
int32_t get_curr_val(void);
float get_vbus_val(void);
 
#endif /* __BSP_ADC_H */
