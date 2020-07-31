/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   ʹ�ô���1���ض���c��printf������usart�˿ڣ��жϽ���ģʽ
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32 F407 ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
  
#include "./usart/bsp_debug_usart.h"
#include "./led/bsp_led.h"

UART_HandleTypeDef UartHandle;

 /**
  * @brief  DEBUG_USART GPIO ����,����ģʽ���á�115200 8-N-1
  * @param  ��
  * @retval ��
  */  
void DEBUG_USART_Config(void)
{ 
  
  UartHandle.Instance          = DEBUG_USART;
  
  UartHandle.Init.BaudRate     = DEBUG_USART_BAUDRATE;
  UartHandle.Init.WordLength   = USART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = USART_STOPBITS_1;
  UartHandle.Init.Parity       = USART_PARITY_NONE;
//  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = USART_MODE_TX_RX;
  
  HAL_UART_Init(&UartHandle);
    
  /* ʹ�ܴ��ڿ��ж� */
  __HAL_UART_ENABLE_IT(&UartHandle, USART_IT_RXNE);
  
  HAL_NVIC_SetPriority(DEBUG_USART_IRQ, 0, 0);	// ��ռ���ȼ�0�������ȼ�1
  HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ );		      // ʹ��USART1�ж�ͨ�� 
}


/**
  * @brief UART MSP ��ʼ�� 
  * @param huart: UART handle
  * @retval ��
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  DEBUG_USART_CLK_ENABLE();
	
	DEBUG_USART_RX_GPIO_CLK_ENABLE();
  DEBUG_USART_TX_GPIO_CLK_ENABLE();
  
/**USART1 GPIO Configuration    
  PA9     ------> USART1_TX
  PA10    ------> USART1_RX 
  */
  /* ����Tx����Ϊ���ù���  */
  GPIO_InitStruct.Pin = DEBUG_USART_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = DEBUG_USART_TX_AF;
  HAL_GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStruct);
  
  /* ����Rx����Ϊ���ù��� */
  GPIO_InitStruct.Pin = DEBUG_USART_RX_PIN;
  GPIO_InitStruct.Alternate = DEBUG_USART_RX_AF;
  HAL_GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStruct); 
}

/*****************  �����ַ� **********************/
void Usart_SendByte(uint8_t str)
{
  HAL_UART_Transmit(&UartHandle, &str, 1, 1000);
}

/*****************  �����ַ��� **********************/
void Usart_SendString(uint8_t *str)
{
	unsigned int k=0;
  do 
  {
      HAL_UART_Transmit(&UartHandle,(uint8_t *)(str + k) ,1,1000);
      k++;
  } while(*(str + k)!='\0');
}

///�ض���c�⺯��printf������DEBUG_USART���ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
	/* ����һ���ֽ����ݵ�����DEBUG_USART */
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	
	return (ch);
}

///�ض���c�⺯��scanf������DEBUG_USART����д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		
	int ch;
	HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	return (ch);
}

///**
//  * @brief �������ڽ��յ�������
//  * @param cmd������
//  * @param ch: ����ͨ��
//  * @param data������
//  * @retval ��
//  */
//void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *husart)
//{
//  packet_head_t packet;
//    
//  packet.cmd = UART_RxBuffer[CMD_INDEX_VAL];
//  packet.len  = COMPOUND_32BIT(&UART_RxBuffer[LEN_INDEX_VAL]);     // �ϳɳ���
//  packet.head = COMPOUND_32BIT(&UART_RxBuffer[HEAD_INDEX_VAL]);    // �ϳɰ�ͷ
//  
//  if (packet.head == PACKET_HEAD)    // ����ͷ
//  {
//    /* ��ͷ��ȷ */
//    if (check_sum(0, UART_RxBuffer, packet.len - 1) == UART_RxBuffer[packet.len - 1])    // ���У����Ƿ���ȷ
//    {
//      switch(packet.cmd)
//      {
//        case SET_P_I_D_CMD:
//        {
//          uint32_t temp0 = COMPOUND_32BIT(&UART_RxBuffer[13]);
//          uint32_t temp1 = COMPOUND_32BIT(&UART_RxBuffer[17]);
//          uint32_t temp2 = COMPOUND_32BIT(&UART_RxBuffer[21]);
//          
//          float p_temp, i_temp, d_temp;
//          
//          p_temp = *(float *)&temp0;
//          i_temp = *(float *)&temp1;
//          d_temp = *(float *)&temp2;
//          
//          set_p_i_d(p_temp, i_temp, d_temp);    // ���� P I D
//        }
//        break;

//        case SET_TARGET_CMD:
//        {
//          int actual_temp = COMPOUND_32BIT(&UART_RxBuffer[13]);    // �õ�����
//          
//          set_pid_target(actual_temp);    // ����Ŀ��ֵ
//        }
//        break;
//        
//        case START_CMD:
//        {
//          set_motor_enable();              // �������
//        }
//        break;
//        
//        case STOP_CMD:
//        {
//          set_motor_disable();              // ֹͣ���
//        }
//        break;
//        
//        case RESET_CMD:
//        {
//          HAL_NVIC_SystemReset();          // ��λϵͳ
//        }
//        break;
//        
//        case SET_PERIOD_CMD:
//        {
//          uint32_t temp = COMPOUND_32BIT(&UART_RxBuffer[13]);     // ������
//          SET_BASIC_TIM_PERIOD(temp);                             // ���ö�ʱ������1~1000ms
//        }
//        break;
//      }
//    }
//  }
//}

/*********************************************END OF FILE**********************/
