#ifndef __BSP_BLDCM_CONTROL_H
#define	__BSP_BLDCM_CONTROL_H

#include "stm32h7xx.h"
#include "./tim/bsp_motor_tim.h"
#include "main.h"

//���Ŷ���
/*******************************************************/
// ����������� SD ��
#define SHUTDOWN_PIN                  GPIO_PIN_6
#define SHUTDOWN_GPIO_PORT            GPIOE
#define SHUTDOWN_GPIO_CLK_ENABLE()    __GPIOE_CLK_ENABLE()
/*******************************************************/

/* ��� SD or EN ʹ�ܽ� */
#define BLDCM_ENABLE_SD()                     HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT, SHUTDOWN_PIN, GPIO_PIN_SET)      // �ߵ�ƽ��-�ߵ�ƽʹ�� 
#define BLDCM_DISABLE_SD()                    HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT, SHUTDOWN_PIN, GPIO_PIN_RESET)    // �͵�ƽ�ض�-�͵�ƽ����

/* ����������ö�� */
typedef enum
{
  MOTOR_FWD = 0,
  MOTOR_REV,
}motor_dir_t;

typedef struct
{
  motor_dir_t direction;    // �������
  uint16_t dutyfactor;      // PWM ���ռ�ձ�
  uint8_t is_enable;        // ʹ�ܵ��
  uint32_t lock_timeout;    // �����ת��ʱ
}bldcm_data_t;

void bldcm_init(void);
void set_bldcm_speed(uint16_t v);
void set_bldcm_direction(motor_dir_t dir);
motor_dir_t get_bldcm_direction(void);
void set_bldcm_enable(void);
void set_bldcm_disable(void);

#endif /* __BSP_BLDCM_CONTROL_H */

