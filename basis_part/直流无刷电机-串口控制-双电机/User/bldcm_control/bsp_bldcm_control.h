#ifndef __BSP_BLDCM_CONTROL_H
#define	__BSP_BLDCM_CONTROL_H

#include "stm32h7xx.h"
#include "./tim/bsp_motor_tim.h"
#include "main.h"

//���Ŷ���
/*******************************************************/
// ����������� SD ��
#define MOTOR1_SHUTDOWN_PIN                  GPIO_PIN_12
#define MOTOR1_SHUTDOWN_GPIO_PORT            GPIOG
#define MOTOR1_SHUTDOWN_GPIO_CLK_ENABLE()    __GPIOG_CLK_ENABLE()

#define MOTOR2_SHUTDOWN_PIN                  GPIO_PIN_6
#define MOTOR2_SHUTDOWN_GPIO_PORT            GPIOE
#define MOTOR2_SHUTDOWN_GPIO_CLK_ENABLE()    __GPIOE_CLK_ENABLE()
/*******************************************************/

/* ��� SD or EN ʹ�ܽ� */
#define MOTOR1_BLDCM_ENABLE_SD()                     HAL_GPIO_WritePin(MOTOR1_SHUTDOWN_GPIO_PORT, MOTOR1_SHUTDOWN_PIN, GPIO_PIN_SET)      // �ߵ�ƽ��-�ߵ�ƽʹ�� 
#define MOTOR1_BLDCM_DISABLE_SD()                    HAL_GPIO_WritePin(MOTOR1_SHUTDOWN_GPIO_PORT, MOTOR1_SHUTDOWN_PIN, GPIO_PIN_RESET)    // �͵�ƽ�ض�-�͵�ƽ����

#define MOTOR2_BLDCM_ENABLE_SD()                     HAL_GPIO_WritePin(MOTOR2_SHUTDOWN_GPIO_PORT, MOTOR2_SHUTDOWN_PIN, GPIO_PIN_SET)      // �ߵ�ƽ��-�ߵ�ƽʹ�� 
#define MOTOR2_BLDCM_DISABLE_SD()                    HAL_GPIO_WritePin(MOTOR2_SHUTDOWN_GPIO_PORT, MOTOR2_SHUTDOWN_PIN, GPIO_PIN_RESET)    // �͵�ƽ�ض�-�͵�ƽ����
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

void set_motor1_bldcm_speed(uint16_t v);
void set_motor1_bldcm_direction(motor_dir_t dir);
motor_dir_t get_motor1_bldcm_direction(void);
void set_motor1_bldcm_enable(void);
void set_motor1_bldcm_disable(void);

void set_motor2_bldcm_speed(uint16_t v);
void set_motor2_bldcm_direction(motor_dir_t dir);
motor_dir_t get_motor2_bldcm_direction(void);
void set_motor2_bldcm_enable(void);
void set_motor2_bldcm_disable(void);

#endif /* __BSP_BLDCM_CONTROL_H */

