#include "./pid/bsp_pid.h"

//����ȫ�ֱ���

_pid pid;

/**
  * @brief  PID������ʼ��
	*	@note 	��
  * @retval ��
  */
void PID_param_init()
{
		/* ��ʼ������ */
    printf("PID_init begin \n");
    pid.target_val=0.0;				
    pid.actual_val=0.0;
    pid.err=0.0;
    pid.err_last=0.0;
    pid.integral=0.0;
//		pid.Kp = 0.31;
//		pid.Ki = 0.070;
//		pid.Kd = 0.3;


		pid.Kp = 0.01;//24
		pid.Ki = 0.80;
		pid.Kd = 0.04;
	
    printf("PID_init end \n");

}
/**
  * @brief  PID�㷨ʵ��
  * @param  val		Ŀ��ֵ
	*	@note 	��
  * @retval ͨ��PID���������
  */
float PID_realize(float temp_val)
{
		/*����Ŀ��ֵ*/
    pid.target_val=temp_val;
		/*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid.err=pid.target_val-pid.actual_val;
		/*����ۻ�*/
    pid.integral+=pid.err;
		/*PID�㷨ʵ��*/
    pid.actual_val=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
		/*����*/
    pid.err_last=pid.err;
		/*���ص�ǰʵ��ֵ*/
    return pid.actual_val;
}
/**
  * @brief  ��ʱ�����ڵ��ú���
  * @param  ��
	*	@note 	��
  * @retval ��
  */
//һֱ����
//void time_period_fun()
//{
//	float set_point=200.0;
//	float val=PID_realize(set_point);
//	printf("val,%f;act,%f\n",set_point,val);
//}
//���������ȶ���ֹͣ
void time_period_fun()
{
	static int flag=0;
	static int num=0;
	static int run_i=0;
		
	float set_point=200.0;
	if(!flag)
	{
		float val=PID_realize(set_point);
		printf("val,%f;act,%f\n",set_point,val);	
		run_i++;
		if(abs(val-set_point)<=1)
		{
			num++;
		}
		else//����������������
		{
			num=0;
		}
		if(num>20)//�ȶ�����
		{
			printf("PID�㷨����%d �κ��ȶ�\r\n",run_i);
			flag=1;
		}
	}
}




