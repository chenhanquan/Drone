#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx.h"


typedef struct PID
{
	//��Ҫ�޸�
  float Kp;           //��������
  float Ki;           //���ֳ���
  float Kd;           //΢�ֳ���
	float Kd_freq;      //΢�ֻ��ڽ�ֹƵ��
	float Max_Error;    //������
	float IntIimit;     //�����޷�
	
	//����Ҫ�޸�
  float Error;        //ƫ��
  float Integral;     //����
	float Dout;
	float last_Error;
  float Output;
}PID_Type; 

void PID_Normal(PID_Type*PID,const float target,const float measure);
void PID_AlterI(PID_Type*PID,float target,float measure,float limit_A,float limit_B);
void PID_AlterI_PreD(PID_Type*PID,float target,float measure,float last_measure,float limit_A,float limit_B,float gama);

#endif 
