#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx.h"

#define INTEGRAL_APART    //���ַ������λ

typedef struct PID
{
  float Kp;           //��������
  float Ki;           //���ֳ���
  float Kd;           //΢�ֳ���
  float Error;        //ƫ��
  float Integral;     //����
  float Differ;       //΢��
  float PreError;     //��һ��ƫ��
#ifdef INTEGRAL_APART
	float IntRange;     //���ַ��뷶Χ  
#endif
  float IntIimit;     //�����޷���Χ
  uint8_t Ki_Flag;    //���ֿ���λ
  float Output;       //PID�����
}PID_Type; 

void vPIDCalculate(PID_Type*PID,float target,float measure);

#endif 
