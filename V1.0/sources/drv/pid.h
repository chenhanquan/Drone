#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx.h"

#define INTEGRAL_APART    //积分分离控制位

typedef struct PID
{
  float Kp;           //比例常数
  float Ki;           //积分常数
  float Kd;           //微分常数
  float Error;        //偏差
  float Integral;     //积分
  float Differ;       //微分
  float PreError;     //上一次偏差
#ifdef INTEGRAL_APART
	float IntRange;     //积分分离范围  
#endif
  float IntIimit;     //积分限幅范围
  uint8_t Ki_Flag;    //积分控制位
  float Output;       //PID总输出
}PID_Type; 

void vPIDCalculate(PID_Type*PID,float target,float measure);

#endif 
