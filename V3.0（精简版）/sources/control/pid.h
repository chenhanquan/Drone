#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx.h"


typedef struct PID
{
	//需要修改
  float Kp;           //比例常数
  float Ki;           //积分常数
  float Kd;           //微分常数
	float Kd_freq;      //微分环节截止频率
	float Max_Error;    //最大误差
	float IntIimit;     //积分限幅
	
	//不需要修改
  float Error;        //偏差
  float Integral;     //积分
	float Dout;
	float last_Error;
  float Output;
}PID_Type; 

void PID_Normal(PID_Type*PID,const float target,const float measure);
void PID_AlterI(PID_Type*PID,float target,float measure,float limit_A,float limit_B);
void PID_AlterI_PreD(PID_Type*PID,float target,float measure,float last_measure,float limit_A,float limit_B,float gama);

#endif 
