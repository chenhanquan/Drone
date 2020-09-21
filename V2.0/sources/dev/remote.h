#ifndef __REMOTE_H__
#define __REMOTE_H__

#include "stm32f4xx.h"

typedef struct
{
	u16 Ch1_Roll;    //通道1
	u16 Ch2_Pitch;   //通道2
	u16 Ch3_Thro;    //通道3
	u16 Ch4_Yaw;     //通道4
	u16 Ch5_SWA;     //通道5
	u16 Ch6_SWB;     //通道6
	u16 Ch7_SWC;     //通道7
	u16 Ch8_SWD;     //通道8
	float fAngelRoll;    //处理后的roll角
	float fAngelPitch;   //处理后的pitch角
	float fAngelYaw;     //处理后的yaw角
	u16 Thro;            //处理后的油门
}Remote_Type;


extern u16 PPM_Databuf[10];    //外部的ppm储存数据数组

void vRemoteDataHandle(Remote_Type* xRemoteData);



#endif 

