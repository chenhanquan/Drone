#ifndef __REMOTE_H__
#define __REMOTE_H__

#include "stm32f4xx.h"

typedef struct
{
	u16 Ch1_Roll;    //ͨ��1
	u16 Ch2_Pitch;   //ͨ��2
	u16 Ch3_Thro;    //ͨ��3
	u16 Ch4_Yaw;     //ͨ��4
	u16 Ch5_SWA;     //ͨ��5
	u16 Ch6_SWB;     //ͨ��6
	u16 Ch7_SWC;     //ͨ��7
	u16 Ch8_SWD;     //ͨ��8
	float fAngelRoll;    //������roll��
	float fAngelPitch;   //������pitch��
	float fAngelYaw;     //������yaw��
	u16 Thro;            //����������
}Remote_Type;


extern u16 PPM_Databuf[10];    //�ⲿ��ppm������������

void vRemoteDataHandle(Remote_Type* xRemoteData);



#endif 

