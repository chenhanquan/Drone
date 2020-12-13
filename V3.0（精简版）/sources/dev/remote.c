#include "remote.h"
#include "math.h"

#define REMOTE_CHANNELS  8
#define YAWINT_NUM 0.05               //偏航积分系数

#define CONTROL_ANGEL_MAX   35
#define CONTROL_YAW_MAX     100
#define RC_Max 				2000
#define RC_Min 				1000
#define RC_Middle 		1500
#define RC_Half       500.0f
#define RC_Rang 			(RC_Max-RC_Min)*0.1  //遥控器的量程
#define RC_Deadband_Top		RC_Middle+RC_Rang*0.5        //死区上限
#define RC_Deadband_Buttom		RC_Middle-RC_Rang*0.5        //死区下限  整个死区范围 +—50

extern u16 PPM_Databuf[10];    //外部的ppm储存数据数组

static void RemoteFilter(Remote_Type* xRemoteData,const u16 *ppm_data);    //遥控滤波

void vRemoteDataHandle(Remote_Type* xRemoteData)
{
	RemoteFilter(xRemoteData,PPM_Databuf);
//	if(xRemoteData->Ch1_Roll>RC_Deadband_Buttom&&xRemoteData->Ch1_Roll<RC_Deadband_Top) xRemoteData->Ch1_Roll=RC_Middle;
//	if(xRemoteData->Ch2_Pitch>RC_Deadband_Buttom&&xRemoteData->Ch2_Pitch<RC_Deadband_Top) xRemoteData->Ch2_Pitch=RC_Middle;
//	if(xRemoteData->Ch4_Yaw>RC_Deadband_Buttom&&xRemoteData->Ch4_Yaw<RC_Deadband_Top) xRemoteData->Ch4_Yaw=RC_Middle;
	
	if(xRemoteData->Ch1_Roll>500)
		{
	//将角的范围规范到+-30度
	xRemoteData->fAngelRoll = (xRemoteData->Ch1_Roll-RC_Middle)/RC_Half*CONTROL_ANGEL_MAX;
	xRemoteData->fAngelPitch = (xRemoteData->Ch2_Pitch-RC_Middle)/RC_Half*CONTROL_ANGEL_MAX;    
	xRemoteData->fAngelYaw = (xRemoteData->Ch4_Yaw-RC_Middle)/RC_Half*CONTROL_ANGEL_MAX;
	
	if(fabs(xRemoteData->fAngelYaw)>5){xRemoteData->YawInt+=YAWINT_NUM*xRemoteData->fAngelYaw;}
	if(fabs(xRemoteData->YawInt)>=360){xRemoteData->YawInt=0;}
	
	//将油门范围变为0-1000
	xRemoteData->Thro =xRemoteData->Ch3_Thro-1000;
	}
}

static void RemoteFilter(Remote_Type* xRemoteData,const u16 *ppm_data)
{
	static u16 filter_buf[REMOTE_CHANNELS][3]={0};
	static u8 count=0;
	for(u8 i=0;i<REMOTE_CHANNELS;i++){filter_buf[i][count]=PPM_Databuf[i];}
	for(u8 i=0;i<REMOTE_CHANNELS;i++)
	{
		if(filter_buf[i][0]==filter_buf[i][1] || filter_buf[i][0]==filter_buf[i][2]){*(&xRemoteData->Ch1_Roll+i)=filter_buf[i][0];}
		else if(filter_buf[i][1]==filter_buf[i][2]){*(&xRemoteData->Ch1_Roll+i)=filter_buf[i][1];}
		else
		{
			if(filter_buf[i][0]>filter_buf[i][1]){*(&xRemoteData->Ch1_Roll+i)=filter_buf[i][0];}
			else{*(&xRemoteData->Ch1_Roll+i)=filter_buf[i][1];}
		}
	}
	count++;
	if(count==3)count=0;
	
}
