#include "remote.h"

#define REMOTE_CHANNELS  8

#define CONTROL_ANGEL_MAX   40
#define CONTROL_YAW_MAX     100
#define RC_Max 				2000
#define RC_Min 				1000
#define RC_Middle 		1500
#define RC_Half       500.0f
#define RC_Rang 			(RC_Max-RC_Min)*0.1  //遥控器的量程
#define RC_Deadband_Top		RC_Middle+RC_Rang*0.5        //死区上限
#define RC_Deadband_Buttom		RC_Middle-RC_Rang*0.5        //死区下限  整个死区范围 +—50

void vRemoteDataHandle(Remote_Type* xRemoteData)
{
	u16 *temp;
	temp = &(xRemoteData->Ch1_Roll);
	xRemoteData->Ch1_Roll = PPM_Databuf[0];
	for(u8 i=0;i<REMOTE_CHANNELS;i++)
	{
		*(temp+i)=PPM_Databuf[i];
		if((*(temp+i))<RC_Min) (*(temp+i))=RC_Min;
		if((*(temp+i))>RC_Max) (*(temp+i))=RC_Max;
	}
	if(xRemoteData->Ch1_Roll>RC_Deadband_Buttom&&xRemoteData->Ch1_Roll<RC_Deadband_Top) xRemoteData->Ch1_Roll=RC_Middle;
	if(xRemoteData->Ch2_Pitch>RC_Deadband_Buttom&&xRemoteData->Ch2_Pitch<RC_Deadband_Top) xRemoteData->Ch2_Pitch=RC_Middle;
	if(xRemoteData->Ch4_Yaw>RC_Deadband_Buttom&&xRemoteData->Ch4_Yaw<RC_Deadband_Top) xRemoteData->Ch4_Yaw=RC_Middle;
	
	//将角的范围规范到+-40度
	xRemoteData->fAngelRoll = (xRemoteData->Ch1_Roll-RC_Middle)/RC_Half*CONTROL_ANGEL_MAX;
	xRemoteData->fAngelPitch = -(xRemoteData->Ch2_Pitch-RC_Middle)/RC_Half*CONTROL_ANGEL_MAX;    //这里取一个反，因为遥控方向和俯仰角定义相反
	xRemoteData->fAngelYaw = (xRemoteData->Ch4_Yaw-RC_Middle)/RC_Half*CONTROL_ANGEL_MAX;
	
	//将油门范围变为0-1000
	xRemoteData->Thro = xRemoteData->Ch3_Thro-RC_Min;
}
