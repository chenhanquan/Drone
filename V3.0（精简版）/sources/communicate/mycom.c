#include "mycom.h"
#include "USART.h"
#include "stdio.h"
#include "stdlib.h"

//定义数据拆分
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	

void MyCommunication(u16 *data,u8 len)
{
	u8 senddata[8]={0};
	//senddata = (u8*)malloc(sizeof(u8)*(len*2+2));    //动态申请一个数组
	senddata[0]=0xbb;
	for(u8 i=0;i<len;i++)
	{
		senddata[2*i+1]=BYTE1(*(data+i));
		senddata[2*i+2]=BYTE0(*(data+i));
	}
	senddata[7]=0xbb;
	USART1_Send(senddata,8);
}