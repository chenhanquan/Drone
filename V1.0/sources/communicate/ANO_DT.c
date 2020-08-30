/**
  ******************************************************************************
  * @file    ANO_DT.c
  * @author  CHEN HANQUAN
  * @version 
  * @date    19-August-2020
  * @brief   ʹ�������ɿ�V7Э��
  *
  ******************************************************************************
  */
#include "ANO_DT.h"
#include "usart.h"
#include "stdlib.h"

#include "mpu6050.h"
#include "filter.h"
#include "remote.h"

#define DRONE_ADDR  0xFF
#define COMPUTER_ADDR  0xFA
#define TASK_NUM 6    //������������

 extern MPU6050Data_TypeDef MPUData;
 extern FilterData_Type FilterData;
 extern Remote_Type xRemote;
 
 extern u16 uMotor1PWM,uMotor2PWM,uMotor3PWM,uMotor4PWM;

u8 uReceiveData[20]={0};    //δ�޸ĺ�

/////////////////////////////////////////////////////////////////////////////////////
//�������ݲ��
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	

static void vSendDataPort(const unsigned char *DateToSend,int len);                //���ͽӿ�
static void vSendOneFrame(const u8 address,const u8 id ,const u8 len,u8 *data);    //����һ֡
static void vDataAnalyze(u8 *uAnaData);    //���ݽ��շ���
static void vCMDHandle(u8 *uAnaData);      //�������

/**
 * @brief  ���ݷ�����ѯ����
 * @param  None
 * @return None
 */
void vSendDataPolling()
{
	static u8 uDataToSend[20]={0};
	static u32 uTaskLastTime[TASK_NUM]={0};    //��¼ÿ��������һ��ִ�е�ʱ��
	u8 uTaskDuringTime[TASK_NUM]={10,10,10,10,10,10};      //ÿ������ķ��ͼ������
	for(u8 i=0;i<TASK_NUM;i++)                //��ѯ����
 	{
		if((uSystickCount-uTaskLastTime[i])<uTaskDuringTime[i]) continue;    //�ж������Ƿ񵽷���ʱ��
		uTaskLastTime[i]=uSystickCount;    //��¼����ִ�е�ʱ���
		switch(i)
		{
			case 0:    //����1������ԭʼ���ٶȺͼ��ٶ�����
			{
				float temp[6]={0};
				vs16 temp1;
				for(u8 i=0;i<3;i++){temp[i]=MPUData.acc[i];temp[i+3]=MPUData.gyro[i];}
				for(u8 i=0;i<6;i++)
				{
					temp1=(s16)temp[i];
					uDataToSend[2*i]=BYTE1(temp1);
					uDataToSend[2*i+1]=BYTE0(temp1);
				}				
				uDataToSend[12]=0;
				vSendOneFrame(DRONE_ADDR, SEND_ID_SENSOR_IMU,13,uDataToSend);
			}break;
			case 1:
			{
				double temp[3]={0};
				vs16 data_temp;
				temp[0]=FilterData.roll*1.7452;    //100/57.2 ���͵�Ϊ��������
				temp[1]=FilterData.pitch*1.7452;
				temp[2]=FilterData.yaw*1.7452;
				data_temp = (s16)temp[0];
				uDataToSend[0]= BYTE1(data_temp);
				uDataToSend[1]= BYTE0(data_temp);
				data_temp = (s16)temp[1];
				uDataToSend[2]= BYTE1(data_temp);
				uDataToSend[3]= BYTE0(data_temp);
				data_temp = (s16)temp[2];
				uDataToSend[4]= BYTE1(data_temp);
				uDataToSend[5]= BYTE0(data_temp);
				
				uDataToSend[6]= 0;
				vSendOneFrame(DRONE_ADDR, SEND_ID_EULER,7,uDataToSend);
			}break;
			case 2:
			{
				float temp[4]={0};
				vs16 data_temp;
				for(u8 i=0;i<4;i++){temp[i]=FilterData.quad[i]*10000;}
				data_temp = (s16)temp[0];
				uDataToSend[0]= BYTE1(data_temp);
				uDataToSend[1]= BYTE0(data_temp);
				data_temp = (s16)temp[1];
				uDataToSend[2]= BYTE1(data_temp);
				uDataToSend[3]= BYTE0(data_temp);
				data_temp = (s16)temp[2];
				uDataToSend[4]= BYTE1(data_temp);
				uDataToSend[5]= BYTE0(data_temp);
				data_temp = (s16)temp[3];
				uDataToSend[6]= BYTE1(data_temp);
				uDataToSend[7]= BYTE0(data_temp);
				
				uDataToSend[8]= 0;
				vSendOneFrame(DRONE_ADDR, SEND_ID_QUAD,9,uDataToSend);
				}break;
			case 3:
			{
				float temp[6]={0};
				vs16 temp1;
				for(u8 i=0;i<3;i++){temp[i]=MPUData.acchandle[i];temp[i+3]=MPUData.gyrohandle[i];}
				for(u8 i=0;i<6;i++)
				{
					temp1=(s16)temp[i];
					uDataToSend[2*i]=BYTE1(temp1);
					uDataToSend[2*i+1]=BYTE0(temp1);
				}	
				uDataToSend[12]=0;
				vSendOneFrame(DRONE_ADDR,SEND_ID_USERDATA1,13,uDataToSend);
			}break;
			case 4:
			{
				u16 *temp;
				int16_t temp1[10];
				temp=&(xRemote.Ch1_Roll);
				for(u8 i=0;i<8;i++)
				{
					temp1[i]=*(temp+i);
				}
				for(u8 i=8;i<10;i++)
				{
					temp1[i]=0;
				}
				for(u8 i=0;i<20;i+=2)
				{
					uDataToSend[i]= BYTE1(temp1[i/2]);
					uDataToSend[i+1]= BYTE0(temp1[i/2]);
				}
				
				vSendOneFrame(DRONE_ADDR,RECEIVE_ID_REMOTE,20,uDataToSend);
			}break;
			case 5:    //����1
			{
				uDataToSend[0]=BYTE1(uMotor1PWM);
				uDataToSend[1]=BYTE0(uMotor1PWM);
				uDataToSend[2]=BYTE1(uMotor2PWM);
				uDataToSend[3]=BYTE0(uMotor2PWM);
				uDataToSend[4]=BYTE1(uMotor3PWM);
				uDataToSend[5]=BYTE0(uMotor3PWM);
				uDataToSend[6]=BYTE1(uMotor4PWM);
				uDataToSend[7]=BYTE0(uMotor4PWM);
				
				vSendOneFrame(DRONE_ADDR,SEND_ID_CONTROL_PWM,8,uDataToSend);
			}break;
			default: break;
		}
	}
}

u8 vReceiveData(u8 data)    //������û����ɣ��˺�����Ч
{
	static u8 Flag = 0;
	u8 sumcheck=0;
	u8 addcheck=0;
	u8 error=0;
	if(data==0xAA&&Flag==0) Flag=1;uReceiveData[0]=data;return error;
	if(Flag==1){if(data==DRONE_ADDR){Flag=2;uReceiveData[1]=data;return error;}else{Flag=0;error=1;return error;}}
	if(Flag==2)
	{
		if(data==0xE0|data==0xE1|data==0xE2)
		{
			uReceiveData[2]=data;
			Flag=3;
			return error;
		}
		else{Flag=0;error=1;return error;}
	}
	if(Flag==3) uReceiveData[3]=data;Flag=4;return error;
	if(Flag==4)
	{
		static u8 count=0;
		if(count<uReceiveData[3])
		{
			uReceiveData[4+count]=data;
			count++;
			return error;
		}
		else{
			count=0;
			Flag=5;
		}
	}
	if(Flag==5) uReceiveData[uReceiveData[3]+4]=data;Flag=6;return error;
	if(Flag==6)
	{
		uReceiveData[uReceiveData[3]+5]=data;
		for(u8 i=0;i<(uReceiveData[3]+4);i++)
		{
			sumcheck +=uReceiveData[i];
			addcheck +=sumcheck;
		}
		if(sumcheck==uReceiveData[uReceiveData[3]+4]&&addcheck==uReceiveData[uReceiveData[3]+5])
		{
			u8 checkdata[3];
			checkdata[0]=uReceiveData[2];
			checkdata[1]=uReceiveData[uReceiveData[3]+4];
			checkdata[2]=uReceiveData[uReceiveData[3]+5];
			vSendOneFrame(DRONE_ADDR,SEND_ID_CHECK ,3,checkdata);
			Flag=7;
		}
		else{Flag=0;error=1;return error;}
	}
	if(Flag==7) vDataAnalyze(uReceiveData);Flag=0;return error;
}

static void vDataAnalyze(u8 *uAnaData)    //δ���
{
	switch(uAnaData[2])
	{
		case 0xE0: vCMDHandle(uAnaData);break;
		case 0xE1:
		{
			u8 temp[2]={0};
			temp[0]=DRONE_ADDR;
			temp[1]=0x00;
			vSendOneFrame(COMPUTER_ADDR ,0xE2,2,temp);
		}break;
		case 0xE2: break;
		default: break;
	}
}

static void vCMDHandle(u8 *uAnaData)    //δ���
{
	switch(uAnaData[4])
	{
		case 0x01:
		{
			switch(uAnaData[4])
			{
				case 0x00: break;
				case 0x01: break;
				case 0x10: break;
				case 0x20: break;
			}
		}break;
		case 0x02:
		{
			switch(uAnaData[4])
			{
				case 0x00: break;
				case 0x01: break;
				case 0x02: break;
				case 0x03: break;
			}break;
		}
	}
}

static void vSendDataPort(const unsigned char *DateToSend,int len)
{
	USART1_Send(DateToSend,len);
}


/**
 * @brief  ����һ֡����
 * @param  ��ַ��id�����ݳ��ȣ�Ҫ���͵�����
 * @return None
 */
static void vSendOneFrame(const u8 address ,const u8 id ,const u8 len,u8 *data)
{
	u8 *temp=NULL;
	u8 sumcheck=0;    //��У��
	u8 addcheck=0;    //����У��
	temp = (u8*)malloc(sizeof(u8)*(len+6));    //��̬����һ������
	if(temp==NULL) return;    //����ʧ�ܷ���
	*temp = 0xAA;
	*(temp+1) = address;
	*(temp+2) = id;
	*(temp+3) = len;
	for(u8 i=0;i<len;i++)    //��������
	{
		*(temp+4+i) = data[i];
	}
	for(u8 i=0;i<(len+4);i++)    //У�����
	{
		sumcheck += *(temp+i);
		addcheck += sumcheck;
	}
	*(temp+len+4)=sumcheck;
	*(temp+len+5)=addcheck;
	vSendDataPort(temp,(len+6));    //����
	free(temp);    //�ͷ��ڴ�
}

