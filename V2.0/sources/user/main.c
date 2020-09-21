/**
  ******************************************************************************
  * @file    main.c
  * @author  CHEN HANQUAN
  * @version Demo
  * @date    13-August-2020
  * @brief    
  *
  ******************************************************************************
  */

#include "stm32f4xx.h"
#include "delay.h"

#include "allcontrol.h"
#include "led.h"
#include "mpu6050.h"
#include "ANO_DT.h"
#include "remote.h"
#include "AK8975.h"
#include "Kalman.h"

u32 uSystickCount = 0;    //ȫ�ּ�ʱ����


MPU6050Data_TypeDef MPUData;    //6050���ݱ���ṹ��
AK8975Data_Type CompassData;    //���������ݱ���ṹ��
LED_Type xLED;                  //LED���ƽṹ��
Remote_Type xRemote;            //ң�����ݱ���ṹ��
Drone_Type MyDrone;             //���˻�״̬����ṹ��
KalmanData_Type KalmanData;

void RCCConfig(void);

int main(void)
{
	u8 temp_data[8]={0};
	u8 temp[8]={0};
//	u8 ID_TEST=0;
//	u8 test_id=0;
	RCCConfig();    //�趨stm32Ƶ��
	Delay_Init();   //Systick  1ms
	//while(SysTick_Config(SystemCoreClock/1000)){};    //Systick  1ms
  vAllInit();     //ȫ�ֳ�ʼ��
	
	/////////////////////////////////////////////////////////
//	ID_TEST=AT24C02_ByteCurrentRead();
////	for(u8 i=0;i<32;i++)
////	{
////		u8 temp[8]={0};
////		for(u8 j=0;j<8;j++)
////		{
////			temp[j]=i+j;
////		}
////		AT24C02_WriteNByte(8*i,8,temp);
////	}
//	for(u8 i=0;i<8;i++){temp[i]=i+15;}
//	AT24C02_PageWrite(1,temp);
//	test_id=AT24C02_ByteCurrentRead();
//	
	////////////////////////////////////////////////////////////
	
	while(1){
    if(MyDrone.State==unLock) vDroneunLock();    //���˻�����״̬
		if(MyDrone.State==Lock) vDroneLock();        //���˻�����״̬
//		vSendDataPolling();
		
//�����ô��룬�ϴ�ʱɾ
//		MPU6050_GetGyroAccData(&MPUData);
//		AK8975_GetMagData(&CompassData);
//		temp=AK8975_GetID();
//		AT24C02_NByteRandomRead(8 ,8,temp_data);
	}
}

////�ض���fputc���� 
//int fputc(int ch, FILE *f)
//{ 	
//  USART_SendData(USART2, ch); 
//	return ch;
//}

void RCCConfig()
{
	RCC_HSEConfig(RCC_HSE_ON);                         //ʹ���ⲿ����ʱ��
	RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 8);    //168MHz
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);         //SYSCLKʱ��168MHz
	RCC_HCLKConfig(RCC_SYSCLK_Div1);                   //HCLK  168MHz
	RCC_PCLK1Config(RCC_HCLK_Div4);                    //PCLK1  42MHz
	RCC_PCLK2Config(RCC_HCLK_Div2);                    //PCLK2  84MHz
	RCC_PLLCmd(ENABLE);                                //ʹ���ⲿ����ʱ��
}

