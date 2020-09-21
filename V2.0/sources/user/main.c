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

u32 uSystickCount = 0;    //全局计时参数


MPU6050Data_TypeDef MPUData;    //6050数据保存结构体
AK8975Data_Type CompassData;    //磁力计数据保存结构体
LED_Type xLED;                  //LED控制结构体
Remote_Type xRemote;            //遥控数据保存结构体
Drone_Type MyDrone;             //无人机状态保存结构体
KalmanData_Type KalmanData;

void RCCConfig(void);

int main(void)
{
	u8 temp_data[8]={0};
	u8 temp[8]={0};
//	u8 ID_TEST=0;
//	u8 test_id=0;
	RCCConfig();    //设定stm32频率
	Delay_Init();   //Systick  1ms
	//while(SysTick_Config(SystemCoreClock/1000)){};    //Systick  1ms
  vAllInit();     //全局初始化
	
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
    if(MyDrone.State==unLock) vDroneunLock();    //无人机解锁状态
		if(MyDrone.State==Lock) vDroneLock();        //无人机锁定状态
//		vSendDataPolling();
		
//调试用代码，上传时删
//		MPU6050_GetGyroAccData(&MPUData);
//		AK8975_GetMagData(&CompassData);
//		temp=AK8975_GetID();
//		AT24C02_NByteRandomRead(8 ,8,temp_data);
	}
}

////重定义fputc函数 
//int fputc(int ch, FILE *f)
//{ 	
//  USART_SendData(USART2, ch); 
//	return ch;
//}

void RCCConfig()
{
	RCC_HSEConfig(RCC_HSE_ON);                         //使用外部高速时钟
	RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 8);    //168MHz
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);         //SYSCLK时钟168MHz
	RCC_HCLKConfig(RCC_SYSCLK_Div1);                   //HCLK  168MHz
	RCC_PCLK1Config(RCC_HCLK_Div4);                    //PCLK1  42MHz
	RCC_PCLK2Config(RCC_HCLK_Div2);                    //PCLK2  84MHz
	RCC_PLLCmd(ENABLE);                                //使能外部高速时钟
}

