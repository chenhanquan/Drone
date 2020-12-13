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
#include "AT24C02.h"
#include "GPS.h"
#include "spl06.h"

u32 uSystickCount = 0;    //全局计时参数


MPU6050Data_TypeDef MPUData;    //6050数据保存结构体
AK8975Data_Type CompassData;    //磁力计数据保存结构体
LED_Type xLED;                  //LED控制结构体
Remote_Type xRemote;            //遥控数据保存结构体
Drone_Type MyDrone;             //无人机状态保存结构体

//u16 model_state=0x1000;

void RCCConfig(void);

int main(void)
{
 	RCCConfig();    //设定stm32频率
	Delay_Init();   //Systick  1ms
	//while(SysTick_Config(SystemCoreClock/1000)){};    //Systick  1ms
  vAllInit();     //全局初始化
//	uart_init(115200);
//	uart2_init(115200);
	//Init_GPS();
	
	

	while(1){
    if(MyDrone.State==unLock) vDroneunLock();    //无人机解锁状态
		if(MyDrone.State==Lock) vDroneLock();        //无人机锁定状态
		
		
//调试用代码，上传时删
		
//		if(uSystickCount-tickcount>10)
//		{
//			tickcount=uSystickCount;
//			READ_GPS_DATA();
//		}
//    u8 a=0x67;
//		
//		Usart2_Send(&a, 1);
//		Usart_Send(&a, 1);
	}
}


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


