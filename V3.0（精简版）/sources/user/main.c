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

u32 uSystickCount = 0;    //ȫ�ּ�ʱ����


MPU6050Data_TypeDef MPUData;    //6050���ݱ���ṹ��
AK8975Data_Type CompassData;    //���������ݱ���ṹ��
LED_Type xLED;                  //LED���ƽṹ��
Remote_Type xRemote;            //ң�����ݱ���ṹ��
Drone_Type MyDrone;             //���˻�״̬����ṹ��

//u16 model_state=0x1000;

void RCCConfig(void);

int main(void)
{
 	RCCConfig();    //�趨stm32Ƶ��
	Delay_Init();   //Systick  1ms
	//while(SysTick_Config(SystemCoreClock/1000)){};    //Systick  1ms
  vAllInit();     //ȫ�ֳ�ʼ��
//	uart_init(115200);
//	uart2_init(115200);
	//Init_GPS();
	
	

	while(1){
    if(MyDrone.State==unLock) vDroneunLock();    //���˻�����״̬
		if(MyDrone.State==Lock) vDroneLock();        //���˻�����״̬
		
		
//�����ô��룬�ϴ�ʱɾ
		
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
	RCC_HSEConfig(RCC_HSE_ON);                         //ʹ���ⲿ����ʱ��
	RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 8);    //168MHz
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);         //SYSCLKʱ��168MHz
	RCC_HCLKConfig(RCC_SYSCLK_Div1);                   //HCLK  168MHz
	RCC_PCLK1Config(RCC_HCLK_Div4);                    //PCLK1  42MHz
	RCC_PCLK2Config(RCC_HCLK_Div2);                    //PCLK2  84MHz
	RCC_PLLCmd(ENABLE);                                //ʹ���ⲿ����ʱ��
}


