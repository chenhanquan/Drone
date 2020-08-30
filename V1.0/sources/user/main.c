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
#include "allcontrol.h"
#include "PPM.h"
#include "led.h"
#include "mpu6050.h"
#include "ANO_DT.h"
#include "filter.h"
#include "remote.h"

u32 uSystickCount = 0;    //ȫ�ּ�ʱ����


MPU6050Data_TypeDef MPUData;    //6050���ݱ���ṹ��
LED_Type xLED;                  //LED���ƽṹ��
FilterData_Type FilterData;     //�����ں����ݱ���ṹ��
Remote_Type xRemote;            //ң�����ݱ���ṹ��
Drone_Type MyDrone;             //���˻�״̬����ṹ��


void RCCConfig(void);

int main(void)
{
	RCCConfig();    //�趨stm32Ƶ��
	while(SysTick_Config(SystemCoreClock/1000)){};    //Systick  1ms
  vAllInit();     //ȫ�ֳ�ʼ��
	while(1){
    if(MyDrone.State==unLock) vDroneunLock();    //���˻�����״̬
		if(MyDrone.State==Lock) vDroneLock();        //���˻�����״̬
		vSendDataPolling();
	}
//	return 0;
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

