/**
  ******************************************************************************
  * @file    led.c
  * @author  CHEN HANQUAN
  * @version V1.0
  * @date    16-August-2020
  * @brief   �˺������ڿ���zinС���RGB�ƣ�ʹ��ʱ��Ҫ����Systick��ʱ����������һ
	*          ��ȫ�ּ�ʱ����uSystickCount���ڼ�ʱ��
  * @note    �˺��������޸ı��˵Ĵ��룬�������ļ����ļ�֮��Ĺ����ԡ��˺�����ʹ��
	*          ��ѯˢ��ʱ��ķ���������˸������ʹ����ʱ�ķ��������ϵͳЧ�ʡ�
	*
  ******************************************************************************
	                               ʹ�÷���
	******************************************************************************
	* ����һ��LED_Type�ṹ�壬���úõƵ���˸ʱ�䡢��ʽ����ɫ�����ṹ�崫��vLEDDisplay����
  */
#ifndef __LED_H__
#define __LED_H__

#include "stm32f4xx.h"

typedef struct
{
	short FlashTime;      //LED����ʱ��
	u32 uFlashTimestart;  //��ʱ���
//	u32 uSystickTime;    //��ʱ����
	enum
	{
		AlwaysOn,           //���� 
		AlwaysOff,          //����
		Flash,              //��˸һ��
	}status,last_status;  //ģʽѡ��   ��ǰģʽ����һ�ε�ģʽ
	enum                  //��ɫѡ��
	{
		RED,
		GREE,    //green
		BLUE,
		CYAN,    //cyan  ��ɫ
		PINK,
		YELLOW,
		WHITE,
		RANDOM,  //�����ɫ
	}color,last_color;    //��ɫѡ��   ��ǰ��ɫ����һ����ɫ
}LED_Type;

extern u32 uSystickCount;

void vLEDDisplay(LED_Type *xLED);
void vLEDInit(void);

#endif 

