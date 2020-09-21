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
#include "led.h"



#define LED_PORT GPIOE
#define LED_PIN_0 GPIO_Pin_0    //����ɫ��
#define LED_PIN_1 GPIO_Pin_1    //����ɫ��
#define LED_PIN_2 GPIO_Pin_2    //����ɫ��

//ָʾ����Ϣ
//��ɫ			 
#define RED_H LED_PORT->ODR |= 0x04           //��
#define RED_L  LED_PORT->ODR &= ~0x04         //��
#define RED_Toggle  LED_PORT->ODR ^= 0x04     //��	
//��ɫ
#define GREE_H LED_PORT->ODR |= 0x02          //��
#define GREE_L  LED_PORT->ODR &= ~0x02        //��
#define GREE_Toggle LED_PORT->ODR ^= 0x02     //��	
//��ɫ
#define BLUE_H LED_PORT->ODR |= 0x01          //��
#define BLUE_L  LED_PORT->ODR &= ~0x01        //��
#define BLUE_Toggle  LED_PORT->ODR ^= 0x01    //��	
//��ɫ
#define CYAN_H  LED_PORT->ODR |= 0x03         //��
#define CYAN_L  LED_PORT->ODR &= ~0x03		     //��							 
#define CYAN_Toggle LED_PORT->ODR ^= 0x03     //��	
//��ɫ										
#define PINK_H  LED_PORT->ODR |= 0x05         //��
#define PINK_L  LED_PORT->ODR &= ~0x05		     //��								
#define PINK_Toggle LED_PORT->ODR ^= 0x05	   //��	
//��ɫ										
#define YELLOW_H LED_PORT->ODR |= 0x06	       //��
#define YELLOW_L LED_PORT->ODR &= ~0x06	     //��
#define YELLOW_Toggle LED_PORT->ODR ^= 0x06	 //��																	
//��ɫ										
#define WHITE_H  LED_PORT->ODR |= 0x07        //��
#define WHITE_L  LED_PORT->ODR &= ~0x07		   //��							
#define WHITE_Toggle LED_PORT->ODR ^= 0x07	   //��	

/**
  * @brief  ��ʼ������
  * @param  None
  * @retval None
  */
void vLEDInit(void)	
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	
  GPIO_StructInit(&GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin   = LED_PIN_2| LED_PIN_1| LED_PIN_0;
	GPIO_Init(LED_PORT, &GPIO_InitStructure);
	
	GPIO_SetBits(LED_PORT, LED_PIN_2);		
	GPIO_SetBits(LED_PORT, LED_PIN_1);		
	GPIO_SetBits(LED_PORT, LED_PIN_0);			
}

/**
  * @brief  ����LED��ɫ
  * @param  LED_Type�ṹ��
  * @retval None
  */
static void vSetLEDColor(LED_Type *xLED)
{
	WHITE_H;    //Ϩ�����е�
	switch(xLED->color)
	{
		case RED: RED_L; break;
		case GREE: GREE_L; break;
		case BLUE: BLUE_L; break;
		case CYAN: CYAN_L; break;
		case PINK: PINK_L; break;
		case YELLOW: YELLOW_L; break;
		case WHITE: WHITE_L; break;
		default: WHITE_H; break;
		
	}
}

/**
  * @brief  ����LED��ɫ��ת
  * @param  LED_Type�ṹ��
  * @retval None
  */
static void vSetLEDToggle(LED_Type *xLED)
{
	switch(xLED->color)
	{
		case RED: RED_Toggle; break;
		case GREE: GREE_Toggle; break;
		case BLUE: BLUE_Toggle; break;
		case CYAN: CYAN_Toggle; break;
		case PINK: PINK_Toggle; break;
		case YELLOW: YELLOW_Toggle; break;
		case WHITE: WHITE_Toggle; break;
		case RANDOM:
		{ 
			static u8 random;
			random += TIM2->CNT ;
			random++;
			LED_PORT->ODR ^= (random&0x01);
			LED_PORT->ODR ^= ((random>>1)&0x02);
			LED_PORT->ODR ^= ((random>>2)&0x04);
		}
			break;
		default: WHITE_H; break;
		
	}
}

/**
  * @brief  LED��ʾ����
  * @param  LED_Type�ṹ��
  * @retval None
  */
void vLEDDisplay(LED_Type *xLED)
{
	if(xLED->status != xLED->last_status)      //״̬�л�
	{
		WHITE_H;
		xLED->last_status = xLED->status;
	}
	else if(xLED->color != xLED->last_color)  //��ɫ�л�
	{
		WHITE_H;
		xLED->last_color = xLED->color;
	}
	switch(xLED->status)
	{
		case AlwaysOff: WHITE_H; break;
		case AlwaysOn: vSetLEDColor(xLED); break;
		case Flash:
		{
			if(uSystickCount-xLED->uFlashTimestart < xLED->FlashTime) return;    //�����Ƿ�ˢ��ʱ��
			xLED->uFlashTimestart = uSystickCount;
			vSetLEDToggle(xLED); 
		}break;
	}
}
