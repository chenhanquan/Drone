/**
  ******************************************************************************
  * @file    led.c
  * @author  CHEN HANQUAN
  * @version V1.0
  * @date    16-August-2020
  * @brief   此函数用于控制zin小店的RGB灯，使用时需要配置Systick计时器，并定义一
	*          个全局计时参数uSystickCount用于计时。
  * @note    此函数我是修改别人的代码，减少了文件与文件之间的关联性。此函数是使用
	*          查询刷新时间的方法进行闪烁，不是使用延时的方法，提高系统效率。
	*
  ******************************************************************************
	                               使用方法
	******************************************************************************
	* 定义一个LED_Type结构体，设置好灯的闪烁时间、方式、颜色，将结构体传入vLEDDisplay函数
  */
#include "led.h"



#define LED_PORT GPIOE
#define LED_PIN_0 GPIO_Pin_0    //连蓝色灯
#define LED_PIN_1 GPIO_Pin_1    //连绿色灯
#define LED_PIN_2 GPIO_Pin_2    //连红色灯

//指示灯信息
//红色			 
#define RED_H LED_PORT->ODR |= 0x04           //暗
#define RED_L  LED_PORT->ODR &= ~0x04         //亮
#define RED_Toggle  LED_PORT->ODR ^= 0x04     //闪	
//绿色
#define GREE_H LED_PORT->ODR |= 0x02          //暗
#define GREE_L  LED_PORT->ODR &= ~0x02        //亮
#define GREE_Toggle LED_PORT->ODR ^= 0x02     //闪	
//蓝色
#define BLUE_H LED_PORT->ODR |= 0x01          //暗
#define BLUE_L  LED_PORT->ODR &= ~0x01        //亮
#define BLUE_Toggle  LED_PORT->ODR ^= 0x01    //闪	
//青色
#define CYAN_H  LED_PORT->ODR |= 0x03         //暗
#define CYAN_L  LED_PORT->ODR &= ~0x03		     //亮							 
#define CYAN_Toggle LED_PORT->ODR ^= 0x03     //闪	
//粉色										
#define PINK_H  LED_PORT->ODR |= 0x05         //暗
#define PINK_L  LED_PORT->ODR &= ~0x05		     //亮								
#define PINK_Toggle LED_PORT->ODR ^= 0x05	   //闪	
//黄色										
#define YELLOW_H LED_PORT->ODR |= 0x06	       //暗
#define YELLOW_L LED_PORT->ODR &= ~0x06	     //亮
#define YELLOW_Toggle LED_PORT->ODR ^= 0x06	 //闪																	
//白色										
#define WHITE_H  LED_PORT->ODR |= 0x07        //暗
#define WHITE_L  LED_PORT->ODR &= ~0x07		   //亮							
#define WHITE_Toggle LED_PORT->ODR ^= 0x07	   //闪	

/**
  * @brief  初始化函数
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
  * @brief  设置LED颜色
  * @param  LED_Type结构体
  * @retval None
  */
static void vSetLEDColor(LED_Type *xLED)
{
	WHITE_H;    //熄灭所有灯
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
  * @brief  设置LED颜色翻转
  * @param  LED_Type结构体
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
  * @brief  LED显示函数
  * @param  LED_Type结构体
  * @retval None
  */
void vLEDDisplay(LED_Type *xLED)
{
	if(xLED->status != xLED->last_status)      //状态切换
	{
		WHITE_H;
		xLED->last_status = xLED->status;
	}
	else if(xLED->color != xLED->last_color)  //颜色切换
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
			if(uSystickCount-xLED->uFlashTimestart < xLED->FlashTime) return;    //计算是否到刷新时间
			xLED->uFlashTimestart = uSystickCount;
			vSetLEDToggle(xLED); 
		}break;
	}
}
