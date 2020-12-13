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
#ifndef __LED_H__
#define __LED_H__

#include "stm32f4xx.h"

typedef struct
{
	short FlashTime;      //LED更新时间
	u32 uFlashTimestart;  //计时标记
//	u32 uSystickTime;    //计时参数
	enum
	{
		AlwaysOn,           //常亮 
		AlwaysOff,          //常暗
		Flash,              //闪烁一次
	}status,last_status;  //模式选择   当前模式，上一次的模式
	enum                  //颜色选择
	{
		RED,
		GREE,    //green
		BLUE,
		CYAN,    //cyan  青色
		PINK,
		YELLOW,
		WHITE,
		RANDOM,  //随机颜色
	}color,last_color;    //颜色选择   当前颜色，上一次颜色
}LED_Type;

extern u32 uSystickCount;

void vLEDDisplay(LED_Type *xLED);
void vLEDInit(void);

#endif 

