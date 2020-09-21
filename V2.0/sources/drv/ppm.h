#ifndef _PPM_H
#define _PPM_H
#include "stm32f4xx.h"
#include "TIM.h"



//extern u16 PPM_Sample_Cnt;
//extern u16 PPM_Isr_Cnt;
//extern u32 Last_PPM_Time;
//extern u32 PPM_Time;
//extern u32 PPM_Time_Delta;
//extern u16 PPM_Time_Max;
//extern u16 PPM_Start_Time;
//extern u16 PPM_Finished_Time;
//extern u16 PPM_Is_Okay;
//extern u16 PPM_Databuf[10];//ppm数据的十个通道储存位置
void vPPMInit(void);
void vPPMCallback(TIM_TypeDef* TIMx);
//extern u16 PPM_Isr_Cnt;
//extern u16 PPM_Databuf[10];
//extern u32 PPM_Time;

#endif
