#include "ppm.h"
#include "string.h"
u16 PPM_Sample_Cnt=0;
u16 PPM_Isr_Cnt=0;
u32 Last_PPM_Time=0;
u32 PPM_Time=0;
u32 PPM_Time_Delta=1000;
u16 PPM_Time_Max=0;
u16 PPM_Start_Time=0;
u16 PPM_Finished_Time=0;
u16 PPM_Is_Okay=0;
u16 PPM_Databuf[10]={0};//ppm数据的十个通道储存位置
u32 TIME_ISR_CNT=0;    //用于TIM中断计时,需在中断处理中外部引用
/**
  * @brief  在初始化函数中配置PPM所用外部中断以及计时器
  * @param  None
  * @retval None
  */
void vPPMInit()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;//定义NVIC初始化结构体
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOA时钟
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;//GPIO_Pin_0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入下拉
	GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_DOWN;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, GPIO_PinSource12);
  EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd	= ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	

	//PPM接收机
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
	
	//配置计时器
	vTIM2Init(10000-1,84-1);    //10ms中断一次
	
}



static u16 PPM_buf[8]={0};//读取x个通道的储存位置
/**
  * @brief  PPM解析回调函数，在对应中断处理函数中调用
  * @param  用于计时的时钟
  * @retval None
  */
void vPPMCallback(TIM_TypeDef* TIMx)
{
    //系统运行时间获取，单位us
    Last_PPM_Time=PPM_Time;
    PPM_Time=10000*TIME_ISR_CNT+TIMx->CNT;//us
    PPM_Time_Delta=PPM_Time-Last_PPM_Time;
    //PPM中断进入判断
    if(PPM_Isr_Cnt<100)  PPM_Isr_Cnt++;
    //PPM解析开始
    if(PPM_Is_Okay==1)
    {
      if(PPM_Time_Delta>=800&&PPM_Time_Delta<=2200)
      {
        PPM_Sample_Cnt++;
        //对应通道写入缓冲区
        PPM_buf[PPM_Sample_Cnt-1]=PPM_Time_Delta;
        //单次解析结束
        if(PPM_Sample_Cnt>=8)
        {
          memcpy(PPM_Databuf,PPM_buf,PPM_Sample_Cnt*sizeof(u16));//将读取到的x个通道写入ppm数据十个通道的储存位置
          PPM_Is_Okay=0;
        }
      }
      else
      {
        if(PPM_Time_Delta>=2500)//帧结束电平至少2ms=2000us，由于部分老版本遥控器、
          //接收机输出PPM信号不标准，当出现解析异常时，尝试改小此值，该情况仅出现一例：使用天地飞老版本遥控器
        {
          memcpy( PPM_Databuf,PPM_buf,PPM_Sample_Cnt*sizeof(u16));
          PPM_Is_Okay = 1;
          PPM_Sample_Cnt=0;
        }
        else  PPM_Is_Okay=0;
      }
    }
    else if(PPM_Time_Delta>=2500)//帧结束电平至少2ms=2000us
    {
      PPM_Is_Okay=1;
      PPM_Sample_Cnt=0;
    }	
}
