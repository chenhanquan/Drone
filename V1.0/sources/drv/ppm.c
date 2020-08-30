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
u16 PPM_Databuf[10]={0};//ppm���ݵ�ʮ��ͨ������λ��
u32 TIME_ISR_CNT=0;    //����TIM�жϼ�ʱ,�����жϴ������ⲿ����
/**
  * @brief  �ڳ�ʼ������������PPM�����ⲿ�ж��Լ���ʱ��
  * @param  None
  * @retval None
  */
void vPPMInit()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;//����NVIC��ʼ���ṹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOAʱ��
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;//GPIO_Pin_0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��������
	GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_DOWN;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, GPIO_PinSource12);
  EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd	= ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	

	//PPM���ջ�
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
	
	//���ü�ʱ��
	vTIM2Init(10000-1,84-1);    //10ms�ж�һ��
	
}



static u16 PPM_buf[8]={0};//��ȡx��ͨ���Ĵ���λ��
/**
  * @brief  PPM�����ص��������ڶ�Ӧ�жϴ������е���
  * @param  ���ڼ�ʱ��ʱ��
  * @retval None
  */
void vPPMCallback(TIM_TypeDef* TIMx)
{
    //ϵͳ����ʱ���ȡ����λus
    Last_PPM_Time=PPM_Time;
    PPM_Time=10000*TIME_ISR_CNT+TIMx->CNT;//us
    PPM_Time_Delta=PPM_Time-Last_PPM_Time;
    //PPM�жϽ����ж�
    if(PPM_Isr_Cnt<100)  PPM_Isr_Cnt++;
    //PPM������ʼ
    if(PPM_Is_Okay==1)
    {
      if(PPM_Time_Delta>=800&&PPM_Time_Delta<=2200)
      {
        PPM_Sample_Cnt++;
        //��Ӧͨ��д�뻺����
        PPM_buf[PPM_Sample_Cnt-1]=PPM_Time_Delta;
        //���ν�������
        if(PPM_Sample_Cnt>=8)
        {
          memcpy(PPM_Databuf,PPM_buf,PPM_Sample_Cnt*sizeof(u16));//����ȡ����x��ͨ��д��ppm����ʮ��ͨ���Ĵ���λ��
          PPM_Is_Okay=0;
        }
      }
      else
      {
        if(PPM_Time_Delta>=2500)//֡������ƽ����2ms=2000us�����ڲ����ϰ汾ң������
          //���ջ����PPM�źŲ���׼�������ֽ����쳣ʱ�����Ը�С��ֵ�������������һ����ʹ����ط��ϰ汾ң����
        {
          memcpy( PPM_Databuf,PPM_buf,PPM_Sample_Cnt*sizeof(u16));
          PPM_Is_Okay = 1;
          PPM_Sample_Cnt=0;
        }
        else  PPM_Is_Okay=0;
      }
    }
    else if(PPM_Time_Delta>=2500)//֡������ƽ����2ms=2000us
    {
      PPM_Is_Okay=1;
      PPM_Sample_Cnt=0;
    }	
}
