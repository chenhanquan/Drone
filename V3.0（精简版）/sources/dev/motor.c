#include "motor.h"

#define PWM_MAX 2000

void vMotorInit()//400hz  单位占空比百分比% //航模占空比标准输出都是为40%~80%
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB,ENABLE);

//----------------------------TIM3------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);	
	
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 10000;									
  TIM_TimeBaseStructure.TIM_Prescaler = 21;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;    //PWM1模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;    //PWM输出使能
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; //PWM反极性输出失能
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;        //PWM有效电平为高
  TIM_OCInitStructure.TIM_Pulse = 0;                            //预装载值为4000

  /* PWM1 Mode configuration: Channel1 */
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);	

}

void vMotorPWMSet(u16 Motor1,u16 Motor2,u16 Motor3,u16 Motor4)
{
	if(Motor1 > PWM_MAX) Motor1=PWM_MAX;
	if(Motor2 > PWM_MAX) Motor2=PWM_MAX;
	if(Motor3 > PWM_MAX) Motor3=PWM_MAX;
	if(Motor4 > PWM_MAX) Motor4=PWM_MAX;
	
	TIM3->CCR4=(uint32_t)(6.0f*Motor1+3850);    //2400最小  4000为满    对电机1补偿50pwm
	TIM3->CCR3=(uint32_t)(6.0f*Motor2+3850);
	TIM3->CCR2=(uint32_t)(6.0f*Motor3+3850);
	TIM3->CCR1=(uint32_t)(6.0f*Motor4+3850);
	
//	TIM3->CCR4=Motor1+3850;    //电机测试代码
//	TIM3->CCR3=Motor2+3850;
//	TIM3->CCR2=Motor3+3850;
//	TIM3->CCR1=Motor4+3850;


}

void vMotorLock()
{
	vMotorPWMSet(0,0,0,0);
}

