
#include "usart.h"	

 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F4探索者开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/6/10
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
void _ttywrch(int ch)
{
	ch=ch;
}
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1); //GPIOB6复用为USART1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); //GPIOB7复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化B6，B7

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_Cmd(USART1, ENABLE);  //使能串口1 
//		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
//  USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif
	
}

/****************************************************************************************************
* 函  数: void USART1_IRQHandler(void)
* 功  能: USART1中断函数，调试使用
* 参  数: 无
* 返回值: 无
* 备  注: 当插线调参时，用USART1对于连续的数据帧的接收 接收中断与空闲中断配合能解决对报问题
****************************************************************************************************/
void USART1_IRQHandler(void)
{
	uint8_t clear = clear; //定义这个变量是针对编译出现“没有用到这个变量”的警告提示
	uint8_t res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //接收中断
	{ 
		res = USART1->DR;
//		ANO_DT_Data_Receive_Prepare(res); //上位机数据接收与解析
	}else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) //空闲中断
	{
		clear = USART1->SR; //读SR寄存器
		clear = USART1->DR; //读DR寄存器（先读SR,再度DR,就是为了清除IDIE中断）
		
	}
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		}	
#endif

void Usart_Send(uint8_t *data, uint8_t length)
{
	uint8_t  i;
		for(i=0;i<length;i++)
		{
			 USART_SendData(USART1, *(data+i));
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
			{}
		}

}




//初始化IO 串口2 
//bound:波特率
void uart2_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	USART_DeInit(USART2);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟

	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); //GPIOD5复用为USART2
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); //GPIOD6复用为USART2
	
	//USART2端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; //GPIOD5与GPIOD6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化D5，D6

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
  USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
  USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
  USART_Cmd(USART2, ENABLE);  //使能串口2 

	
	//USART_ClearFlag(USART2, USART_FLAG_TC);
	
#if EN_USART2_RX	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif
	
}

/****************************************************************************************************
* 函  数: void USART2_IRQHandler(void)
* 功  能: USART2中断函数，调试使用
* 参  数: 无
* 返回值: 无
* 备  注: 当插线调参时，用USART1对于连续的数据帧的接收 接收中断与空闲中断配合能解决对报问题
****************************************************************************************************/
void USART2_IRQHandler(void)
{
	uint8_t clear = clear; //定义这个变量是针对编译出现“没有用到这个变量”的警告提示
	uint8_t res;
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //接收中断
	{ 
		res = USART2->DR;
//		ANO_DT_Data_Receive_Prepare(res); //上位机数据接收与解析
	}else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) //空闲中断
	{
		clear = USART2->SR; //读SR寄存器
		clear = USART2->DR; //读DR寄存器（先读SR,再度DR,就是为了清除IDIE中断）
		
	}
	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}


void Usart2_Send(uint8_t *data, uint8_t length)
{
	uint8_t  i;
		for(i=0;i<length;i++)
		{
			 USART_SendData(USART2, *(data+i));
			while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
			{}
		}

}



//void USART4_Init(unsigned long bound)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//  
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE );
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE );
//  
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10复用为USART4
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11复用为USART4
//	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC10与GPIOC11
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
//	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化C10，C11
//  
//  USART_InitStructure.USART_BaudRate = bound; //波特率；
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b; //数据位8位；
//  USART_InitStructure.USART_StopBits = USART_StopBits_1; //停止位1位；
//  USART_InitStructure.USART_Parity = USART_Parity_No ; //无校验位；
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控；
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式；
//  USART_Init(UART4, &USART_InitStructure);//配置串口参数；
//  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//中断	
//	USART_Cmd(UART4, ENABLE); //使能串口；
//	
//	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口2中断通道
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级2
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
//  RingBuff_Init(&OpticalFlow_Ringbuf);
////	OpticalFlow_Is_Work=Config_Init_Uart();//检查光流是否可用
//}

//void Usart4_Send(uint8_t *data, uint8_t length)
//{
//	uint8_t  i;
//		for(i=0;i<length;i++)
//		{
//			 USART_SendData(UART4, *(data+i));
//			while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET)
//			{}
//		}
//}

//void USART4_Send(u8 Data) //发送一个字节；
//{ 
//  USART_SendData(UART4,Data);
//  while( USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET );
//}


//void UART4_Send(u8 *Data) //发送字符串；
//{
//  while(*Data)
//    USART4_Send(*Data++);
//}

//void UART4_IRQHandler(void) //中断处理函数；
//{ 
//  if(USART_GetITStatus(UART4, USART_IT_RXNE) == SET) //判断是否发生中断；
//	{
//			USART_ClearFlag(UART4, USART_IT_RXNE); //清除标志位
//      RingBuf_Write(USART_ReceiveData(UART4),&OpticalFlow_Ringbuf,28);//往环形队列里面写数据
//	}
//}
