#include "stm32f4xx.h"
#include "usart.h"



void USART2_init(u32 bound)  
{  
    //定义中断结构体  
    NVIC_InitTypeDef NVIC_InitStructure ;  
    //定义IO初始化结构体  
    GPIO_InitTypeDef GPIO_InitStructure;  
    //定义串口结构体    
    USART_InitTypeDef USART_InitStructure;  
    //定义DMA结构体  
    DMA_InitTypeDef DMA_InitStructure;  
  
    //打开串口对应的外设时钟    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  
       
    //串口发DMA配置    
    //启动DMA时钟  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  
    //DMA发送中断设置  
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure);  
    //DMA通道配置  
    DMA_DeInit(DMA1_Stream6);  
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;   
    //外设地址  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);  
    //内存地址  	
    DMA_InitStructure.DMA_Memory0BaseAddr = 0; 
    //dma传输方向  
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  
    //设置DMA在传输时缓冲区的长度  
    DMA_InitStructure.DMA_BufferSize = 0;  
    //设置DMA的外设递增模式，一个外设  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    //设置DMA的内存递增模式  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    //外设数据字长  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
    //内存数据字长  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
    //设置DMA的传输模式  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
    //设置DMA的优先级别  
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
      
    //指定如果FIFO模式或直接模式将用于指定的流 ： 不使能FIFO模式    
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;      
    //指定了FIFO阈值水平  
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;          
    //指定的Burst转移配置内存传输   
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         
    //指定的Burst转移配置外围转移 */    
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
  
    //配置DMA1的通道           
    DMA_Init(DMA1_Stream6, &DMA_InitStructure);    
    //使能中断  
    DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);  
		
   //初始化串口参数    
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;    
    USART_InitStructure.USART_StopBits = USART_StopBits_1;    
    USART_InitStructure.USART_Parity = USART_Parity_No;    
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;    
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      
    USART_InitStructure.USART_BaudRate = bound;   
    //初始化串口   
    USART_Init(USART2,&USART_InitStructure);    
      
    //中断配置  
    USART_ITConfig(USART2,USART_IT_TC,DISABLE);  
    USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);  
    USART_ITConfig(USART2,USART_IT_IDLE,DISABLE);    
  
    //配置中断    
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);  
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;               //通道设置为串口中断    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //中断占先等级  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              //中断响应优先级   
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //打开中断    
    NVIC_Init(&NVIC_InitStructure);     
          
    //采用DMA方式发送  
    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  

    //启动串口    
    USART_Cmd(USART2, ENABLE);      
  
    //设置IO口时钟        
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);   
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
  
		//配置PD5作为USART2　Tx
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
		GPIO_Init(GPIOD, &GPIO_InitStructure); 
		//配置PD6作为USART2　Rx
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
		GPIO_Init(GPIOD, &GPIO_InitStructure); 
	     
}  


void USART2_Send(const u8 *data,u8 len)
{
  u8 i;
	for(i=0;i<len;i++)
	{
				while(!(USART2->SR & 0x80));
		//		USART_SendData(USART2, *(data+i));         //向串口1发送数据
			USART2->DR = *(data+i);
	}
}


//void USART2_IRQHandler(void)                	//串口1中断服务程序
//{
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//	{

//		  		 
//  } 

//} 


void DMA1_Stream6_IRQHandler(void)
{
	if(DMA1->HISR & DMA_IT_TCIF6)
  {
     DMA1->HIFCR = DMA_IT_TCIF6; //清除中断标志位

		 DMA1_Stream6->CR  &= ~ DMA_SxCR_EN; //失能DMA
  }
}
