#include "stm32f4xx.h"
#include "CAN.h"


//	//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN初始化环回模式,波特率500Kbps    

////////////////////////////////////////////////////////////////////////////////// 	 

//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//则波特率为:42M/((6+7+1)*6)=500Kbps
//返回值:0,初始化OK;
//    其他,初始化失败; 

uint8_t CAN1_buf[8];
uint8_t CAN2_buf[8];
uint8_t CAN_BUF_1[600][8];
uint8_t CAN_BUF_2[600][8];
int buf1 = 0,buf2 = 0;

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
		u16 std_id_1 =0x123;  
	
		u16 std_id_2 =0x123;  
	
  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //使能相关时钟
//	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟	   
	
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
//		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12
		
//		 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA12,PA13
	
	  //引脚复用映射配置
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
//		GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOA11复用为CAN2
//	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOA12复用为CAN2
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
    
//		CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
//  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
//  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
//  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
//  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
//  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
//  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
//  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
//  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
//  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
//  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
//  	CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN2
		//配置过滤器
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //屏蔽模式
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)std_id_1<<21)&0xFFFF0000)>>16;;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=(((u32)std_id_1<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF; 	

  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffff;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
/*---------------------------------------------------------------------------------------------------------------------------------*/		
		
//		CAN_FilterInitStructure.CAN_FilterNumber=14; //过滤器14
//  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //屏蔽模式
//  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
//  	CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)std_id_2<<21)&0xFFFF0000)>>16;;////32位ID
//  	CAN_FilterInitStructure.CAN_FilterIdLow=(((u32)std_id_2<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF; 
//		
//  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffff;//32位MASK
//  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;
//   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
//  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
//		CAN_FilterInit(&CAN_FilterInitStructure);  //CAN2
		
#if CAN1_RX0_INT_ENABLE
	
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif

//#if CAN2_RX0_INT_ENABLE
//	
//	  CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
//  
//  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // 次优先级为0
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  	NVIC_Init(&NVIC_InitStructure);
//#endif
	return 0;
}   
 
#if CAN1_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void CAN1_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
	int i=0;
    CAN_Receive(CAN1, 0, &RxMessage);
		for(i=0;i<8;i++)
	  {CAN_BUF_1[buf1][i] = RxMessage.Data[i];
		}
		buf1++;
//	printf("CAN1中断接收:");
//	for(i=0;i<8;i++)
//	printf("%d,",RxMessage.Data[i]);
//	printf("\r\n");
}
#endif

//#if CAN2_RX0_INT_ENABLE	//使能RX0中断
////中断服务函数			    
//void CAN2_RX0_IRQHandler(void)
//{
//  	CanRxMsg RxMessage;
//	int i=0;
//    CAN_Receive(CAN2, 0, &RxMessage);
//	for(i=0;i<8;i++)
//	     CAN_BUF_2[buf2][i] = RxMessage.Data[i];
//	buf2++;
////	printf("%d,",RxMessage.Data[i]);
////	printf("\r\n");
//}
//#endif


//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x12;	 // 标准标识符为0
  TxMessage.ExtId=0x12;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		

}

//u8 CAN2_Send_Msg(u8* msg,u8 len)
//{	
//  u8 mbox;
//  u16 i=0;
//  CanTxMsg TxMessage;
//  TxMessage.StdId=0x15;	 // 标准标识符为0
//  TxMessage.ExtId=0x12;	 // 设置扩展标示符（29位）
//  TxMessage.IDE=0;		  // 使用扩展标识符
//  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
//  TxMessage.DLC=len;							 // 发送两帧信息
//  for(i=0;i<len;i++)
//  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
//  mbox= CAN_Transmit(CAN2, &TxMessage);   
//  i=0;
//  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
//  if(i>=0XFFF)return 1;
//  return 0;		

//}
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}

//u8 CAN2_Receive_Msg(u8 *buf)
//{
// 	u32 i;
//	CanRxMsg RxMessage;
//    if( CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
//    CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);//读取数据	
//    for(i=0;i<RxMessage.DLC;i++)
//    buf[i]=RxMessage.Data[i];  
//	return RxMessage.DLC;	
//}







