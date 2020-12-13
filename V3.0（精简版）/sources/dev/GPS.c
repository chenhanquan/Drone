#include "GPS.h"

gps_data GPS;
extern u8 USART_RX_BUF[];
//extern  u16 model_state;

u8 GPS_RESET[21]={0xB5,0x62,0x06,0x09,0x0D,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x03,0x1B,0x9A};
u8 CLOSE_GxGGA[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24};
u8 CLOSE_GxGLL[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B};
u8 CLOSE_GxGSA[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32};
u8 CLOSE_GxGSV[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39};
u8 CLOSE_GxRMC[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40};
u8 CLOSE_GxVTG[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47};
u8 OPEN_NAV_PVT[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1};
u8 RAT_10[14]={0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12};
u8 BAUDRATE[28]={0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x10,0x0E,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x1B,0x5A};
u8 STOP_GPS[12]={0xB5,0x62,0x06,0x04,0x04,0x00,0x00,0x00,0x08,0x00,0x16,0x74};
u8 START_GPS[12]={0xB5,0x62,0x06,0x04,0x04,0x00,0x00,0x00,0x09,0x00,0x17,0x76};
u8 GPS_ERR=0;


void Init_GPS(void)
{
u8 i=11;
//model_state=model_state|0x1000;
//复位GPS模块的所有设置
while(i--)
			{
					switch (i)
								{
									case 10: uart2_init(921600);
													 break;
									case 9:  uart2_init(460800);
													 break;
									case 8:  uart2_init(230400);	
													 break;
									case 7:  uart2_init(115200);	
													 break;
									case 6:  uart2_init(57600);
													 break;
									case 5:  uart2_init(38400);
													 break;
									case 4:  uart2_init(19200);	
													 break;
									case 3:  uart2_init(9600);
													 break;
									case 2:  uart2_init(4800);	
													 break;
									case 1:  uart2_init(2400);
													 break;
									case 0:  uart2_init(1200);
													 break;				 
								}
				delay_ms(1);
				MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)GPS_RESET,21,1,0);           //DMA1通道7,外设为串口2,存储器为USART_TX_BUF,长度21.		   
//        USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);								
				MYDMA_Enable(DMA1_Stream6,USART_REC_LEN);                                                   //开始DMA传输！					
				while((DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==RESET));
				DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
				delay_ms(500);
			}
//关闭GxGGA语句
	  uart2_init(9600);
		delay_ms(500);	
		MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)CLOSE_GxGGA,16,1,0);            //DMA1通道7,外设为串口2,存储器为USART_TX_BUF,长度16.		    
	  MYDMA_Enable(DMA1_Stream6,16);
		while((DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==RESET));
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);	
		delay_ms(500);
//关闭GxGLL语句			
		MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)CLOSE_GxGLL,16,1,0);            //DMA1通道7,外设为串口2,存储器为USART_TX_BUF,长度16.		    
	  MYDMA_Enable(DMA1_Stream6,16);
		while((DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==RESET));
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);	
		delay_ms(500);
//关闭GxGSA语句			
		MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)CLOSE_GxGSA,16,1,0);            //DMA1通道7,外设为串口2,存储器为USART_TX_BUF,长度16.		    
	  MYDMA_Enable(DMA1_Stream6,16);
		while((DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==RESET));
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);	
    delay_ms(500);		
//关闭GxGSV语句			
		MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)CLOSE_GxGSV,16,1,0);            //DMA1通道7,外设为串口2,存储器为USART_TX_BUF,长度16.		   
	  MYDMA_Enable(DMA1_Stream6,16);
		while((DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==RESET));
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);	
    delay_ms(500);		
//关闭GxRMC语句			
		MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)CLOSE_GxRMC,16,1,0);            //DMA1通道7,外设为串口2,存储器为USART_TX_BUF,长度16.		     
	  MYDMA_Enable(DMA1_Stream6,16);
		while((DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==RESET));
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);	
    delay_ms(500);		
//关闭GxVTGL语句			
		MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)CLOSE_GxVTG,16,1,0);            //DMA1通道7,外设为串口2,存储器为USART_TX_BUF,长度16.		   
	  MYDMA_Enable(DMA1_Stream6,16);
		while((DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==RESET));
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
    delay_ms(500);		
//开启NAV_PVT语句			
		MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)OPEN_NAV_PVT,16,1,0);           //DMA1通道7,外设为串口2,存储器为USART_TX_BUF,长度16.		   
	  MYDMA_Enable(DMA1_Stream6,16);
		while((DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==RESET));
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);	
		delay_ms(500);
//设置刷新率为10hz(M8N)		
		MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)RAT_10,14,1,0);                 //DMA1通道7,外设为串口2,存储器为USART_TX_BUF,长度14.		    
	  MYDMA_Enable(DMA1_Stream6,14);
		while((DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==RESET));
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
		delay_ms(500);
//设置GPS波特率为921600		
		MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)BAUDRATE,28,1,0);               //DMA1通道7,外设为串口2,存储器为USART_TX_BUF,长度28.		     
	  MYDMA_Enable(DMA1_Stream6,28);
		while((DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==RESET));
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
		delay_ms(500);
//设置串口波特率为921600			
    uart2_init(921600);
//自动循环接收GPS模块数据	
	  MYDMA_Config(DMA1_Stream5,DMA_Channel_4,(u32)&USART2->DR,(u32)USART_RX_BUF,USART_REC_LEN,0,1);//DMA1通道6,外设为串口2,存储器为USART_RX_BUF,长度USART_REC_LEN.		
//	  USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);                                      //使能串口2的DMA接收      
	  MYDMA_Enable(DMA1_Stream5,USART_REC_LEN);
    while((DMA_GetFlagStatus(DMA1_Stream5,DMA_FLAG_TCIF5)==RESET));
	 // model_state=model_state&0xEFFF; 
}


//GPS 数据解析
void READ_GPS_DATA(void)
{
//	if((model_state&0x0100)==0x0000)
//	{
//		model_state=model_state|0x1000;
		if ((DMA_GetFlagStatus(DMA1_Stream5,DMA_FLAG_TCIF5)!=RESET))
		{
	   //GPS数据正常
		 DMA_ClearFlag(DMA2_Stream5,DMA_FLAG_TCIF5);
		 GPS.year=(u16)((USART_RX_BUF[11]<<8)|USART_RX_BUF[10]);
		 GPS.month=USART_RX_BUF[12];
		 GPS.day=USART_RX_BUF[13];
		 GPS.hour=USART_RX_BUF[14];
		 GPS.min=USART_RX_BUF[15];
		 GPS.sec=USART_RX_BUF[16];
		 GPS.fixType=USART_RX_BUF[26]; //定位类型
		 GPS.numSV=USART_RX_BUF[29];   //卫星数量 
     if(GPS.fixType>0)
     {			 
		 GPS.time=(u32)((USART_RX_BUF[9]<<24)|(USART_RX_BUF[8]<<16)|(USART_RX_BUF[7]<<8)|(USART_RX_BUF[6]));
		 GPS.Longitude=((double)((USART_RX_BUF[33]<<24)|(USART_RX_BUF[32]<<16)|(USART_RX_BUF[31]<<8)|(USART_RX_BUF[30])));//*0.0000001f;          //deg
		 GPS.Latitude=((double)((USART_RX_BUF[37]<<24)|(USART_RX_BUF[36]<<16)|(USART_RX_BUF[35]<<8)|(USART_RX_BUF[34])));//*0.0000001f;           //deg
		 GPS.height=((float)((USART_RX_BUF[40]<<24)|(USART_RX_BUF[40]<<16)|(USART_RX_BUF[39]<<8)|(USART_RX_BUF[38])))*0.1f;      //海拔  cm/s
		 GPS.hAcc=((float)((USART_RX_BUF[49]<<24)|(USART_RX_BUF[48]<<16)|(USART_RX_BUF[47]<<8)|(USART_RX_BUF[46])))*0.001f;        //垂直估计精度m
		 GPS.vAcc=((float)((USART_RX_BUF[53]<<24)|(USART_RX_BUF[52]<<16)|(USART_RX_BUF[51]<<8)|(USART_RX_BUF[50])))*0.001f;        //水平估计精度m
		 GPS.velN=((float)((USART_RX_BUF[57]<<24)|(USART_RX_BUF[56]<<16)|(USART_RX_BUF[55]<<8)|(USART_RX_BUF[54])))*0.1f;        //正北速度  cm/s
		 GPS.velE=((float)((USART_RX_BUF[61]<<24)|(USART_RX_BUF[60]<<16)|(USART_RX_BUF[59]<<8)|(USART_RX_BUF[58])))*0.1f;        //正东速度  cm/s
		 GPS.velD=((float)((USART_RX_BUF[65]<<24)|(USART_RX_BUF[64]<<16)|(USART_RX_BUF[63]<<8)|(USART_RX_BUF[62])))*0.1f;        //正天速度  cm/s
		 GPS.gSpeed=((float)((USART_RX_BUF[69]<<24)|(USART_RX_BUF[68]<<16)|(USART_RX_BUF[67]<<8)|(USART_RX_BUF[66])))*0.1f;      //地速  cm/s	
		 GPS.sAcc=((float)((USART_RX_BUF[77]<<24)|(USART_RX_BUF[76]<<16)|(USART_RX_BUF[75]<<8)|(USART_RX_BUF[74])))*0.1f;        //速度估计精度cm/s
		 GPS.pDOP=(float)((USART_RX_BUF[83]<<8)|USART_RX_BUF[82]);                                                               //空间位置精度
		 }
		 GPS_ERR=0;
		}
		if((USART_RX_BUF[0]!=0xB5)||(USART_RX_BUF[1]!=0x62)||(USART_RX_BUF[2]!=0x01)||(USART_RX_BUF[3]!=0x07))
		{
			//GPS数据异常，复位接收
			//复位串口DMA接收
			uart2_init(921600);
			MYDMA_Config(DMA1_Stream5,DMA_Channel_4,(u32)&USART2->DR,(u32)USART_RX_BUF,USART_REC_LEN,0,1);//DMA1通道6,外设为串口2,存储器为USART_RX_BUF,长度USART_REC_LEN.		    
			MYDMA_Enable(DMA1_Stream5,USART_REC_LEN);                                                      //开始DMA传输！
			GPS_ERR++;
			if(GPS_ERR==10)
			{
			 GPS_ERR=0;
			 //model_state|=0x0100;
			}
		}
//    model_state=model_state&0xEFFF;
//	}
}
