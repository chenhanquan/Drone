#include "dma.h"																	   	  
#include "delay.h"	

//u8 USART_TX=0;
u16 DMA1_MEM_LEN;//保存DMA每次数据传送的长度 	
//Butter_BufferData 	flow_BufferData[2];


//Butter_Parameter Butter_60HZ_Parameter={
//  //200hz---60hz
//1,   0.3695273773512,   0.1958157126558,
//0.3913357725018,   0.7826715450035,   0.3913357725018
//};

//Butter_Parameter Butter_50HZ_Parameter={
//  //200hz---50hz
//1,-1.300707181133e-16,   0.1715728752538,
//0.2065720838261,   0.4131441676523,   0.2065720838261,
//};

//Butter_Parameter Butter_30HZ_Parameter={
//  //200hz---30hz
//1,  -0.7477891782585,    0.272214937925,
//0.1311064399166,   0.2622128798333,   0.1311064399166
//};

//Butter_Parameter Butter_20HZ_Parameter={
//  //200hz---20hz
//  1,    -1.14298050254,   0.4128015980962,
//  0.06745527388907,   0.1349105477781,  0.06745527388907
//};
//Butter_Parameter Butter_15HZ_Parameter={
//  //200hz---15hz
//  1,   -1.348967745253,   0.5139818942197,
//  0.04125353724172,  0.08250707448344,  0.04125353724172
//};

//Butter_Parameter Butter_10HZ_Parameter={
//  //200hz---10hz
//  1,   -1.561018075801,   0.6413515380576,
//  0.02008336556421,  0.04016673112842,  0.02008336556421};

//Butter_Parameter Butter_5HZ_Parameter={
//  //200hz---5hz
//  1,   -1.778631777825,   0.8008026466657,
//  0.005542717210281,  0.01108543442056, 0.005542717210281};


//	Butter_Parameter Butter_1000_6HZ_Parameter={
//  //1000hz---6hz
//   1,   -1.946697540756,   0.9480817061067,
// 0.0003460413376391,0.0006920826752782,0.0003460413376391};
	
/********************************************DMA通道配置*****************************************/
//DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//par:外设地址
//mar:存储器地址
//ndtr:数据传输量  
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr,u8 model,u8 dir)
{ 
 
	DMA_InitTypeDef  DMA_InitStructure;
	
	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
		
	}else 
	{
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	}
  DMA_DeInit(DMA_Streamx);
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}//等待DMA可配置 
	
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = chx;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = par;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA 存储器0地址
	if(dir==1) DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                //数据传输方向，从外设读取到内存
	else DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                      //数据传输方向，从内存读取到外设
//  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = ndtr;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
	if(model==1) DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                   //工作在单次模式
	else DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         //工作在连续模式
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA_Streamx, &DMA_InitStructure);//初始化DMA Stream
	

} 
//开启一次DMA传输
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:数据传输量  
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
}	  

 
