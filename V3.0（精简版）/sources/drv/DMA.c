#include "dma.h"																	   	  
#include "delay.h"	

//u8 USART_TX=0;
u16 DMA1_MEM_LEN;//����DMAÿ�����ݴ��͵ĳ��� 	
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
	
/********************************************DMAͨ������*****************************************/
//DMAx�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMAͨ��ѡ��,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//par:�����ַ
//mar:�洢����ַ
//ndtr:���ݴ�����  
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr,u8 model,u8 dir)
{ 
 
	DMA_InitTypeDef  DMA_InitStructure;
	
	if((u32)DMA_Streamx>(u32)DMA2)//�õ���ǰstream������DMA2����DMA1
	{
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2ʱ��ʹ�� 
		
	}else 
	{
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1ʱ��ʹ�� 
	}
  DMA_DeInit(DMA_Streamx);
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}//�ȴ�DMA������ 
	
  /* ���� DMA Stream */
  DMA_InitStructure.DMA_Channel = chx;  //ͨ��ѡ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = par;//DMA�����ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA �洢��0��ַ
	if(dir==1) DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                //���ݴ��䷽�򣬴������ȡ���ڴ�
	else DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                      //���ݴ��䷽�򣬴��ڴ��ȡ������
//  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�洢��������ģʽ
  DMA_InitStructure.DMA_BufferSize = ndtr;//���ݴ����� 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
	if(model==1) DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                   //�����ڵ���ģʽ
	else DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         //����������ģʽ
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ����ͨģʽ 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�е����ȼ�
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
  DMA_Init(DMA_Streamx, &DMA_InitStructure);//��ʼ��DMA Stream
	

} 
//����һ��DMA����
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:���ݴ�����  
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                      //�ر�DMA���� 
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//ȷ��DMA���Ա�����  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //���ݴ�����  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //����DMA���� 
}	  

 
