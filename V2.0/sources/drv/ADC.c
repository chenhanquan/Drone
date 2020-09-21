#include "stm32f4xx.h"
#include "ADC.h"

uint16_t ADC_Converted_Value[2];

/********************************************************************* 
*                           初始ADC 
**********************************************************************/  
  
void ADC3_Init(void)  
{  
    GPIO_InitTypeDef GPIO_InitStructure;  
    DMA_InitTypeDef DMA_InitStructure;    
    ADC_InitTypeDef ADC_InitStructure;    
    ADC_CommonInitTypeDef ADC_CommonInitStructure;  
  
    //采样脚IO口设置     
    //初始化时钟  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
    //管脚模式:输入口  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;      
    //上拉下拉设置  
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      
    //管脚指定  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;  
    //初始化  
    GPIO_Init(GPIOC, &GPIO_InitStructure);  
  
    //DMA设置  
    //启动DMA时钟    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  
    //DMA通道配置  
    DMA_DeInit(DMA2_Stream0);  
    DMA_InitStructure.DMA_Channel = DMA_Channel_2;   
    //外设地址  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&ADC3->DR);  
    //内存地址  
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADC_Converted_Value;  
    //dma传输方向  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;  
    //设置DMA在传输时缓冲区的长度  
    DMA_InitStructure.DMA_BufferSize = 2;  
    //设置DMA的外设递增模式，一个外设  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    //设置DMA的内存递增模式  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    //外设数据字长  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  
    //内存数据字长  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  
    //设置DMA的传输模式  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  
    //设置DMA的优先级别  
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
      
    //指定如果FIFO模式或直接模式将用于指定的流:不使能FIFO模式    
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;      
    //指定了FIFO阈值水平  
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;          
    //指定的Burst转移配置内存传输   
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         
    //指定的Burst转移配置外围转移 */    
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
      
    //配置DMA的通道           
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);    
    //使能通道  
    DMA_Cmd(DMA2_Stream0, ENABLE);        
  
    //ADC配置    
    //启动ADC时钟    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);  
    //初始化ADC  
    ADC_DeInit();  
    
    //ADC通用配置  
    //关闭DMA存取  
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;    
    //独立工作模式  
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;  
    //分频系数:8,10.5M  
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;  
    //ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;  
    //两次采样间隔  
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;  
    //ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;  
    ADC_CommonInit(&ADC_CommonInitStructure);     
  
    //精度为12位  
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;  
    //扫描方式    
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;     
    //ADC_InitStructure.ADC_ScanConvMode = DISABLE;  
    //连续转换    
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;    
    //外部触发禁止    
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;    
    //数据右对齐    
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;    
    //用于转换的通道数    
    ADC_InitStructure.ADC_NbrOfConversion = 1;    
    ADC_Init(ADC3, &ADC_InitStructure);    
        
    //规则模式通道配置        
    ADC_RegularChannelConfig(ADC3, ADC_Channel_9, 1, ADC_SampleTime_144Cycles);  
    //ADC_RegularChannelConfig(ADC3, ADC_Channel_9, 1, ADC_SampleTime_15Cycles);  
  
    //使能ADC的DMA    
    ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);  
    ADC_DMACmd(ADC3, ENABLE);   
      
    //使能ADC1   
    ADC_Cmd(ADC3, ENABLE);    
        
    //开启ADC1的软件转换  
    ADC_SoftwareStartConv(ADC3);       
}





