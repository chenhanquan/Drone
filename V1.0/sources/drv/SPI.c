#include "stm32f4xx.h"
#include "SPI.h"





#define SPI_CS_H         GPIOB->BSRRL = GPIO_Pin_12
#define SPI_CS_L         GPIOB->BSRRH = GPIO_Pin_12
#define EXIT_1_H         GPIOD->BSRRL = GPIO_Pin_8
#define EXIT_1_L         GPIOD->BSRRH = GPIO_Pin_8  
#define EXIT_2_H         GPIOD->BSRRL = GPIO_Pin_9
#define EXIT_2_L         GPIOD->BSRRH = GPIO_Pin_9 



void SPI2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    //SPI GPIO Configuration
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource13, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource14, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_SPI2);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // SPI CS
    GPIO_InitStructure.GPIO_Pin             = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode            = GPIO_Mode_OUT ;   //推挽输出
    GPIO_InitStructure.GPIO_OType           = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd            = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed           = GPIO_Speed_50MHz;		
    GPIO_Init(GPIOB, &GPIO_InitStructure);
		// SPI EXIT_1  // SPI EXIT_2
		GPIO_InitStructure.GPIO_Pin             = GPIO_Pin_8 | GPIO_Pin_9; 
		GPIO_Init(GPIOD, &GPIO_InitStructure);


    //SPI configuration
    SPI_I2S_DeInit(SPI2);
    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI2, &SPI_InitStructure);
    SPI_Cmd(SPI2, ENABLE);
}




u8 ANO_SPI_RW(u8 dat) 
{ 
		uint8_t cnt;
		/* 当 SPI发送缓冲器非空时等待 */ 
		while ((SPI2->SR & SPI_I2S_FLAG_TXE) == RESET)
		{
			cnt++;
			if(cnt>200)
			{
				return 0;
			}
		}
		cnt=0;
		/* 通过 SPI2发送一字节数据 */ 
		 SPI2->DR = dat;
		/* 当SPI接收缓冲器为空时等待 */ 
		while ((SPI2->SR & SPI_I2S_FLAG_RXNE) == RESET)
		{
			cnt++;
			if(cnt>200)
			{
				return 0;
			}
		}			
		/* Return the byte read from the SPI bus */ 
		return SPI2->DR;
}








