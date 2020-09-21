#ifndef  _____SPI____H___
#define  _____SPI____H___




#define SPI_CS_H         GPIOB->BSRRL = GPIO_Pin_12
#define SPI_CS_L         GPIOB->BSRRH = GPIO_Pin_12
#define EXIT_1_H         GPIOD->BSRRL = GPIO_Pin_8
#define EXIT_1_L         GPIOD->BSRRH = GPIO_Pin_8  
#define EXIT_2_H         GPIOD->BSRRL = GPIO_Pin_9
#define EXIT_2_L         GPIOD->BSRRH = GPIO_Pin_9 



void SPI2_Init(void);


#endif

