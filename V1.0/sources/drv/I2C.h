#ifndef _I2C_SOFT_H
#define	_I2C_SOFT_H



#include "stm32f4xx.h"


/***************I2C GPIO定义******************/
#define ANO_GPIO_I2C	GPIOB
#define I2C_Pin_SCL		GPIO_Pin_6
#define I2C_Pin_SDA		GPIO_Pin_7
#define ANO_RCC_I2C		RCC_AHB1Periph_GPIOB
/*********************************************/
#define SCL_H         ANO_GPIO_I2C->BSRRL = I2C_Pin_SCL
#define SCL_L         ANO_GPIO_I2C->BSRRH = I2C_Pin_SCL
#define SDA_H         ANO_GPIO_I2C->BSRRL = I2C_Pin_SDA
#define SDA_L         ANO_GPIO_I2C->BSRRH = I2C_Pin_SDA
#define SCL_read      ANO_GPIO_I2C->IDR  & I2C_Pin_SCL
#define SDA_read      ANO_GPIO_I2C->IDR  & I2C_Pin_SDA
extern volatile u8 I2C_FastMode;



void Hard_IIC_Init(void);
void I2c_Soft_Init(void);




u8 Hard_IICWriteOneByte(uint8_t SlaveAdd, u8 WriteAdd, u8 Data);
u8 Hard_IIC_ReadOneByte(uint8_t SlaveAdd, u8 ReadAdd,u8 *REG_data);
u8 Hard_IIC_WriteNByte(u8 SlaveAdd, u8 WriteAdd, u8 NumToWrite , u8 * pBuffer);
u8 Hard_IIC_ReadNByte(u8 SlaveAdd, u8 ReadAdd,u8 NumToWrite ,u8 * pBuffer);



u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);
u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);


#ifdef I2C_HARDWARE //使用硬件I2C
#define IIC_Write_One_Byte Hard_IICWriteOneByte
#define IIC_Read_One_Byte  Hard_IIC_ReadOneByte
#define IIC_read_Bytes     Hard_IIC_ReadNByte
#define  I2c_Init Hard_IIC_Init
#else  //使用软件I2C
#define IIC_Write_One_Byte IIC_Write_1Byte
#define IIC_Read_One_Byte  IIC_Read_1Byte
#define IIC_read_Bytes     IIC_Read_nByte
#define  I2c_Init I2c_Soft_Init
#endif




#endif




