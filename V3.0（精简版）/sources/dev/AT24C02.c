#include "AT24C02.h"
#include "delay.h"
#include "i2c.h"


#define AT24C02_ADDR 0xA0  //AT24c02  IIC地址


#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

#define START_ADDR 0  //256字节 //起始地址
#define  STOP_ADDR 0XFF //结束地址

u8 AT24C02_ByteCurrentRead()
{
	u8 temp=0;
	I2c_Soft_Start();
	I2c_Soft_SendByte(AT24C02_ADDR+1);   
	if(I2c_Soft_WaitAsk() == FAILED)
	{
		I2c_Soft_Stop();
		return FAILED;
	}
	temp=I2c_Soft_ReadByte(0);
	I2c_Soft_Stop();
	return temp;
}

void AT24C02_NByteRandomRead(u8 readAddr ,u8 len,u8* pBuffer)
{
	IIC_Read_nByte(AT24C02_ADDR,readAddr,len,pBuffer);
}

u8 AT24C02_NByteSequentialRead(u8 len,u8* pBuffer)
{
	I2c_Soft_Start();
	I2c_Soft_SendByte(AT24C02_ADDR+1);   
	if(I2c_Soft_WaitAsk() == FAILED)
	{
		I2c_Soft_Stop();
		return FAILED;
	}
  while(len) 
	{
		if(len == 1)
		{
			*pBuffer = I2c_Soft_ReadByte(0);
		}
		else
		{
			*pBuffer = I2c_Soft_ReadByte(1);
		}
		pBuffer++;
		len--;
	}
  I2c_Soft_Stop();
  return SUCCESS;
}


uint8_t AT24C02_Test(void)//测试
{
	uint8_t test = 0;
  AT24C02_ByteWrite(0xFF,0xAA);
	
	AT24C02_NByteRandomRead(0xFF,1,&test);
	if(test != 0xAA) //存储器异常
		return FAILED;
	else
		return SUCCESS;
}
	


/**
 * @brief  写函数，配置I2C接口
 * @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
 * @param  writeAddr : address of the register in which the data will be written
 * @return None
 */
void AT24C02_ByteWrite(u8 writeAddr ,u8 data)
{
   IIC_Write_1Byte(AT24C02_ADDR,writeAddr,data);
	 delay_ms(2);
}

void AT24C02_PageWrite(u8 WritePage,u8* pBuffer)
{
	IIC_Write_nByte(AT24C02_ADDR, 8*WritePage,8, pBuffer);
	delay_ms(2);
}

void SaveFloatData(const float DataToSave,uint8_t pos)
{
	u8 *point_temp;
	u8 temp[4];
	point_temp=(u8*)&DataToSave;
	for(u8 i=0;i<4;i++)
	{
		AT24C02_ByteWrite(pos+i ,*(point_temp+i));
	}
}

float ReadFloatData(uint8_t pos)
{
	u8 ReadDataBuf[4]={0};
	float OutputData=0;
	AT24C02_NByteRandomRead(pos,4,ReadDataBuf);
	OutputData=*((float*)ReadDataBuf);
	return OutputData;
}


