#include "AK8975.h"
#include "i2c.h"
#include "delay.h"

#define	MAG_ADDR 0x18   //IIC写入时的地址字节数据，+1为读取

static u8 ASA[3]={0};
static float raw_mag[3]={0};

static void AK8975_ReadByte(uint8_t slaveAddr, uint8_t regAddr,  uint8_t *data);                //读一位寄存器
static void AK8975_ReadNByte(uint8_t slaveAddr, uint8_t regAddr,uint8_t len,  uint8_t *data);   //读多为寄存器
static void AK8975_ByteWrite(u8 slaveAddr,u8 writeAddr ,u8 data);                           //写一位寄存器
static float Calculate2Hadj(s16 Data,u8 axis);

void AK8975_Init()
{
	
  uint8_t temp[3];
  AK8975_ByteWrite(MAG_ADDR,AK8975_CNTL,0X0F);
  AK8975_ReadNByte(MAG_ADDR,AK8975_ASAX,3,temp);
  ASA[0]=temp[0];
  ASA[1]=temp[1];
  ASA[2]=temp[2];
  AK8975_ByteWrite(MAG_ADDR,AK8975_CNTL,0X00);
}


u8 AK8975_GetID()
{
	u8 temp=0;
	AK8975_ReadByte(MAG_ADDR, AK8975_WIA, &temp);                //读一位寄存器
	return temp;
}

static float Calculate2Hadj(s16 Data,u8 axis)
{
	float Hadj=0;
	Hadj=Data*(((float)(ASA[axis]-128))/256+1);
	return Hadj;
}

void AK8975_GetMagData(AK8975Data_Type*MagData)
{
	u8 temp[6]={0};
	AK8975_ReadNByte(MAG_ADDR, AK8975_HXL,6,  temp);   //读多为寄存器
	for(u8 i=0;i<3;i++)
	{
		u16 temp1=0;
		temp1=((temp[2*i+1]<<8)+temp[2*i]);
		raw_mag[i]=Calculate2Hadj((s16)temp1,i);
		MagData->mag[i]=MagData->mag_an[i]*((float)temp1/4096*1229-MagData->mag_offset[i]);
//		MagData->mag[i]=raw_mag[i];
	}
}

void AK8975_MagXYCalibration(AK8975Data_Type*MagData)    //XY水平校准
{
	u8 temp[4]={0};
	static float X_max=0,X_min=0;
	static float Y_max=0,Y_min=0;
	AK8975_ReadNByte(MAG_ADDR, AK8975_HXL,4,temp);   //读多为寄存器
	for(u8 i=0;i<2;i++)
	{
		u16 temp1=0;
		float temp2=0;
		temp1=(temp[2*i+1]<<8)+temp[2*i];
		temp2=Calculate2Hadj((s16)temp1,i);
		if(i==0)
		{
			if(temp2>X_max) X_max=temp2;
			if(temp2<X_min) X_min=temp2;
			MagData->mag_offset[i]=(X_max+X_min)/2;
			MagData->mag_an[i]=(X_max-X_min)/2;
		}
		if(i==1)
		{
			if(temp2>Y_max) Y_max=temp2;
			if(temp2<Y_min) Y_min=temp2;
			MagData->mag_offset[i]=(Y_max+Y_min)/2;
			MagData->mag_an[i]=(Y_max-Y_min)/2;
		}
	}
	MagData->Global_B0x=(X_max-X_min+Y_max-Y_min)/4;
}

void AK8975_MagZCalibration(AK8975Data_Type*MagData)    //z轴校准
{
	u8 temp[2]={0};
	u16 temp1=0;
	float temp2=0;
	static float Z_max=0,Z_min=0;
	AK8975_ReadNByte(MAG_ADDR,AK8975_HZL,2,temp);   //读多为寄存器

	temp1=(temp[1]<<8)+temp[0];
	temp2=Calculate2Hadj((s16)temp1,2);
	if(temp2>Z_max) Z_max=temp2;
	if(temp2<Z_min) Z_min=temp2;
	MagData->mag_offset[2]=(Z_max+Z_min)/2;
	MagData->mag_an[2]=(Z_max-Z_min)/2;
}

void AK8975_GetGlobal0z(AK8975Data_Type*MagData)    //获取垂直磁力
{
	u8 temp[4]={0};
	static float Z_max=0,Z_min=0;
	static float Y_max=0,Y_min=0;
	AK8975_ReadNByte(MAG_ADDR,AK8975_HYL,4,temp);   //读多为寄存器
	for(u8 i=0;i<2;i++)
	{
		u16 temp1=0;
		float temp2=0;
		temp1=((temp[2*i+1]<<8)+temp[2*i])&0x8FFF;
		temp2=temp1*((ASA[i+1]-128)/256+1);
		if(i==0)
		{
			if(temp2>Y_max) Y_max=temp2;
			if(temp2<Y_min) Y_min=temp2;
		}
		if(i==1)
		{
			if(temp2>Z_max) Z_max=temp2;
			if(temp2<Z_min) Z_min=temp2;
		}
	}
	MagData->Global_B0z=(Z_max-Z_min+Y_max-Y_min)/4;
}


/** 读数据函数，配置I2C接口
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for single bit value
 */
static void AK8975_ReadByte(uint8_t slaveAddr, uint8_t regAddr,  uint8_t *data)
{
	  IIC_Read_1Byte(slaveAddr,regAddr,data);
}

static void AK8975_ReadNByte(uint8_t slaveAddr, uint8_t regAddr,uint8_t len,  uint8_t *data)
{
	if(regAddr==AK8975_HXL||regAddr==AK8975_HYL||regAddr==AK8975_HZL)
	{
		u8 flag=0;
		u8 count=0;
		AK8975_ByteWrite(MAG_ADDR,AK8975_CNTL,0x11);
		while(flag!=1)
		{
			if(count>100){AK8975_ByteWrite(MAG_ADDR,AK8975_CNTL,0x11);count=0;}
			AK8975_ReadByte(MAG_ADDR, AK8975_ST1, &flag);
			count++;
		}
	}
	IIC_Read_nByte(slaveAddr, regAddr, len, data);
}

/**
 * @brief  写函数，配置I2C接口
 * @param  slaveAddr : slave address AK8975_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the AK8975.
 * @param  writeAddr : address of the register in which the data will be written
 * @return None
 */
static void AK8975_ByteWrite(u8 slaveAddr,u8 writeAddr ,u8 data)
{
    IIC_Write_1Byte(slaveAddr,writeAddr,data);
}

