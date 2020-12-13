#include "spl06.h"
#include "i2c.h"
#include "delay.h"
#include "math.h"

#define sensor_add (0x77<<1)
#define kT 3670016.0f
//#define kP 1572864.0f

#define kP 3670016.0f

static int16_t c0,c1,c01,c11,c20,c21,c30;
static int32_t c00,c10;

static u8 SPL06_ReadByte(uint8_t regAddr);
static void SPL06_ReadNByte(uint8_t regAddr,uint8_t len,  uint8_t *data);
static void SPL06_ByteWrite(u8 writeAddr ,u8 data);
	

void SPL06_Init()
{
	u8 coefficients[18];
	u8 ID=0;
	
	ID=SPL06_ReadID();
	
	SPL06_ByteWrite(0x0C,0x89);    //reset
	delay_ms(100);
	
	SPL06_ReadNByte(0x10,18,coefficients);
	c0=(coefficients[0]<<4)|((coefficients[1]&0xF0)>>4);
	c0=(c0&0x0800)?(0xF000|c0):c0;
	c1=((coefficients[1]&0x0F)<<8)|coefficients[2];
	c1=(c1&0x0800)?(0xF000|c1):c1;
	c00=(coefficients[3]<<12)|(coefficients[4]<<4)|(coefficients[5]>>4);
	c10=((coefficients[5]&0x0F)<<16)|(coefficients[6]<<8)|(coefficients[7]);
	c00 = (c00&0x080000)?(0xFFF00000|c00):c00;
  c10 = (c10&0x080000)?(0xFFF00000|c10):c10;
	c01=(coefficients[8]<<8)|coefficients[9];
	c11=(coefficients[10]<<8)|coefficients[11];
	c20=(coefficients[12]<<8)|coefficients[13];
	c21=(coefficients[14]<<8)|coefficients[15];
	c30=(coefficients[16]<<8)|coefficients[17];
	
	SPL06_ByteWrite(0x06,0x72);    //4times oversampling  128time pre sec
	//SPL06_ByteWrite(0x07,0x72);
	SPL06_ByteWrite(0x07,0xF2);    //4times oversampling  128time pre sec
	SPL06_ByteWrite(0x08,0x07);    //开启测量
	SPL06_ByteWrite(0x09,0x00);
	delay_ms(20);
}

float SPL06_GetPressureData()
{
	u8 Sensor_RawData[6]={0};
	int32_t Pressure_RawData0,Temperature_RawData0;
	float Pressure_RawData,Temperature_RawData,Pressure_Output,Temperature;
	SPL06_ReadNByte(0x00,6,Sensor_RawData);
	Pressure_RawData0=(Sensor_RawData[0]<<16)|(Sensor_RawData[1]<<8)|(Sensor_RawData[2]);
	Temperature_RawData0=(Sensor_RawData[3]<<16)|(Sensor_RawData[4]<<8)|(Sensor_RawData[5]);
	
	Pressure_RawData0=(Pressure_RawData0&0x800000)?(0xFF000000|Pressure_RawData0):Pressure_RawData0;
	Temperature_RawData0=(Temperature_RawData0&0x800000)?(0xFF000000|Temperature_RawData0):Temperature_RawData0;
	
	Pressure_RawData=(float)Pressure_RawData0/kP;
	Temperature_RawData=(float)Temperature_RawData0/kT;
	Pressure_Output=c00+Pressure_RawData*(c10+Pressure_RawData*(c20+Pressure_RawData*c30))+Temperature_RawData*c01+Temperature_RawData*Pressure_RawData*(c11+Pressure_RawData*c21);
	Temperature=c0*0.5f+c1*Temperature_RawData;
	return Pressure_Output;
}

#define Pa2mmHg 0.0075006f
float SPL06_GetHeight()
{
	float P,Height;
	P=SPL06_GetPressureData();
	P=P*Pa2mmHg;
	Height=-7924.0f*log(P/760.0f);
	return Height;
	
}


u8 SPL06_ReadID()
{
	u8 ID=0;
	ID=SPL06_ReadByte(0x0D);
	return ID;
}































/** 读数据函数，配置I2C接口
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for single bit value
 */
static u8 SPL06_ReadByte(uint8_t regAddr)
{
	u8 data;
	 IIC_Read_1Byte(sensor_add,regAddr,&data);
	return data;
}

static void SPL06_ReadNByte(uint8_t regAddr,uint8_t len,  uint8_t *data)
{
	 IIC_Read_nByte(sensor_add, regAddr, len, data);

}

/**
 * @brief  写函数，配置I2C接口
 * @param  slaveAddr : slave address SPL06_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the SPL06.
 * @param  writeAddr : address of the register in which the data will be written
 * @return None
 */
static void SPL06_ByteWrite(u8 writeAddr ,u8 data)
{
    IIC_Write_1Byte(sensor_add,writeAddr,data);
}
