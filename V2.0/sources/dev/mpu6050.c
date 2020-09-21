/**
  ******************************************************************************
  * @file    mpu6050.c
  * @author  CHEN HANQUAN
  * @version V1.0
  * @date    16-August-2020
  * @brief   mpu6050模块控制函数
  * @note    
	******************************************************************************
  */
#include "mpu6050.h"
#include "filter.h"

#define GYRO_FILTER_NUM 3
#define ACC_FILTER_NUM 12

#define GYRO_FULL_SCALE 1000
#define ACC_FULL_SCALE 8

static void MPU6050_ReadByte(uint8_t slaveAddr, uint8_t regAddr,  uint8_t *data);                //读一位寄存器
static void MPU6050_ReadNByte(uint8_t slaveAddr, uint8_t regAddr,uint8_t len,  uint8_t *data);   //读多为寄存器
static void MPU6050_ByteWrite(u8 slaveAddr,u8 writeAddr ,u8 data);                               //写一位寄存器
static void MPU6050_AccFilter(MPU6050Data_TypeDef *MPU6050Data);                                 //加速度滤波
static void MPU6050_GyroFilter(MPU6050Data_TypeDef *MPU6050Data);                                //角速度滤波
static void MPU6050_GyroRawToRadian(MPU6050Data_TypeDef *MPU6050Data);                           //角速度原始数据转换为弧度
static void MPU6050_AccRawToMS2(MPU6050Data_TypeDef *MPU6050Data);                               //加速度单位转换

/** 读数据函数，配置I2C接口
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for single bit value
 */
static void MPU6050_ReadByte(uint8_t slaveAddr, uint8_t regAddr,  uint8_t *data)
{
	  IIC_Read_1Byte(slaveAddr,regAddr,data);
}

static void MPU6050_ReadNByte(uint8_t slaveAddr, uint8_t regAddr,uint8_t len,  uint8_t *data)
{
	  IIC_Read_nByte(slaveAddr, regAddr, len, data);
}

/**
 * @brief  写函数，配置I2C接口
 * @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
 * @param  writeAddr : address of the register in which the data will be written
 * @return None
 */
static void MPU6050_ByteWrite(u8 slaveAddr,u8 writeAddr ,u8 data)
{
    IIC_Write_1Byte(slaveAddr,writeAddr,data);
}

/**
 * @brief  MPU6050初始化
 * @return None
 */
void MPU6050_Init()
{ 
	MPU6050_ByteWrite(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_PWR_MGMT_1, 0x01);      //时钟选择
	MPU6050_ByteWrite(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_SMPLRT_DIV, 0x00);      // sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz
//	MPU6050_ByteWrite(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_CONFIG, 0x02);          //内部低通滤波频率，98hz
	MPU6050_ByteWrite(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_GYRO_CONFIG, 0x10);     //1000deg/s
	MPU6050_ByteWrite(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_CONFIG, 0x10);    //加速度 正负8g
}

/**
 * @brief  读取ID
 * @return None
 */
uint8_t MPU6050_GetDeviceID()
{
    uint8_t tmp=0;
    MPU6050_ReadByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I,&tmp);
    return tmp;
}

/**
 * @brief  读取加速度和角速度函数
 * @param  MPU6050Data：用于存储数据的结构体
 * @return None
 */
void MPU6050_GetGyroAccData(MPU6050Data_TypeDef *MPU6050Data)
{
	uint8_t temp[14] = {0};
	for(u8 i;i<3;i++){MPU6050Data->preacc[i]=MPU6050Data->acc[i];MPU6050Data->pregyro[i]=MPU6050Data->gyro[i];}
	MPU6050_ReadNByte(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_XOUT_H,14,temp);
	/* Get acceleration */
  for (int i = 0; i < 3; i++)
	{
    MPU6050Data->acc[i] = ((float)((s16) ((u16) temp[2 * i] << 8) + temp[2 * i + 1]))/32767*ACC_FULL_SCALE-MPU6050Data->acc_offset[i];
	}
	/* Get Angular rate */
  for (int i = 4; i < 7; i++)
	{
    MPU6050Data->gyro[i-4] = ((float)((s16) ((u16) temp[2 * i] << 8) + temp[2 * i + 1]))/32767*GYRO_FULL_SCALE-MPU6050Data->gyro_offset[i-4];
	}
}


/**
 * @brief  MPU6050校准
 * @param  MPU6050Data：用于存储数据的结构体
 * @return None
 */
void MPU6050_Calibration(MPU6050Data_TypeDef *MPU6050Data)
{
	u8 CalibraTime = 20;    //校准次数
	float CaliData[6];
	for(u8 j=0;j<CalibraTime;j++)
	{
  uint8_t temp[14] = {0};
	MPU6050_ReadNByte(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_XOUT_H,14,temp);
	/* Get acceleration */
  for (int i = 0; i < 3; i++)
	{
   CaliData[i] += ((float)((s16) ((u16) temp[2 * i] << 8) + temp[2 * i + 1]))/32767*ACC_FULL_SCALE;
	}
	/* Get Angular rate */
  for (int i = 4; i < 7; i++)
	{
   CaliData[i-1] += ((float)((s16) ((u16) temp[2 * i] << 8) + temp[2 * i + 1]))/32767*GYRO_FULL_SCALE;
	}
	}
	CaliData[2] -= CalibraTime;    //加速度计z轴零点修正
	for(int i=0;i<3;i++)
	{
		MPU6050Data->acc_offset[i] = CaliData[i]/CalibraTime;
		MPU6050Data->gyro_offset[i] = CaliData[i+3]/CalibraTime;
	}
}

static void MPU6050_AccFilter(MPU6050Data_TypeDef *MPU6050Data)
{
  static float LastAccHandle[3]={0};
	static float Kp_now=0.7,Kp_last=0.3;
  for(u8 i=0;i<3;i++){MPU6050Data->acchandle[i]=MPU6050Data->acc[i];}  
	SortAver_FilterXYZ(MPU6050Data->acc,MPU6050Data->acchandle,12);    //加速度去最值滤波
	for(u8 i=0;i<3;i++)
	{
		MPU6050Data->acchandle[i]=MPU6050Data->acchandle[i]*Kp_now+LastAccHandle[i]*Kp_last;
		LastAccHandle[i]=MPU6050Data->acchandle[i];
	}
}

static void MPU6050_GyroFilter(MPU6050Data_TypeDef *MPU6050Data)
{
	static float Filter[3][GYRO_FILTER_NUM];
	static u8 FilterCount=0;
	float FilterSum[3]={0};
	for(u8 i=0;i<3;i++){Filter[i][FilterCount]=MPU6050Data->gyro[i];}
	for(u8 i=0;i<3;i++)
	{
	for(u8 j=0;j<GYRO_FILTER_NUM;j++){FilterSum[i]+=Filter[i][j];}
	}
	for(u8 i=0;i<3;i++){MPU6050Data->gyrohandle[i]=FilterSum[i]/GYRO_FILTER_NUM;}
	FilterCount++;
	if(FilterCount==GYRO_FILTER_NUM) FilterCount=0;
}

static void MPU6050_GyroRawToRadian(MPU6050Data_TypeDef *MPU6050Data)
{
	static float RawToRadian=0.0174533f;
	for(u8 i=0;i<3;i++){MPU6050Data->gyrohandle[i]=MPU6050Data->gyrohandle[i]*RawToRadian;}
}

static void MPU6050_AccRawToMS2(MPU6050Data_TypeDef *MPU6050Data)
{
	static float G=9.80665f;
	for(u8 i=0;i<3;i++){MPU6050Data->acchandle[i]=MPU6050Data->acchandle[i]*G;}
}

void MPU6050_RawDataHandle(MPU6050Data_TypeDef *MPU6050Data)
{
	MPU6050_AccFilter(MPU6050Data);
	MPU6050_AccRawToMS2(MPU6050Data);
	MPU6050_GyroFilter(MPU6050Data);
	MPU6050_GyroRawToRadian(MPU6050Data);
}



