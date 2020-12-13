 /**
  ******************************************************************************
  * @file    mpu6050.c
  * @author  CHEN HANQUAN
  * @version V1.0
  * @date    16-August-2020
  * @brief   mpu6050ģ����ƺ���
  * @note    
	******************************************************************************
  */
#include "mpu6050.h"
#include "arm_math.h"
#include "delay.h"

#define GRAVITY_MSS			9.80665f		      // m/s^2
#define DegToRadian      0.0174533f;

#define GYRO_FILTER_NUM 10
#define ACC_FILTER_NUM 12

#define GYRO_FULL_SCALE 1000
#define ACC_FULL_SCALE 8

static void MPU6050_ReadByte(uint8_t slaveAddr, uint8_t regAddr,  uint8_t *data);                //��һλ�Ĵ���
static void MPU6050_ReadNByte(uint8_t slaveAddr, uint8_t regAddr,uint8_t len,  uint8_t *data);   //����Ϊ�Ĵ���
static void MPU6050_ByteWrite(u8 slaveAddr,u8 writeAddr ,u8 data);                               //дһλ�Ĵ���

static void MPU6050_MinusOffset(float *raw_data,float *data_offset);
static void MPU6050_GetRawData(float raw_acc_data[3],float raw_gyro_data[3]);
static void MPU6050_GyroDegToRadian(float DegData[3],float RadianData[3]);                           //���ٶ�ԭʼ����ת��Ϊ����
static void MPU6050_AccGToMS2(float G_Data[3],float MS2Data[3]);                               //���ٶȵ�λת��


/** �����ݺ���������I2C�ӿ�
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
 * @brief  д����������I2C�ӿ�
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
 * @brief  MPU6050��ʼ��
 * @return None
 */
void MPU6050_Init()
{ 
	MPU6050_ByteWrite(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_PWR_MGMT_1, 0x01);      //ʱ��ѡ��
	MPU6050_ByteWrite(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_SMPLRT_DIV, 0x00);      // sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz
	MPU6050_ByteWrite(MPU6050_DEFAULT_ADDRESS,0x1D, 0x04);     //20Hz�˲� ���ٶ�
	MPU6050_ByteWrite(MPU6050_DEFAULT_ADDRESS,0x1A, 0x03);     //40Hz   ���ٶ�
	MPU6050_ByteWrite(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_CONFIG, 0x10);    //���ٶ� ����8g
	MPU6050_ByteWrite(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_GYRO_CONFIG, 0x10);

}

/**
 * @brief  ��ȡID
 * @return None
 */
uint8_t MPU6050_GetDeviceID()
{
    uint8_t tmp=0;
    MPU6050_ReadByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I,&tmp);
    return tmp;
}

static void MPU6050_GetRawData(float raw_acc_data[3],float raw_gyro_data[3])
{
	uint8_t temp[14] = {0};
	float raw_acc_xyz[3]={0},raw_gyro_xyz[3]={0};
	MPU6050_ReadNByte(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_XOUT_H,14,temp);
	/* Get acceleration */
  for (int i = 0; i < 3; i++)
	{
    raw_acc_xyz[i] = ((float)((s16) ((u16) temp[2 * i] << 8) + temp[2 * i + 1]))/32767*ACC_FULL_SCALE;
	}
	/* Get Angular rate */
  for (int i = 4; i < 7; i++)
	{
    raw_gyro_xyz[i-4] = ((float)((s16) ((u16) temp[2 * i] << 8) + temp[2 * i + 1]))/32767*GYRO_FULL_SCALE;
	}
	raw_acc_data[0]=-raw_acc_xyz[1];    //оƬ����ת����������
	raw_acc_data[1]=-raw_acc_xyz[0];
	raw_acc_data[2]=raw_acc_xyz[2];
	
	raw_gyro_data[0]=raw_gyro_xyz[1];
	raw_gyro_data[1]=raw_gyro_xyz[0];
	raw_gyro_data[2]=-raw_gyro_xyz[2];
}

/**
 * @brief  ��ȡ���ٶȺͽ��ٶȺ���
 * @param  MPU6050Data�����ڴ洢���ݵĽṹ��
 * @return None
 */
void MPU6050_GetGyroAccData(MPU6050Data_TypeDef *MPU6050Data)
{
	float raw_acc[3],raw_gyro[3];
	MPU6050_GetRawData(raw_acc,raw_gyro);                                           //���ԭʼ����
	MPU6050_MinusOffset(raw_acc,MPU6050Data->acc_offset);
	MPU6050_MinusOffset(raw_gyro,MPU6050Data->gyro_offset);
	
	MPU6050_GyroDegToRadian(raw_gyro,MPU6050Data->gyro);                            //���ٶȵ�λת�������������λ����
	MPU6050_AccGToMS2(raw_acc,MPU6050Data->acc);                                          //���ٶȵ�λת��
	
	
}


/**
 * @brief  MPU6050У׼
 * @param  MPU6050Data�����ڴ洢���ݵĽṹ��
 * @return None
 */
void MPU6050_Calibration(float *gyro_offset,float *acc_offset)
{
	float raw_gyro[3],raw_acc[3];
	u16 CalibraTime = 100;    //У׼����
	float gyro_CaliData[3]={0},acc_CaliData[3]={0};
	for(u8 j=0;j<CalibraTime;j++)
	{
  MPU6050_GetRawData(raw_acc,raw_gyro);
	delay_ms(1);
	/* Get Angular rate */
  for (int i = 0; i < 3; i++)
	{
   gyro_CaliData[i] += raw_gyro[i];
	 acc_CaliData[i] += raw_acc[i];
	}
	}
	for(int i=0;i<3;i++)
	{
		gyro_offset[i] = gyro_CaliData[i]/CalibraTime;
		acc_offset[i] = acc_CaliData[i]/CalibraTime;
	}
	acc_offset[2]-=1.0f;
}

static void MPU6050_GyroDegToRadian(float DegData[3],float RadianData[3])
{
	for(u8 i=0;i<3;i++){RadianData[i]=DegData[i]* DegToRadian ;}
}

static void MPU6050_AccGToMS2(float G_Data[3],float MS2Data[3])
{
	for(u8 i=0;i<3;i++){MS2Data[i]=G_Data[i]*GRAVITY_MSS;}
}

static void MPU6050_MinusOffset(float *raw_data,float *data_offset)
{
	for(u8 i=0;i<3;i++){raw_data[i]-=data_offset[i];}
}
