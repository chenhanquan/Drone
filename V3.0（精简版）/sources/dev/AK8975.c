#include "AK8975.h"
#include "i2c.h"
#include "delay.h"
#include "arm_math.h"

#define	MAG_ADDR 0x18   //IIC写入时的地址字节数据，+1为读取
#define MAG_FILTER_NUM 10

static u8 ASA[3]={0};

static void AK8975_ReadByte(uint8_t slaveAddr, uint8_t regAddr,  uint8_t *data);                //读一位寄存器
static void AK8975_ReadNByte(uint8_t slaveAddr, uint8_t regAddr,uint8_t len,  uint8_t *data);   //读多为寄存器
static void AK8975_ByteWrite(u8 slaveAddr,u8 writeAddr ,u8 data);                           //写一位寄存器
static float Calculate2Hadj(float Data,u8 axis);
static void AK8975_GetMagRawData(AK8975Data_Type*MagData,float *raw_mag_output);

static void MatMultiVector(const arm_matrix_instance_f32 *Mat,const float *Vec,float *Vec_out);

void AK8975_Init()
{
	
  uint8_t temp[3];
  AK8975_ByteWrite(MAG_ADDR,AK8975_CNTL,0X0F);
  AK8975_ReadNByte(MAG_ADDR,AK8975_ASAX,3,temp);
  ASA[0]=temp[0];
  ASA[1]=temp[1];
  ASA[2]=temp[2];
  AK8975_ByteWrite(MAG_ADDR,AK8975_CNTL,0X01);
}


u8 AK8975_GetID()
{
	u8 temp=0;
	AK8975_ReadByte(MAG_ADDR, AK8975_WIA, &temp);                //读一位寄存器
	return temp;
}

static float Calculate2Hadj(float Data,u8 axis)
{
	float Hadj=0;
	Hadj=Data*(((float)(ASA[axis]-128))/256+1);
	return Hadj;
}

static void AK8975_GetMagRawData(AK8975Data_Type*MagData,float *raw_mag_output)
{
	u8 temp[6]={0};
	float raw_mag_temp[3]={0};
	AK8975_ReadNByte(MAG_ADDR, AK8975_HXL,6,  temp);   //读多为寄存器
	for(u8 i=0;i<3;i++)
	{
		u16 temp1=0;
		temp1=((temp[2*i+1]<<8)+temp[2*i]);         //获取原始数据
		raw_mag_temp[i]=(s16)temp1/4096.0f*1229.0f; //转换数据
		raw_mag_temp[i]=Calculate2Hadj(raw_mag_temp[i],i);    //数据修正
		raw_mag_temp[i]=MagData->mag_an[i]*(raw_mag_temp[i] - MagData->mag_offset[i]);    //数据校准和单位转换
	}
	
//	AK8975_Calibration(raw_mag_temp,MagData->mag_an,MagData->mag_offset);
//	
//	raw_mag_temp[0]=MagData->mag_an[0]*(raw_mag_temp[0] - MagData->mag_offset[0]);
//	raw_mag_temp[1]=MagData->mag_an[1]*(raw_mag_temp[1] - MagData->mag_offset[1]);
//	raw_mag_temp[2]=MagData->mag_an[2]*(raw_mag_temp[2] - MagData->mag_offset[2]);
	
	raw_mag_output[0]=-raw_mag_temp[1];    //芯片坐标转到机体坐标
	raw_mag_output[1]=raw_mag_temp[0];
	raw_mag_output[2]=raw_mag_temp[2];
	
//	raw_mag_output[0]=raw_mag_temp[0];    //芯片坐标转到机体坐标
//	raw_mag_output[1]=raw_mag_temp[1];
//	raw_mag_output[2]=raw_mag_temp[2];
	
}

void AK8975_GetMagData(AK8975Data_Type*MagData)
{
	AK8975_GetMagRawData(MagData,MagData->mag);
}

//void AK8975_Calibration(float *mag_data,float *mag_an,float *mag_offset)
//{
//	static u8 Sample_Count=0;
//	float D[9]={0};
//	static float theta[9]={0};
//	static float PM_Buf[81]={0};
//	arm_matrix_instance_f32 PM;
//	arm_mat_init_f32(&PM,9,9,PM_Buf);
//	
//	if(Sample_Count<=5)
//	{
//		static float Mat_D_Buf[45]={0};
//		Mat_D_Buf[Sample_Count*9]=mag_data[0]*mag_data[0]; Mat_D_Buf[Sample_Count*9+1]=mag_data[1]*mag_data[1]; Mat_D_Buf[Sample_Count*9+2]=mag_data[2]*mag_data[2];    //x^2  y^2  z^2
//		Mat_D_Buf[Sample_Count*9+3]=2.0f*mag_data[0]*mag_data[1]; Mat_D_Buf[Sample_Count*9+4]=2.0f*mag_data[0]*mag_data[2]; Mat_D_Buf[Sample_Count*9+5]=2.0f*mag_data[1]*mag_data[2];    //2xy 2xz 2yz
//		Mat_D_Buf[Sample_Count*9+6]=2.0f*mag_data[0]; Mat_D_Buf[Sample_Count*9+7]=2.0f*mag_data[1]; Mat_D_Buf[Sample_Count*9+8]=2.0f*mag_data[2];    //2x 2y 2z
////		for(u8 i=0;i<9;i++){Mat_D_Buf[Sample_Count*9+i]=Mat_D_Buf[Sample_Count*9+i]*0.00001f;}
//		if(Sample_Count==5)
//		{
//			float Mat_D_T_Buf[45]={0};
//			float Mat_Temp_Buf[81];
//			float Vec_one[5]={1.0f,1.0f,1.0f,1.0f,1.0f};
//			arm_matrix_instance_f32 Mat_Temp;
//			arm_matrix_instance_f32 Mat_D;
//			arm_matrix_instance_f32 Mat_D_T;
//			arm_mat_init_f32(&Mat_D,5,9,Mat_D_Buf);
//			arm_mat_init_f32(&Mat_D_T,9,5,Mat_D_T_Buf);
//			arm_mat_init_f32(&Mat_Temp,9,9,Mat_Temp_Buf);
//			
//			arm_mat_trans_f32(&Mat_D,&Mat_D_T);
//			
//			arm_mat_mult_f32(&Mat_D_T,&Mat_D,&Mat_Temp);
//			arm_mat_inverse_f32(&Mat_Temp,&PM);
//			
//			arm_mat_init_f32(&Mat_Temp,9,5,Mat_Temp_Buf);
//			arm_mat_mult_f32(&PM,&Mat_D_T,&Mat_Temp);
//			MatMultiVector(&Mat_Temp,Vec_one,theta);
//		}
//		Sample_Count++;
//	}
//	else
//	{
//		float GN[9];
//		float Mat_Temp1_Buf[81];
//		float Mat_Temp2_Buf[81];
//		float number_inv;
//		arm_matrix_instance_f32 Vec_GN;
//		arm_matrix_instance_f32 Vec_D_T;
//		arm_matrix_instance_f32 Mat_Temp1;
//		arm_matrix_instance_f32 Mat_Temp2;
//		
//		D[0]=mag_data[0]*mag_data[0];D[1]=mag_data[1]*mag_data[1];D[2]=mag_data[2]*mag_data[2];    //x^2  y^2  z^2
//		D[3]=2.0f*mag_data[0]*mag_data[1];D[4]=2.0f*mag_data[0]*mag_data[2];D[5]=2.0f*mag_data[1]*mag_data[2];    //2xy 2xz 2yz
//		D[6]=2.0f*mag_data[0];D[7]=2.0f*mag_data[1];D[8]=2.0f*mag_data[2];    //2x 2y 2z
////		for(u8 i=0;i<9;i++){D[i]=D[i]*0.00001f;}
//		
//		arm_mat_init_f32(&Vec_D_T,1,9,D);
//		arm_mat_init_f32(&Vec_GN,9,1,GN);
//		
//		arm_mat_init_f32(&Mat_Temp1,1,9,Mat_Temp1_Buf);
//		
//		arm_mat_mult_f32(&Vec_D_T,&PM,&Mat_Temp1);    //求GN
//		MatMultiVector(&Mat_Temp1,D,Mat_Temp2_Buf);
//		number_inv=1.0f/(1.0f+Mat_Temp2_Buf[0]);
//		MatMultiVector(&PM,D,Mat_Temp1_Buf);
//		arm_scale_f32(Mat_Temp1_Buf,number_inv,GN,9);
//		
//		MatMultiVector(&Vec_D_T,theta,Mat_Temp2_Buf);    //更新theta
//		arm_scale_f32(GN,(1.0f-Mat_Temp2_Buf[0]),Mat_Temp1_Buf,9);
//		arm_add_f32(theta,Mat_Temp1_Buf,Mat_Temp2_Buf,9);
//		for(u8 i=0;i<9;i++){theta[i]=Mat_Temp2_Buf[i];}
////		arm_scale_f32(Mat_Temp2_Buf,-1.0f,theta,9);
//		
//		arm_mat_init_f32(&Mat_Temp1,9,9,Mat_Temp1_Buf);    //更新PM
//		arm_mat_init_f32(&Mat_Temp2,9,9,Mat_Temp2_Buf);
//		arm_mat_mult_f32(&Vec_GN,&Vec_D_T,&Mat_Temp1);
//		arm_mat_scale_f32(&Mat_Temp1,-1.0f,&Mat_Temp2);
//		for(u8 i=0;i<9;i++){Mat_Temp2_Buf[i*9+i]+=1.0f;}
//		arm_mat_mult_f32(&Mat_Temp2,&PM,&Mat_Temp1);
//		for(u8 i=0;i<81;i++){PM_Buf[i]=Mat_Temp1_Buf[i];}
//		
//		if(theta[0]<0&&theta[1]<0&&theta[2]<0)
//		{
//		
//		mag_offset[0]=-theta[6]/theta[0];
//		mag_offset[1]=-theta[7]/theta[1];
//		mag_offset[2]=-theta[8]/theta[2];
//		
//		arm_sqrt_f32(-1.0f/(theta[0]),mag_an);
//		arm_sqrt_f32(-1.0f/(theta[1]),mag_an+1);
//		arm_sqrt_f32(-1.0f/(theta[2]),mag_an+2);
//		
//		}
//	}
//}

static void MatMultiVector(const arm_matrix_instance_f32 *Mat,const float *Vec,float *Vec_out)
{
	for(int i=0;i<Mat->numRows;i++)
	{
		Vec_out[i]=0;
		for(int j=0;j<Mat->numCols;j++)
		{
			Vec_out[i]+=Mat->pData[i*Mat->numCols+j]*Vec[j];
		}
	}
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
	u8 flag=0;
	if(regAddr==AK8975_HXL||regAddr==AK8975_HYL||regAddr==AK8975_HZL)
	{
		while(!flag){AK8975_ReadByte(MAG_ADDR, AK8975_ST1, &flag);}
	}
	IIC_Read_nByte(slaveAddr, regAddr, len, data);
	if(regAddr==AK8975_HXL||regAddr==AK8975_HYL||regAddr==AK8975_HZL)
	{
		AK8975_ByteWrite(MAG_ADDR,AK8975_CNTL,0x01);
	}
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

