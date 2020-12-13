#ifndef __AK8975_H__
#define __AK8975_H__

#include "stm32f4xx.h"

/* Memory Map ----------------------------------------------------------------*/
#define AK8975_WIA      0x00
#define AK8975_INFO     0x01
#define AK8975_ST1      0x02
#define AK8975_HXL      0x03
#define AK8975_HXH      0x04
#define AK8975_HYL      0x05
#define AK8975_HYH      0x06
#define AK8975_HZL      0x07
#define AK8975_HZH      0x08
#define AK8975_ST2      0x09
#define AK8975_CNTL     0x0A
#define AK8975_RSV      0x0B    //????
#define AK8975_ASTC     0x0C
#define AK8975_TS1      0x0D    //????
#define AK8975_TS2      0x0E    //????
#define AK8975_I2CDIS   0x0F
#define AK8975_ASAX     0x10    //?????ROM,?????
#define AK8975_ASAY     0x11    //?????ROM,?????
#define AK8975_ASAZ     0x12    //?????ROM,?????

typedef struct{
	float mag[3];
	float mag_offset[3];
	float mag_an[3];
}AK8975Data_Type;

void AK8975_Init(void);
u8 AK8975_GetID();
void AK8975_GetMagData(AK8975Data_Type*MagData);

//void AK8975_Calibration(float *mag_data,float *mag_an,float *mag_offset);


#endif 