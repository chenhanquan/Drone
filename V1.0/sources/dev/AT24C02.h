#ifndef _AT24C02_H
#define _AT24C02_H

#define GRYO_OFFSET 0//水平零偏校准
#define AK8975_OFFSET 13
#define MPU6050_SIX_B_OFFSET 20 //加速度六面校准
#define MPU6050_SIX_K_OFFSET 33 //加速度六面校准


unsigned char test_AT24C02(void);
unsigned char read_AT24C02(unsigned char addr,void *data,unsigned char len);
void write_AT24C02(unsigned char addr,void *data,unsigned char len);
#endif
