#ifndef _AT24C02_H
#define _AT24C02_H

#define GRYO_OFFSET 0//ˮƽ��ƫУ׼
#define AK8975_OFFSET 13
#define MPU6050_SIX_B_OFFSET 20 //���ٶ�����У׼
#define MPU6050_SIX_K_OFFSET 33 //���ٶ�����У׼


unsigned char test_AT24C02(void);
unsigned char read_AT24C02(unsigned char addr,void *data,unsigned char len);
void write_AT24C02(unsigned char addr,void *data,unsigned char len);
#endif
