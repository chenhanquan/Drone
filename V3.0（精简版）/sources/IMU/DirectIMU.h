#ifndef __DirectIMU_H__
#define __DirectIMU_H__

#include "stm32f4xx.h"

void IMU_Init(float *quad);
void DirectIMU(float *quad,const float *acc,const float *gyro,const float *mag,float *angle,const u16 Thro);
#endif 
