#ifndef FILTER_H
#define FILTER_H

#include "stm32f4xx.h"
#include <math.h>

typedef struct{
	float acc[3];    //加速度数据 0:x,1:y,2:z  输入
	float gyro[3];   //角速度数据 0:x,1:y,2:z  输入
	float quad[4];   //四元数
	float roll;      //翻滚角  输出
	float pitch;     //俯仰角  输出
	float yaw;       //偏航角  输出
	
}FilterData_Type;

void vQuadToEuler(FilterData_Type *xFilterData);     //四元数转换成欧拉角
void vComFilter(FilterData_Type *xFilterData);       //互补滤波
void vQuadInit(FilterData_Type *xFilterData);        //四元数数据初始化

#endif
