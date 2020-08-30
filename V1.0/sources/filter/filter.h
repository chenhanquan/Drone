#ifndef FILTER_H
#define FILTER_H

#include "stm32f4xx.h"
#include <math.h>

typedef struct{
	float acc[3];    //���ٶ����� 0:x,1:y,2:z  ����
	float gyro[3];   //���ٶ����� 0:x,1:y,2:z  ����
	float quad[4];   //��Ԫ��
	float roll;      //������  ���
	float pitch;     //������  ���
	float yaw;       //ƫ����  ���
	
}FilterData_Type;

void vQuadToEuler(FilterData_Type *xFilterData);     //��Ԫ��ת����ŷ����
void vComFilter(FilterData_Type *xFilterData);       //�����˲�
void vQuadInit(FilterData_Type *xFilterData);        //��Ԫ�����ݳ�ʼ��

#endif
