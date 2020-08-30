#include <filter.h>

/*****�����˲�����������*****/
#define QUAD_KP  1.5f        //��������
#define QUAD_KI  0.005f      //���ֳ���
#define QUAD_HALFT 0.001f  //������
/****************************/

/**
  * @name   vQuadToEuler
  * @brief  ����Ԫ��ת��Ϊŷ����  
  * @param  xFilterData
  * @retval None
  */
void vQuadToEuler(FilterData_Type *xFilterData)
{
	//��Ԫ���˷�����
	float q0q0 = xFilterData->quad[0] * xFilterData->quad[0];							
	float q0q1 = xFilterData->quad[0] * xFilterData->quad[1];
	float q0q2 = xFilterData->quad[0] * xFilterData->quad[2];
	float q1q1 = xFilterData->quad[1] * xFilterData->quad[1];
	float q1q3 = xFilterData->quad[1] * xFilterData->quad[3];
	float q2q2 = xFilterData->quad[2] * xFilterData->quad[2];
	float q2q3 = xFilterData->quad[2] * xFilterData->quad[3];
	float q3q3 = xFilterData->quad[3] * xFilterData->quad[3];
	//
	//ŷ����ת��
	xFilterData->pitch=atan2(2.0f*(q0q1+q2q3),q0q0-q1q1-q2q2+q3q3)*57.30f;
  xFilterData->roll=asin(2.0f*(q0q2-q1q3))*57.30f; 
  xFilterData->yaw=atan2(2*(q0q1+q2q3),q0q0+q1q1-q2q2-q3q3)*57.30f;
	//
}

/**
  * @name   vComFilter
  * @brief  �����˲���  
  * @param  xFilterData
  * @retval None
  */
void vComFilter(FilterData_Type *xFilterData) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;  
	static float exInt=0,eyInt=0,ezInt=0;    //����ֵ
  //	
	//��Ԫ���˷�����
	float q0q0 = xFilterData->quad[0] * xFilterData->quad[0];							
	float q0q1 = xFilterData->quad[0] * xFilterData->quad[1];
	float q0q2 = xFilterData->quad[0] * xFilterData->quad[2];
	float q1q1 = xFilterData->quad[1] * xFilterData->quad[1];
	float q1q3 = xFilterData->quad[1] * xFilterData->quad[3];
	float q2q2 = xFilterData->quad[2] * xFilterData->quad[2];
	float q2q3 = xFilterData->quad[2] * xFilterData->quad[3];
	float q3q3 = xFilterData->quad[3] * xFilterData->quad[3];
	//
	//��һ������
	norm = sqrt(xFilterData->acc[0]*xFilterData->acc[0] + xFilterData->acc[1]*xFilterData->acc[1] + xFilterData->acc[2]*xFilterData->acc[2]);     
	xFilterData->acc[0] = xFilterData->acc[0] / norm;
	xFilterData->acc[1] = xFilterData->acc[1] / norm;
	xFilterData->acc[2] = xFilterData->acc[2] / norm;   
  //	
	//����С��������ϵ	
	vx = 2*(q1q3 - q0q2);								
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	//
	//����ϵ�������������
	ex = (xFilterData->acc[1]*vz - xFilterData->acc[2]*vy);								
	ey = (xFilterData->acc[2]*vx - xFilterData->acc[0]*vz);
	ez = (xFilterData->acc[0]*vy - xFilterData->acc[1]*vx);
	//
	//��������
	exInt = exInt + ex*QUAD_KI;
	eyInt = eyInt + ey*QUAD_KI;
	ezInt = ezInt + ez*QUAD_KI;
	//
	//�������ں�
	xFilterData->gyro[0] = xFilterData->gyro[0] + QUAD_KP * ex + exInt;
	xFilterData->gyro[1] = xFilterData->gyro[1] + QUAD_KP * ey + eyInt;
	xFilterData->gyro[2] = xFilterData->gyro[2] + QUAD_KP * ez + ezInt;
	//
	//������Ԫ����
	xFilterData->quad[0] = xFilterData->quad[0] + (-xFilterData->quad[1] * xFilterData->gyro[0] - xFilterData->quad[2]*xFilterData->gyro[1] - xFilterData->quad[3] * xFilterData->gyro[2]) * QUAD_HALFT;
	xFilterData->quad[1] = xFilterData->quad[1] + (xFilterData->quad[0] * xFilterData->gyro[0] + xFilterData->quad[2] * xFilterData->gyro[2] - xFilterData->quad[3]*xFilterData->gyro[1]) * QUAD_HALFT;
	xFilterData->quad[2] = xFilterData->quad[2] + (xFilterData->quad[0] * xFilterData->gyro[1] - xFilterData->quad[1] * xFilterData->gyro[2] + xFilterData->quad[3]*xFilterData->gyro[0]) * QUAD_HALFT;
	xFilterData->quad[3] = xFilterData->quad[3] + (xFilterData->quad[0] * xFilterData->gyro[2] + xFilterData->quad[1] * xFilterData->gyro[1] - xFilterData->quad[2]*xFilterData->gyro[0]) * QUAD_HALFT;  
	//
	//��һ������
	norm = sqrt(xFilterData->quad[0]*xFilterData->quad[0] + xFilterData->quad[1]*xFilterData->quad[1] + xFilterData->quad[2]*xFilterData->quad[2] + xFilterData->quad[3]*xFilterData->quad[3]);	
	xFilterData->quad[0] = xFilterData->quad[0] / norm;
	xFilterData->quad[1] = xFilterData->quad[1] / norm;
	xFilterData->quad[2] = xFilterData->quad[2] / norm;
	xFilterData->quad[3] = xFilterData->quad[3] / norm;
	//
	vQuadToEuler(xFilterData);
}

/**
  * @brief  ��Ԫ����ʼ��
  * @param  xFilterData
  * @retval None
  */
void vQuadInit(FilterData_Type *xFilterData)
{
	xFilterData->quad[0]=1;
	for(u8 i=0;i<3;i++){xFilterData->quad[i+1]=0;}
}

