#include <filter.h>

/*****互补滤波器参数设置*****/
#define QUAD_KP  1.5f        //比例常数
#define QUAD_KI  0.005f      //积分常数
#define QUAD_HALFT 0.001f  //半周期
/****************************/

/**
  * @name   vQuadToEuler
  * @brief  将四元数转换为欧拉角  
  * @param  xFilterData
  * @retval None
  */
void vQuadToEuler(FilterData_Type *xFilterData)
{
	//四元数乘法运算
	float q0q0 = xFilterData->quad[0] * xFilterData->quad[0];							
	float q0q1 = xFilterData->quad[0] * xFilterData->quad[1];
	float q0q2 = xFilterData->quad[0] * xFilterData->quad[2];
	float q1q1 = xFilterData->quad[1] * xFilterData->quad[1];
	float q1q3 = xFilterData->quad[1] * xFilterData->quad[3];
	float q2q2 = xFilterData->quad[2] * xFilterData->quad[2];
	float q2q3 = xFilterData->quad[2] * xFilterData->quad[3];
	float q3q3 = xFilterData->quad[3] * xFilterData->quad[3];
	//
	//欧拉角转换
	xFilterData->pitch=atan2(2.0f*(q0q1+q2q3),q0q0-q1q1-q2q2+q3q3)*57.30f;
  xFilterData->roll=asin(2.0f*(q0q2-q1q3))*57.30f; 
  xFilterData->yaw=atan2(2*(q0q1+q2q3),q0q0+q1q1-q2q2-q3q3)*57.30f;
	//
}

/**
  * @name   vComFilter
  * @brief  互补滤波器  
  * @param  xFilterData
  * @retval None
  */
void vComFilter(FilterData_Type *xFilterData) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;  
	static float exInt=0,eyInt=0,ezInt=0;    //积分值
  //	
	//四元数乘法运算
	float q0q0 = xFilterData->quad[0] * xFilterData->quad[0];							
	float q0q1 = xFilterData->quad[0] * xFilterData->quad[1];
	float q0q2 = xFilterData->quad[0] * xFilterData->quad[2];
	float q1q1 = xFilterData->quad[1] * xFilterData->quad[1];
	float q1q3 = xFilterData->quad[1] * xFilterData->quad[3];
	float q2q2 = xFilterData->quad[2] * xFilterData->quad[2];
	float q2q3 = xFilterData->quad[2] * xFilterData->quad[3];
	float q3q3 = xFilterData->quad[3] * xFilterData->quad[3];
	//
	//归一化处理
	norm = sqrt(xFilterData->acc[0]*xFilterData->acc[0] + xFilterData->acc[1]*xFilterData->acc[1] + xFilterData->acc[2]*xFilterData->acc[2]);     
	xFilterData->acc[0] = xFilterData->acc[0] / norm;
	xFilterData->acc[1] = xFilterData->acc[1] / norm;
	xFilterData->acc[2] = xFilterData->acc[2] / norm;   
  //	
	//建立小四轴坐标系	
	vx = 2*(q1q3 - q0q2);								
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	//
	//坐标系和重力叉积运算
	ex = (xFilterData->acc[1]*vz - xFilterData->acc[2]*vy);								
	ey = (xFilterData->acc[2]*vx - xFilterData->acc[0]*vz);
	ez = (xFilterData->acc[0]*vy - xFilterData->acc[1]*vx);
	//
	//比例运算
	exInt = exInt + ex*QUAD_KI;
	eyInt = eyInt + ey*QUAD_KI;
	ezInt = ezInt + ez*QUAD_KI;
	//
	//陀螺仪融合
	xFilterData->gyro[0] = xFilterData->gyro[0] + QUAD_KP * ex + exInt;
	xFilterData->gyro[1] = xFilterData->gyro[1] + QUAD_KP * ey + eyInt;
	xFilterData->gyro[2] = xFilterData->gyro[2] + QUAD_KP * ez + ezInt;
	//
	//整合四元数率
	xFilterData->quad[0] = xFilterData->quad[0] + (-xFilterData->quad[1] * xFilterData->gyro[0] - xFilterData->quad[2]*xFilterData->gyro[1] - xFilterData->quad[3] * xFilterData->gyro[2]) * QUAD_HALFT;
	xFilterData->quad[1] = xFilterData->quad[1] + (xFilterData->quad[0] * xFilterData->gyro[0] + xFilterData->quad[2] * xFilterData->gyro[2] - xFilterData->quad[3]*xFilterData->gyro[1]) * QUAD_HALFT;
	xFilterData->quad[2] = xFilterData->quad[2] + (xFilterData->quad[0] * xFilterData->gyro[1] - xFilterData->quad[1] * xFilterData->gyro[2] + xFilterData->quad[3]*xFilterData->gyro[0]) * QUAD_HALFT;
	xFilterData->quad[3] = xFilterData->quad[3] + (xFilterData->quad[0] * xFilterData->gyro[2] + xFilterData->quad[1] * xFilterData->gyro[1] - xFilterData->quad[2]*xFilterData->gyro[0]) * QUAD_HALFT;  
	//
	//归一化处理
	norm = sqrt(xFilterData->quad[0]*xFilterData->quad[0] + xFilterData->quad[1]*xFilterData->quad[1] + xFilterData->quad[2]*xFilterData->quad[2] + xFilterData->quad[3]*xFilterData->quad[3]);	
	xFilterData->quad[0] = xFilterData->quad[0] / norm;
	xFilterData->quad[1] = xFilterData->quad[1] / norm;
	xFilterData->quad[2] = xFilterData->quad[2] / norm;
	xFilterData->quad[3] = xFilterData->quad[3] / norm;
	//
	vQuadToEuler(xFilterData);
}

/**
  * @brief  四元数初始化
  * @param  xFilterData
  * @retval None
  */
void vQuadInit(FilterData_Type *xFilterData)
{
	xFilterData->quad[0]=1;
	for(u8 i=0;i<3;i++){xFilterData->quad[i+1]=0;}
}

