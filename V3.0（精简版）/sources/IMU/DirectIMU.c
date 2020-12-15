#include "DirectIMU.h"
#include "math.h"
#include "arm_math.h"

#define dt 0.01f     //系统运行时间
#define ANGLE_NUM 0.001    //加速度计置信参数
#define YAW_NUM 0.002     //磁力计置信参数
#define Deg2Rad  0.01745329f
#define Rad2Deg  57.295779513f

static void IMU_UpdataY(float *Y,const float *acc);    //更新观测值
static void IMU_UpdataF(arm_matrix_instance_f32 *Mat,float *Mat_Buf,const float *gyro);      //更新过程矩阵
static void IMU_UpdataH(arm_matrix_instance_f32 *Mat,float *Mat_Buf,const float *quad);      //更新观测矩阵
static void MatMultiVector(arm_matrix_instance_f32 *Mat,const float *Vec,float *Vec_out);    //矩阵乘向量函数
static void YawCalcul(float *angle,const float *mag,const float K,const float wz,const u16 Thro);     //单独修正Yaw

void IMU_Init(float *quad)
{
	quad[0]=1.0f;
	quad[1]=0.0f;
	quad[2]=0.0f;
	quad[3]=0.0f;
}

void DirectIMU(float *quad,const float *acc,const float *gyro,const float *mag,float *angle,const u16 Thro)
{
	float norm;
	float quad_pre[4]={0},quad_temp1[4]={0},quad_temp2[4]={0};
	float Y[3]={0};
	float Mat_Buf_F[16],Mat_Buf_H[12],Mat_Buf_HT[12];
	arm_matrix_instance_f32 Mat_F;
	arm_matrix_instance_f32 Mat_H;
	arm_matrix_instance_f32 Mat_HT;
	
	//四元数乘法辅助
	float q0q0;							
	float q0q1;
	float q0q2;
	float q0q3;
	float q1q1;
	float q1q2;
	float q1q3;
	float q2q2;
	float q2q3;
	float q3q3;
	
	arm_mat_init_f32(&Mat_HT,4,3,Mat_Buf_HT);
	
	IMU_UpdataF(&Mat_F,Mat_Buf_F,gyro);
	IMU_UpdataH(&Mat_H,Mat_Buf_H,quad);
	IMU_UpdataY(Y,acc);
	arm_mat_trans_f32(&Mat_H,&Mat_HT);
	
	MatMultiVector(&Mat_F,quad,quad_pre);    //过程预测
	
	MatMultiVector(&Mat_H,quad_pre,quad_temp1);    //测量值
	arm_sub_f32(Y,quad_temp1,quad_temp2,3);        //求误差
	arm_scale_f32(quad_temp2,ANGLE_NUM,quad_temp1,3);    //数据融合
	MatMultiVector(&Mat_HT,quad_temp1,quad_temp2);
	arm_add_f32(quad_pre,quad_temp2,quad,4);
	
	//归一化
  arm_sqrt_f32(quad[0]*quad[0] + quad[1]*quad[1] + quad[2]*quad[2] + quad[3]*quad[3],&norm);  
	quad[0] = quad[0] / norm;  
  quad[1] = quad[1] / norm;  
  quad[2] = quad[2] / norm;  
  quad[3] = quad[3] / norm;
	
	//四元数乘法辅助
	q0q0 = quad[0] * quad[0];							
	q0q1 = quad[0] * quad[1];
	q0q2 = quad[0] * quad[2];
	q0q3 = quad[0] * quad[3];
	q1q1 = quad[1] * quad[1];
	q1q2 = quad[1] * quad[2];
	q1q3 = quad[1] * quad[3];
	q2q2 = quad[2] * quad[2];
	q2q3 = quad[2] * quad[3];
	q3q3 = quad[3] * quad[3];
	
	//解算出角度
	angle[0]=atan2(2.0f*(q0q1+q2q3),q0q0-q1q1-q2q2+q3q3)*Rad2Deg;    //roll
  angle[1]=asin(2.0f*(q0q2-q1q3))*Rad2Deg;                         //pitch
	//angle[2]=atan2(2.0f*(q1q2+q0q3),q0q0+q1q1-q2q2-q3q3)*Rad2Deg;
	YawCalcul(angle,mag,YAW_NUM,gyro[2]*Rad2Deg,Thro);
}

static void IMU_UpdataF(arm_matrix_instance_f32 *Mat,float *Mat_Buf,const float *gyro)
{
	
	float half_dt=0.5f*dt;
	Mat_Buf[0]=1.0f;Mat_Buf[1]=-gyro[0]*half_dt;Mat_Buf[2]=-gyro[1]*half_dt;Mat_Buf[3]=-gyro[2]*half_dt;
	Mat_Buf[4]=gyro[0]*half_dt;Mat_Buf[5]=1.0f;Mat_Buf[6]=gyro[2]*half_dt;Mat_Buf[7]=-gyro[1]*half_dt;
	Mat_Buf[8]=gyro[1]*half_dt;Mat_Buf[9]=-gyro[2]*half_dt;Mat_Buf[10]=1.0f;Mat_Buf[11]=gyro[0]*half_dt;
	Mat_Buf[12]=gyro[2]*half_dt;Mat_Buf[13]=gyro[1]*half_dt;Mat_Buf[14]=-gyro[0]*half_dt;Mat_Buf[15]=1.0f;
	
	arm_mat_init_f32(Mat,4,4,Mat_Buf);
}

static void IMU_UpdataH(arm_matrix_instance_f32 *Mat,float *Mat_Buf,const float *quad)
{
	
	float quad2[4]={0};
	quad2[0]=2.0f*quad[0];
	quad2[1]=2.0f*quad[1];
	quad2[2]=2.0f*quad[2];
	quad2[3]=2.0f*quad[3];
	
	Mat_Buf[0]=-quad2[2];Mat_Buf[1]=quad2[3];Mat_Buf[2]=-quad2[0];Mat_Buf[3]=quad2[1];    //[0 0 1]T
	Mat_Buf[4]=quad2[1];Mat_Buf[5]=quad2[0];Mat_Buf[6]=quad2[3];Mat_Buf[7]=quad2[2];
	Mat_Buf[8]=quad2[0];Mat_Buf[9]=-quad2[1];Mat_Buf[10]=-quad2[2];Mat_Buf[11]=quad2[3];
	
//	Mat_Buf[0]=quad2[2];Mat_Buf[1]=-quad2[3];Mat_Buf[2]=quad2[0];Mat_Buf[3]=-quad2[1];    //加速度计方法反了，这里我也暂时想不明白，[0 0 -1]T
//	Mat_Buf[4]=-quad2[1];Mat_Buf[5]=-quad2[0];Mat_Buf[6]=-quad2[3];Mat_Buf[7]=-quad2[2];
//	Mat_Buf[8]=-quad2[0];Mat_Buf[9]=quad2[1];Mat_Buf[10]=quad2[2];Mat_Buf[11]=-quad2[3];
	
	arm_mat_init_f32(Mat,3,4,Mat_Buf);
}

static void IMU_UpdataY(float *Y,const float *acc)
{
	float norm=0,acc_norm[3];
	norm= sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
	acc_norm[0]=acc[0]/norm;
	acc_norm[1]=acc[1]/norm;
	acc_norm[2]=acc[2]/norm;
	
	Y[0]=acc_norm[0];
	Y[1]=acc_norm[1];
	Y[2]=acc_norm[2];
}

static void MatMultiVector(arm_matrix_instance_f32 *Mat,const float *Vec,float *Vec_out)
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

static void YawCalcul(float *angle,const float *mag,const float K,const float wz,const u16 Thro)
{
	float Bx,By;
	float angle_temp[3];
	
	float angle_Pre;

	for(u8 i=0;i<2;i++){angle_temp[i]=angle[i] * Deg2Rad;}
	
	Bx=mag[0]*cos(angle_temp[1]) + mag[1]*sin(angle_temp[1])*sin(angle_temp[0])+mag[2]*sin(angle_temp[1])*cos(angle_temp[0]);
	By=mag[1]*cos(angle_temp[0]) - mag[2]*sin(angle_temp[0]);
	
	if(By==0.0&&Bx>0){angle_temp[2]=0;}
	else if(By<0){angle_temp[2]=PI/2.0f+atan(Bx/By);}
	else if(By==0.0&&Bx<0){angle_temp[2]=PI;}
	else if(By>0.0){angle_temp[2]=-PI/2.0f+atan(Bx/By);}
	
	angle_temp[2]=angle_temp[2]*Rad2Deg;
	
	
//	if(Thro>100){angle_temp[2]=angle_temp[2]-(0.0002*Thro*Thro-0.3058*Thro+51.5024);}    //油门补偿
	
	
	if(angle_temp[2]>180)angle_temp[2]=angle_temp[2]-360;
	if(angle_temp[2]<-180)angle_temp[2]=angle_temp[2]+360;
	
	//预测
	angle_Pre=angle[2]+ wz * dt;

	angle[2]=angle_Pre + K*(angle_temp[2]-angle_Pre);
	
	if(angle[2]>170||angle[2]<-170)angle[2]=angle_temp[2];

	
//	angle[2]=angle_temp[2];
	
//	angle[2]=angle[2]+ wz * dt;
//	if(angle[2]>180)angle[2]-=360;
//	if(angle[2]<-180)angle[2]+=360;
}

