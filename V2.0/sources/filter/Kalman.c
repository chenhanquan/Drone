#include "Kalman.h"
#include "HQ_math.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#define delta_t 0.001
#define PI 3.1415926

static float raw_quad[4]={0},quad_norm[4]={0};
static float angle[3]={0};
static float Global_B0x,Global_B0z;

static void DataNormalization(float*Data2Normalization,float*NormalizationData,int len);    //归一化
static void MeasureAngle(KalmanData_Type*Kalmandata);                                       //得到测量角度
static void Angle2Quad();                                                                   //角度转换为4元数
static void GetPoseA(KalmanData_Type*Kalmandata,float*Pose_A);                              //得到状态矩阵
static void GetPoseC(KalmanData_Type*Kalmandata,float *Pose_C);                             //得到观测矩阵
static void GetPoseZ(KalmanData_Type*Kalmandata,float*Z);                                   //得到测量值
static void vQuadToEuler(KalmanData_Type*Kalmandata);                                       //四元数转换为欧拉角

static void MeasureAngle(KalmanData_Type*Kalmandata)
{
	float totalAcc;
	float Hx,Hy;
	
	DataNormalization(Kalmandata->raw_acc,Kalmandata->acc_norm,3);
	DataNormalization(Kalmandata->raw_mag,Kalmandata->mag_norm,3);
	
	totalAcc=sqrt(Kalmandata->acc_norm[0]*Kalmandata->acc_norm[0]+Kalmandata->acc_norm[1]*Kalmandata->acc_norm[1]+Kalmandata->acc_norm[2]*Kalmandata->acc_norm[2]);
	angle[1]=asin(Kalmandata->acc_norm[0]/totalAcc);
	angle[0]=atan2(-Kalmandata->acc_norm[1],-Kalmandata->acc_norm[2]);
	Hx=Kalmandata->mag_norm[0]*cos(angle[1])+Kalmandata->mag_norm[1]*sin(angle[1])+Kalmandata->mag_norm[2]*cos(angle[0])*sin(angle[1]);
	Hy=Kalmandata->mag_norm[1]*cos(angle[0])-Kalmandata->mag_norm[2]*sin(angle[0]);
	if(fabs(Hy)<2)
	{
		if(Hx>0) angle[2]=0;    //情况1
		else angle[2]=PI;       //情况2
	}
	else
	{
		angle[2]=atan(Hx/Hy);
		if(Hy<0) angle[2]+=PI/2;    //情况3
		else angle[2] +=PI*3/2;     //情况4
	}
	angle[2]=angle[2]-0.11461;        //地磁偏角
	if(angle[2]<0.0) angle[2]+=2*PI;
}

static void Angle2Quad()
{
	float cosFai=cos(angle[0]/2);
	float sinFai=sin(angle[0]/2);
	float cosTheta=cos(angle[1]/2);
	float sinTheta=sin(angle[1]/2);
	float cosPsi=cos(angle[2]/2);
	float sinPsi=sin(angle[2]/2);
	
	raw_quad[0]=cosFai*cosTheta*cosPsi+sinFai*sinTheta*sinPsi;
	raw_quad[1]=sinFai*cosTheta*cosPsi-cosFai*sinTheta*sinPsi;
	raw_quad[2]=cosFai*sinTheta*cosPsi+sinFai*cosTheta*sinPsi;
	raw_quad[3]=cosFai*cosTheta*sinPsi-sinFai*sinTheta*cosPsi;
	
	DataNormalization(raw_quad,quad_norm,4);
}

static void GetPoseA(KalmanData_Type*Kalmandata,float*Pose_A)
{
	GetMatEye(4,Pose_A);
	Pose_A[1]=-Kalmandata->raw_gyro[0] * delta_t;
	Pose_A[2]=-Kalmandata->raw_gyro[1] * delta_t;
	Pose_A[3]=-Kalmandata->raw_gyro[2] * delta_t;
	Pose_A[4]=Kalmandata->raw_gyro[0] * delta_t;
	Pose_A[6]=Kalmandata->raw_gyro[2] * delta_t;
	Pose_A[7]=-Kalmandata->raw_gyro[1] * delta_t;
	Pose_A[8]=Kalmandata->raw_gyro[1] * delta_t;
	Pose_A[9]=-Kalmandata->raw_gyro[2] * delta_t;
	Pose_A[11]=Kalmandata->raw_gyro[0] * delta_t;
	Pose_A[12]=Kalmandata->raw_gyro[2] * delta_t;
	Pose_A[13]=Kalmandata->raw_gyro[1] * delta_t;
	Pose_A[14]=-Kalmandata->raw_gyro[0] * delta_t;
}

static void GetPoseC(KalmanData_Type*Kalmandata,float *Pose_C)
{
	float q2[4]={0};
	float mag_0[3]={0};
	float norm=0;

	q2[0]=2*quad_norm[0];
	q2[1]=2*quad_norm[1];
	q2[2]=2*quad_norm[2];
	q2[3]=2*quad_norm[3];
	
	norm=sqrt(Kalmandata->raw_mag_hor*Kalmandata->raw_mag_hor+Kalmandata->raw_mag_vert*Kalmandata->raw_mag_vert);
	Global_B0x=Kalmandata->raw_mag_hor/norm;
	Global_B0z=Kalmandata->raw_mag_vert/norm;
	
	mag_0[0]=Global_B0x;
	mag_0[1]=0;
	mag_0[2]=Global_B0z;
	
	Pose_C[0]=q2[2];Pose_C[1]=-q2[3];Pose_C[2]=q2[0];Pose_C[3]=-q2[1];
	Pose_C[4]=-q2[1];Pose_C[5]=-q2[0];Pose_C[6]=-q2[3];Pose_C[7]=-q2[2];
	Pose_C[8]=-q2[0];Pose_C[9]=q2[1];Pose_C[10]=q2[2];Pose_C[11]=-q2[3];
	
	Pose_C[12]=mag_0[0]*q2[0]-mag_0[2]*q2[2];
	Pose_C[13]=mag_0[0]*q2[1]+mag_0[2]*q2[3];
	Pose_C[14]=-mag_0[0]*q2[2]-mag_0[2]*q2[0];
	Pose_C[15]=-mag_0[0]*q2[3]+mag_0[2]*q2[1];
	
	Pose_C[16]=-mag_0[0]*q2[3]+mag_0[2]*q2[1];
	Pose_C[17]=mag_0[0]*q2[2]+mag_0[2]*q2[0];
	Pose_C[18]=mag_0[0]*q2[1]+mag_0[2]*q2[3];
	Pose_C[19]=-mag_0[0]*q2[0]+mag_0[2]*q2[2];
	
	Pose_C[20]=mag_0[0]*q2[2]+mag_0[2]*q2[0];
	Pose_C[21]=mag_0[0]*q2[3]+mag_0[2]*q2[1];
	Pose_C[22]=mag_0[0]*q2[0]-mag_0[2]*q2[2];
	Pose_C[23]=mag_0[0]*q2[1]+mag_0[2]*q2[3];
	
}

static void GetPoseZ(KalmanData_Type*Kalmandata,float*Z)
{
	for(u8 i=0;i<3;i++)
	{
		Z[i]=Kalmandata->acc_norm[i];
		Z[i+3]=Kalmandata->mag_norm[i];
	}
}

static void DataNormalization(float*Data2Normalization,float*NormalizationData,int len)
{
	float norm=0;
	for(u8 i=0;i<len;i++)
	{
		norm+=*(Data2Normalization+i*sizeof(float))*(*(Data2Normalization+i*sizeof(float)));
	}
	norm=sqrt(norm);
	for(u8 i=0;i<len;i++)
	{
		*(NormalizationData+i*sizeof(float))=*(Data2Normalization+i*sizeof(float))/norm;
	}
}


void Kalman(KalmanData_Type*Kalmandata)
{
	float A[16],A_T[16];
	float C[24],C_T[24];
	float Z[6];
	float I[16]={1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1};
	static float X_k[4]={1,0,0,0},X_Pre[4];
	float H[24];
	float*Temp=NULL,*Temp1=NULL;
	static float Pose_Q[16]={0};    //过程噪声
	static float Pose_R[36]={0};    //观测噪声
	static float Pose_P0[16]={1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1};
	static float Pose_P1[16]={0};
	
	MeasureAngle(Kalmandata);
	Angle2Quad();
	
	GetPoseA(Kalmandata,A);
	GetPoseC(Kalmandata,C);
	GetPoseZ(Kalmandata,Z);
	
	//状态预测
	MatMultiVector(A,4,4,X_k,4,X_Pre);    //X_pre=A*X(k-1)
	
	//协方差预测
	Temp=(float*)malloc(sizeof(float)*36);
	Temp1=(float*)malloc(sizeof(float)*36);
	MatTranspose(A,4,4,A_T);                      //得到A转置
	MatMulti(A,4,4,Pose_P0,4,4,Temp);
	MatMulti(Temp,4,4,A_T,4,4,Temp1);
	MatPlus(Temp1,4,4,Pose_Q,4,4,Pose_P1);       //P1=A*P0*A_T + Q
//	free(Temp);
//	free(Temp1);
	
	//卡尔曼增益计算
//	Temp=(float*)malloc(sizeof(float)*36);
//	Temp1=(float*)malloc(sizeof(float)*36);
	MatTranspose(C,6,4,C_T);                      //得到C转置
	MatMulti(C,6,4,Pose_P1,4,4,Temp);
	MatMulti(Temp,6,4,C_T,4,6,Temp1);
	MatPlus(Temp1,6,6,Pose_R,6,6,Temp);           //C*P1*C_T + R
	MatInv(Temp,6,Temp1);
	MatMulti(Pose_P1,4,4,C_T,4,6,Temp);          //P1*C_T
	MatMulti(Temp,4,6,Temp1,6,6,H);               //H=P1*C_T*inv(C*P1*C_T+R)
//	free(Temp);
//	free(Temp1);
	
	
	//滤波增益方程
//	Temp=(float*)malloc(sizeof(float)*24);
//	Temp1=(float*)malloc(sizeof(float)*24);
	MatMultiVector(C,6,4,X_Pre,4,Temp);
	MatMinus(Z,6,1,Temp,6,1,Temp1);                //新息  Z-C*X_Pre
	MatMultiVector(H,4,6,Temp1,6,Temp);
	MatPlus(X_Pre,4,1,Temp,4,1,X_k);             //X(k)=X_Pre+H*(Z-C*X_Pre)
//	free(Temp);
//	free(Temp1);
	
	//状态更新方程
	MatMulti(H,4,6,C,6,4,Temp);
	MatMinus(I,4,4,Temp,4,4,Temp1);
	MatMulti(Temp1,4,4,Pose_P1,4,4,Pose_P0);       //P0=[I-H*C]*P1
	free(Temp);
	free(Temp1);
	
	for(u8 i=4;i<4;i++){Kalmandata->quad_est[i]=X_k[i];}
	vQuadToEuler(Kalmandata);
}

static void vQuadToEuler(KalmanData_Type*Kalmandata)
{
	//四元数乘法运算
	float q0q0 = Kalmandata->quad_est[0] * Kalmandata->quad_est[0];							
	float q0q1 = Kalmandata->quad_est[0] * Kalmandata->quad_est[1];
	float q0q2 = Kalmandata->quad_est[0] * Kalmandata->quad_est[2];
	float q1q1 = Kalmandata->quad_est[1] * Kalmandata->quad_est[1];
	float q1q3 = Kalmandata->quad_est[1] * Kalmandata->quad_est[3];
	float q2q2 = Kalmandata->quad_est[2] * Kalmandata->quad_est[2];
	float q2q3 = Kalmandata->quad_est[2] * Kalmandata->quad_est[3];
	float q3q3 = Kalmandata->quad_est[3] * Kalmandata->quad_est[3];
	//
	//欧拉角转换
	Kalmandata->pitch=atan2(2.0f*(q0q1+q2q3),q0q0-q1q1-q2q2+q3q3)*57.30f;
  Kalmandata->roll=asin(2.0f*(q0q2-q1q3))*57.30f; 
  Kalmandata->yaw=atan2(2*(q0q1+q2q3),q0q0+q1q1-q2q2-q3q3)*57.30f;
	//
}
