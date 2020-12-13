#include "LeastSquare.h"
#include "arm_math.h"



//x_measure = a * x_real + b 
//LS_Para_List=[x_measure,x_real,para_a,para_b,PN1,PN2,PN3,PN4,Counter]
void LS_Simple(float *LS_Para_List)
{
	if(LS_Para_List[8]==0)
	{
		float PM_Buf[4],Mat_Temp_Buf[4];
		arm_matrix_instance_f32 PM;
		arm_matrix_instance_f32 Mat_Temp;
		arm_mat_init_f32(&PM,2,2,PM_Buf);
		arm_mat_init_f32(&Mat_Temp,2,2,Mat_Temp_Buf);
		
		PM_Buf[0]=LS_Para_List[1]*LS_Para_List[1];
		PM_Buf[1]=LS_Para_List[1];
		PM_Buf[2]=LS_Para_List[1];
		PM_Buf[0]=1.0f;
		
		arm_mat_inverse_f32(&PM,&Mat_Temp);
		for(uint8_t i=0;i<4;i++){LS_Para_List[4+i]=Mat_Temp_Buf[i];}
		LS_Para_List[2]=(Mat_Temp_Buf[0]*LS_Para_List[1]+Mat_Temp_Buf[1])*LS_Para_List[0];
		LS_Para_List[3]=(Mat_Temp_Buf[2]*LS_Para_List[1]+Mat_Temp_Buf[3])*LS_Para_List[0];	
	}
	else
	{
		float GN[2]={0};
		float Vec_Temp1[2],Num_Temp1,Mat_Temp1[4],Mat_Temp2[4];
		
		Vec_Temp1[0]=LS_Para_List[4]*LS_Para_List[1]+LS_Para_List[6];
		Vec_Temp1[1]=LS_Para_List[5]*LS_Para_List[1]+LS_Para_List[7];
		
		Num_Temp1=1.0f/(1.0f+Vec_Temp1[0]*LS_Para_List[1]+Vec_Temp1[1]);
		
		GN[0]=(LS_Para_List[4]*LS_Para_List[1]+LS_Para_List[5])*Num_Temp1;
		GN[1]=(LS_Para_List[6]*LS_Para_List[1]+LS_Para_List[7])*Num_Temp1;
		
		Num_Temp1=LS_Para_List[0]-(LS_Para_List[1]*LS_Para_List[2]+LS_Para_List[3]);
		LS_Para_List[2]=LS_Para_List[2]+GN[0]*Num_Temp1;
		LS_Para_List[3]=LS_Para_List[3]+GN[1]*Num_Temp1;
		
		Mat_Temp1[0]=1.0f-1.0f*GN[0]*LS_Para_List[1];
		Mat_Temp1[1]=-1.0f*GN[0];
		Mat_Temp1[2]=-1.0f*GN[1]*LS_Para_List[1];
		Mat_Temp1[3]=1.0f-1.0f*GN[1];
		
		Mat_Temp2[0]=Mat_Temp1[0]*LS_Para_List[4]+Mat_Temp1[1]*LS_Para_List[6];
		Mat_Temp2[1]=Mat_Temp1[0]*LS_Para_List[5]+Mat_Temp1[1]*LS_Para_List[7];
		Mat_Temp2[2]=Mat_Temp1[2]*LS_Para_List[4]+Mat_Temp1[3]*LS_Para_List[6];
		Mat_Temp2[3]=Mat_Temp1[2]*LS_Para_List[5]+Mat_Temp1[3]*LS_Para_List[7];
		for(uint8_t i=0;i<4;i++){LS_Para_List[4+i]=Mat_Temp2[i];}
	}
	LS_Para_List[8]++;
}