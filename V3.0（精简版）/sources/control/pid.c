#include "pid.h"
#include "math.h"

#define PID_NORMAL_TIME  0.01f
#define PID_ALTERI_TIME      0.01f
#define PID_ALTERI_PRED_TIME 0.01f

void PID_Normal(PID_Type*PID,const float target,const float measure)
{
	float Pout,Iout,Error_abs;
	PID->Error  = target - measure;
	Error_abs=fabs(PID->Error);
	if(PID->Max_Error!=0)
	{
		if(Error_abs>PID->Max_Error&&PID->Error>0) {PID->Error=PID->Max_Error;}
		else if(Error_abs>PID->Max_Error&&PID->Error<0){PID->Error=-PID->Max_Error;}
	}
	
	//Pout
	Pout=PID->Kp*PID->Error;
	
  //Iout
	PID->Integral += PID->Error;
	if(PID->Integral > PID->IntIimit){PID->Integral = PID->IntIimit;}
	if(PID->Integral < -PID->IntIimit){PID->Integral = -PID->IntIimit; }
	Iout = PID->Ki * PID->Integral * PID_NORMAL_TIME;
	
	//Dout
//	PID->Dout=PID->Kd*PID->Kd_freq*(PID->Error - PID->last_Error)+(1.0f-PID->Kd_freq * PID_NORMAL_TIME)*PID->Dout;
//	PID->last_Error=PID->Error;
	
	PID->Dout=PID->Kd*(PID->Error-PID->last_Error);
	PID->last_Error=PID->Error;
	
	PID->Output = Pout + Iout + PID->Dout;
	                    
}

void PID_AlterI(PID_Type*PID,const float target,const float measure,const float limit_A,const float limit_B)
{
	float Pout,Iout,Ki_num,Error_abs,temp;
	PID->Error  = target - measure;
	Error_abs=fabs(PID->Error);
	if(PID->Max_Error!=0)
	{
		if(Error_abs>PID->Max_Error&&PID->Error>0) {PID->Error=PID->Max_Error;}
		else if(Error_abs>PID->Max_Error&&PID->Error<0){PID->Error=-PID->Max_Error;}
	}
	
	//Pout
	Pout=PID->Kp*PID->Error;
	
  //Iout
	PID->Integral += PID->Error;
	if(Error_abs>limit_A+limit_B){Ki_num=0;PID->Integral=0;}
	else if(Error_abs>limit_B){Ki_num=(limit_A+limit_B-Error_abs)/limit_A;}
	else{Ki_num=1.0f;}
	if(PID->Integral > PID->IntIimit){PID->Integral = PID->IntIimit;}
	if(PID->Integral < -PID->IntIimit){PID->Integral = -PID->IntIimit;}
	Iout = PID->Ki * PID->Integral * Ki_num * PID_ALTERI_TIME;
	
	//Dout
//	PID->Dout=PID->Kd*PID->Kd_freq*(PID->Error - PID->last_Error)+(1.0f-PID->Kd_freq * PID_ALTERI_TIME)*PID->Dout;
//	PID->last_Error=PID->Error;
	
	PID->Dout=PID->Kd*(PID->Error-PID->last_Error);
	PID->last_Error=PID->Error;
	
	PID->Output = Pout + Iout + PID->Dout;
	                    
}


//void PID_AlterI_PreD(PID_Type*PID,float target,float measure,float last_measure,float limit_A,float limit_B,float gama)
//{
//	float Pout,Iout,Ki_num,Error_abs,c1,c2,c3,temp,temp1;
//	PID->Error  = target - measure;
//	Error_abs=fabs(PID->Error);
//	if(PID->Max_Error!=0)
//	{
//		if(Error_abs>PID->Max_Error&&PID->Error>0) {PID->Error=PID->Max_Error;}
//		else if(Error_abs>PID->Max_Error&&PID->Error<0){PID->Error=-PID->Max_Error;}
//	}
//	
//	//Pout
//	Pout=PID->Kp*PID->Error;
//	
//  //Iout
//	PID->Integral += PID->Error;
//	if(Error_abs>limit_A+limit_B){Ki_num=0;PID->Integral=0;}
//	else if(Error_abs>limit_B){Ki_num=(limit_A+limit_B-Error_abs)/limit_A;}
//	else{Ki_num=1.0f;}
//	Iout = PID->Ki * PID->Integral * Ki_num * PID_ALTERI_PRED_TIME;
//	
//	//Dout
//	temp=gama*PID->Kd;
//	temp1=temp+PID_ALTERI_PRED_TIME;
//	c1=temp/temp1;
//	c2=(PID->Kd+ PID_ALTERI_PRED_TIME )/temp1;
//	c3=PID->Kd / temp1;
//	PID->Dout=c1*PID->Dout+c2*measure-c3*last_measure;
//	
//	PID->Output = Pout + Iout + PID->Dout;
//	                    
//}