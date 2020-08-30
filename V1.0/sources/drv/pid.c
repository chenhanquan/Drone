#include "pid.h"


void vPIDCalculate(PID_Type*PID,float target,float measure)
{
	float Pout,Iout,Dout;
	PID->Ki_Flag = 1;
	PID->Error  = target - measure;
	PID->Differ = PID->Error - PID->PreError;
	
  #ifdef INTEGRAL_APART
	if(PID->Error > (PID->IntRange)||PID->Error < -PID->IntRange){PID->Ki_Flag = 0;}
	#endif
	
	PID->Integral += PID->Error;
	if(PID->Integral > PID->IntIimit)PID->Integral = PID->IntIimit;
	if(PID->Integral < -PID->IntIimit) PID->Integral = -PID->IntIimit; 
	
	Pout = PID->Kp * PID->Error;
	Iout = PID->Ki * PID->Integral * PID->Ki_Flag;
	Dout = PID->Kd * PID->Differ;
	
	PID->Output = Pout + Iout + Dout;
	
	PID->PreError = PID->Error ;                     
}
