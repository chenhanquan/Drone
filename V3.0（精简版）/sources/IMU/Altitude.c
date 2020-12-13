#include "Altitude.h"
#include "math.h"

#define Deg2Rad  0.01745329f
#define dt 0.01

void Altitude_GetRelativeAltitude(float *H,float *angle,float *acc)
{
	static float Accz=0;
	static float H_Accz,H_Pressure;
	
	Accz=acc[2]*cos(angle[0]*Deg2Rad)*cos(angle[1]*Deg2Rad)-9.8f;
	
	H_Accz=H[2]-0.5*Accz*dt*dt;
	
	H_Pressure=H[0]-H[1];
	
	H[2]=H_Accz+0.5*(H_Pressure-H_Accz);
	
	H[2]=H_Pressure;
}
