#ifndef _KALMAN_H_
#define _KALMAN_H_

#include "stm32f4xx.h"

typedef struct
	{
		float roll;
		float pitch;
		float yaw;
		float quad_est[4];
		float raw_gyro[3];
		float raw_acc[3];
		float acc_norm[3];
		float raw_mag[3];
		float raw_mag_hor;
		float raw_mag_vert;
		float mag_norm[3];
	}KalmanData_Type;

void Kalman(KalmanData_Type *Kalmandata);
	

#endif

