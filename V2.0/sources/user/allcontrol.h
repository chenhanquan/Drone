#ifndef __ALLCONTROL_H__
#define __ALLCONTROL_H__

#include "mpu6050.h"
#include "led.h"
#include "pid.h"
#include "usart.h"
#include "ppm.h"
#include "remote.h"
#include "motor.h"
#include "AK8975.h"
#include "AT24C02.h"
#include "delay.h"
#include "ANO_DT.h"
#include "Kalman.h"

typedef struct
{
	enum{Lock=0,unLock=1}State;    //���˻�״̬
	enum{
		Normal,
		Calibration
	}Mode;
}Drone_Type;

void vAllInit(void);             //���г�ʼ��
void vDroneLock(void);           //���˻�����ģʽ
void vDroneunLock(void);         //���˻�����ģʽ


#endif 


