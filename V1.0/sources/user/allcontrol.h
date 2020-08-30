#ifndef __ALLCONTROL_H__
#define __ALLCONTROL_H__

#include "mpu6050.h"
#include "led.h"
#include "pid.h"
#include "filter.h"
#include "usart.h"
#include "ppm.h"
#include "remote.h"
#include "motor.h"
#include "ANO_DT.h"

typedef struct
{
	enum{Lock=0,unLock=1}State;    //���˻�״̬
}Drone_Type;

void vAllInit(void);             //���г�ʼ��
void vDroneLock(void);           //���˻�����ģʽ
void vDroneunLock(void);         //���˻�����ģʽ


#endif 


