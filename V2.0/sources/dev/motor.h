#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32f4xx.h"

void vMotorInit(void);
void vMotorPWMSet(u16 Motor1,u16 Motor2,u16 Motor3,u16 Motor4);
void vMotorLock(void);

#endif



