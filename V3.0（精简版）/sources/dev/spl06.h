#ifndef _SPL06_H
#define _SPL06_H

#include "stm32f4xx.h"


u8 SPL06_ReadID();
void SPL06_Init();
float SPL06_GetPressureData();
float SPL06_GetHeight();


#endif