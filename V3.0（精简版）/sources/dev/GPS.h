#ifndef __GPS_H
#define __GPS_H

#include "usart.h"	
#include "DMA.h"
#include "delay.h"



typedef struct{
     u16 year ;
     u8 month;
	   u8 day;
	   u8 hour;
	   u8 min;
	   u8 sec;
	   u8 fixType;
	   u8 numSV;
	   double Longitude;
	   double Latitude;
	   float height;
	   float hAcc;
	   float vAcc;
	   float velN;
	   float velE;
	   float velD;
	   float gSpeed;
	   float sAcc;
	   float pDOP;
	   u32 time;
 }gps_data;

 
extern gps_data GPS;

 
 
void READ_GPS_DATA(void);
void Init_GPS(void);

#endif
