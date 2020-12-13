#ifndef _AT24C02_H
#define _AT24C02_H

#include "stm32f4xx.h"

void AT24C02_ByteWrite(u8 writeAddr ,u8 data);
void AT24C02_PageWrite(u8 WritePage,u8* pBuffer);    //าณะด,0-31าณ
u8 AT24C02_ByteCurrentRead(void);
void AT24C02_NByteRandomRead(u8 readAddr ,u8 len,u8* pBuffer);
u8 AT24C02_NByteSequentialRead(u8 len,u8* pBuffer);
uint8_t AT24C02_Test(void);//ฒโสิ,255ฮป 


void SaveFloatData(const float DataToSave,uint8_t pos);
float ReadFloatData(uint8_t pos);


#endif
