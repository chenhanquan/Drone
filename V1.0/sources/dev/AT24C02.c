#include "AT24C02.h"
#include "delay.h"
#include "i2c.h"


#define AT24C02_ADDR 0xA0  //AT24c02  IIC地址


#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

#define START_ADDR 0  //256字节 //起始地址
#define  STOP_ADDR 0XFF //结束地址
 
 

uint8_t read_AT24C02(uint8_t addr,void *data,uint8_t len)
{
	uint8_t flag;
	uint8_t *temp = (uint8_t*)data;
	IIC_Read_One_Byte(AT24C02_ADDR,addr++,&flag); //读取是否已经校准过数据的标志位
	if(flag == 0xaa) //如果该组数据已经校准过了，则读取数据，否则不读
	{
			while(len--)
			{		
					IIC_Read_One_Byte(AT24C02_ADDR,addr++,(temp++));
			}
	}
	return flag; //返回已当前组数据是否曾经被写入的标志位供使用者思考是否再次写入
}

void write_AT24C02(uint8_t addr,void *data,uint8_t len)
{
	uint8_t *temp = (uint8_t*)data;
	IIC_Write_One_Byte(AT24C02_ADDR,addr++,0xaa); //每组数据都有一个字节来表示已经校准过了
	
	while(len--)
	{		
			delay_ms(100);
			IIC_Write_One_Byte(AT24C02_ADDR,addr++,*(temp++));
	}
}




uint8_t test_AT24C02(void)//测试
{
	uint8_t test = 0;
  IIC_Write_One_Byte(AT24C02_ADDR,0x55,0xAA);
	delay_ms(1000);
	IIC_Read_One_Byte(AT24C02_ADDR,0x55,&test);
	if(test != 0xAA) //存储器异常
		return FAILED;
	else
		return SUCCESS;
}
	













