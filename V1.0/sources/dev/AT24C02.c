#include "AT24C02.h"
#include "delay.h"
#include "i2c.h"


#define AT24C02_ADDR 0xA0  //AT24c02  IIC��ַ


#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

#define START_ADDR 0  //256�ֽ� //��ʼ��ַ
#define  STOP_ADDR 0XFF //������ַ
 
 

uint8_t read_AT24C02(uint8_t addr,void *data,uint8_t len)
{
	uint8_t flag;
	uint8_t *temp = (uint8_t*)data;
	IIC_Read_One_Byte(AT24C02_ADDR,addr++,&flag); //��ȡ�Ƿ��Ѿ�У׼�����ݵı�־λ
	if(flag == 0xaa) //������������Ѿ�У׼���ˣ����ȡ���ݣ����򲻶�
	{
			while(len--)
			{		
					IIC_Read_One_Byte(AT24C02_ADDR,addr++,(temp++));
			}
	}
	return flag; //�����ѵ�ǰ�������Ƿ�������д��ı�־λ��ʹ����˼���Ƿ��ٴ�д��
}

void write_AT24C02(uint8_t addr,void *data,uint8_t len)
{
	uint8_t *temp = (uint8_t*)data;
	IIC_Write_One_Byte(AT24C02_ADDR,addr++,0xaa); //ÿ�����ݶ���һ���ֽ�����ʾ�Ѿ�У׼����
	
	while(len--)
	{		
			delay_ms(100);
			IIC_Write_One_Byte(AT24C02_ADDR,addr++,*(temp++));
	}
}




uint8_t test_AT24C02(void)//����
{
	uint8_t test = 0;
  IIC_Write_One_Byte(AT24C02_ADDR,0x55,0xAA);
	delay_ms(1000);
	IIC_Read_One_Byte(AT24C02_ADDR,0x55,&test);
	if(test != 0xAA) //�洢���쳣
		return FAILED;
	else
		return SUCCESS;
}
	













