
#include "usart.h"	

 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F4̽���߿�����
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/6/10
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
void _ttywrch(int ch)
{
	ch=ch;
}
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	

//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1); //GPIOB6����ΪUSART1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); //GPIOB7����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��B6��B7

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
//		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
//  USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
	
}

/****************************************************************************************************
* ��  ��: void USART1_IRQHandler(void)
* ��  ��: USART1�жϺ���������ʹ��
* ��  ��: ��
* ����ֵ: ��
* ��  ע: �����ߵ���ʱ����USART1��������������֡�Ľ��� �����ж�������ж�����ܽ���Ա�����
****************************************************************************************************/
void USART1_IRQHandler(void)
{
	uint8_t clear = clear; //���������������Ա�����֡�û���õ�����������ľ�����ʾ
	uint8_t res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //�����ж�
	{ 
		res = USART1->DR;
//		ANO_DT_Data_Receive_Prepare(res); //��λ�����ݽ��������
	}else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) //�����ж�
	{
		clear = USART1->SR; //��SR�Ĵ���
		clear = USART1->DR; //��DR�Ĵ������ȶ�SR,�ٶ�DR,����Ϊ�����IDIE�жϣ�
		
	}
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		}	
#endif

void Usart_Send(uint8_t *data, uint8_t length)
{
	uint8_t  i;
		for(i=0;i<length;i++)
		{
			 USART_SendData(USART1, *(data+i));
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
			{}
		}

}




//��ʼ��IO ����2 
//bound:������
void uart2_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	USART_DeInit(USART2);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIODʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��

	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); //GPIOD5����ΪUSART2
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); //GPIOD6����ΪUSART2
	
	//USART2�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; //GPIOD5��GPIOD6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOD,&GPIO_InitStructure); //��ʼ��D5��D6

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
  USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
  USART_Init(USART2, &USART_InitStructure); //��ʼ������2
	
  USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2 

	
	//USART_ClearFlag(USART2, USART_FLAG_TC);
	
#if EN_USART2_RX	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
	
}

/****************************************************************************************************
* ��  ��: void USART2_IRQHandler(void)
* ��  ��: USART2�жϺ���������ʹ��
* ��  ��: ��
* ����ֵ: ��
* ��  ע: �����ߵ���ʱ����USART1��������������֡�Ľ��� �����ж�������ж�����ܽ���Ա�����
****************************************************************************************************/
void USART2_IRQHandler(void)
{
	uint8_t clear = clear; //���������������Ա�����֡�û���õ�����������ľ�����ʾ
	uint8_t res;
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //�����ж�
	{ 
		res = USART2->DR;
//		ANO_DT_Data_Receive_Prepare(res); //��λ�����ݽ��������
	}else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) //�����ж�
	{
		clear = USART2->SR; //��SR�Ĵ���
		clear = USART2->DR; //��DR�Ĵ������ȶ�SR,�ٶ�DR,����Ϊ�����IDIE�жϣ�
		
	}
	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}


void Usart2_Send(uint8_t *data, uint8_t length)
{
	uint8_t  i;
		for(i=0;i<length;i++)
		{
			 USART_SendData(USART2, *(data+i));
			while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
			{}
		}

}



//void USART4_Init(unsigned long bound)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//  
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE );
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE );
//  
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10����ΪUSART4
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11����ΪUSART4
//	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC10��GPIOC11
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
//	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��C10��C11
//  
//  USART_InitStructure.USART_BaudRate = bound; //�����ʣ�
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b; //����λ8λ��
//  USART_InitStructure.USART_StopBits = USART_StopBits_1; //ֹͣλ1λ��
//  USART_InitStructure.USART_Parity = USART_Parity_No ; //��У��λ��
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ�����أ�
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�ģʽ��
//  USART_Init(UART4, &USART_InitStructure);//���ô��ڲ�����
//  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//�ж�	
//	USART_Cmd(UART4, ENABLE); //ʹ�ܴ��ڣ�
//	
//	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//����2�ж�ͨ��
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�2
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
//  RingBuff_Init(&OpticalFlow_Ringbuf);
////	OpticalFlow_Is_Work=Config_Init_Uart();//�������Ƿ����
//}

//void Usart4_Send(uint8_t *data, uint8_t length)
//{
//	uint8_t  i;
//		for(i=0;i<length;i++)
//		{
//			 USART_SendData(UART4, *(data+i));
//			while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET)
//			{}
//		}
//}

//void USART4_Send(u8 Data) //����һ���ֽڣ�
//{ 
//  USART_SendData(UART4,Data);
//  while( USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET );
//}


//void UART4_Send(u8 *Data) //�����ַ�����
//{
//  while(*Data)
//    USART4_Send(*Data++);
//}

//void UART4_IRQHandler(void) //�жϴ�������
//{ 
//  if(USART_GetITStatus(UART4, USART_IT_RXNE) == SET) //�ж��Ƿ����жϣ�
//	{
//			USART_ClearFlag(UART4, USART_IT_RXNE); //�����־λ
//      RingBuf_Write(USART_ReceiveData(UART4),&OpticalFlow_Ringbuf,28);//�����ζ�������д����
//	}
//}
