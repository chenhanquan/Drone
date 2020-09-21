/**
  ******************************************************************************
  * @file    allcontrol.c
  * @author  CHEN HANQUAN
  * @version V1.0
  * @date    16-August-2020
  * @brief   ���˻�����ϵͳ��������
  * @note    
	******************************************************************************
  */
#include "allcontrol.h"
#include "math.h"

#define REMOTE_MIDDLE 1500           //ң����ֵ
#define REMOTE_THRO_LIMIT_DOWN 99   //��������
#define RADTODEG 3.1415926f         //���ȵ��Ƕ�ת��
#define CONTROL_TASK 3              //�������������
#define SAVEFIGURE 100000000         //��������λ��
#define CALIBRATIONDURINGTIME  30   //У׼ʱ��  ��λ��


extern MPU6050Data_TypeDef MPUData;
extern AK8975Data_Type CompassData;    //���������ݱ���ṹ��
extern LED_Type xLED;
extern Remote_Type xRemote;
extern Drone_Type MyDrone;
extern KalmanData_Type KalmanData;

u16 uMotor1PWM=0,uMotor2PWM=0,uMotor3PWM=0,uMotor4PWM=0;

//�⻷�ǶȻ�
PID_Type PID_Roll_Angle;
PID_Type PID_Pitch_Angle;
PID_Type PID_Yaw_Angle;
//�ڻ����ٶȻ�
PID_Type PID_Roll_Rate;
PID_Type PID_Pitch_Rate;
PID_Type PID_Yaw_Rate;

static void vPIDParaInit(void);         //PID������ʼ��
static void vDroneSatetyCheck(void);    //���˻���ȫ���
static void vDroneControl(void);        //��������
static void vDroneCalibration(void);    //���˻�У׼
static void vDroneReadCalData(void);    //���˻���У׼����
static void vIMUCalculation(MPU6050Data_TypeDef *xMPUData,KalmanData_Type*KalmanData,AK8975Data_Type*CompassData);    //��̬����

/**
 * @brief  ���г�ʼ��
 * @param None
 * @return None
 */
void vAllInit()
{
	//Ӳ����ʼ��
	vPPMInit();
	USART1_init(115200);
	vLEDInit();
	I2c_Soft_Init();
	MPU6050_Init();
	AK8975_Init();
	vMotorInit();
	
	//������ʼ��
	MyDrone.State=Lock;
	vPIDParaInit();

}

/**
 * @brief ���˻���������
 * @param None
 * @return None
 */
void vDroneLock()
{
	vMotorLock();    //�������
	while(1)
	{
		xLED.status = AlwaysOn;    //��Ƴ���
		xLED.color = RED;
		vLEDDisplay(&xLED);             //LED����
		vRemoteDataHandle(&xRemote);    //ң�ظ���
		vSendDataPolling();             //������λ������
		if(xRemote.Ch5_SWA>REMOTE_MIDDLE&&xRemote.Thro<REMOTE_THRO_LIMIT_DOWN){MyDrone.State=unLock;break;}    //�˳�����״̬
		if(xRemote.Ch1_Roll>1800&&xRemote.Ch2_Pitch>1800&&xRemote.Ch3_Thro>1800&&xRemote.Ch4_Yaw>1800&&MyDrone.State==Lock){MyDrone.Mode=Calibration;vDroneCalibration();}    //����У׼
	}
}

/**
 * @brief ���˻������������÷�������ʱ����������������ʱ��
 * @param None
 * @return None
 */
void vDroneunLock()
{
	static u32 uTaskLastTime[CONTROL_TASK]={0};     //��¼ÿ��������һ��ִ�е�ʱ��
	static u8 uTaskDuringTime[CONTROL_TASK]={2,100,5};     //ÿ������Ŀ���ʱ��������λms
	xLED.status = AlwaysOn;        //LED����
	xLED.color = BLUE;
	vLEDDisplay(&xLED);
	vDroneReadCalData();    //���˻���У׼����
	while(1)
	{
		vDroneSatetyCheck();                            //���彡�����
		vRemoteDataHandle(&xRemote);                    //ң�����ݸ���
		if(MyDrone.State==Lock){break;}                 //��������ѭ��
		for(u8 i=0;i<CONTROL_TASK;i++)                  //��ѯ����
 	 {
		if((uSystickCount-uTaskLastTime[i])<uTaskDuringTime[i]) continue;    //�ж������Ƿ�����ʱ��
		uTaskLastTime[i]=uSystickCount;    //��¼����ִ�е�ʱ���
		switch(i)
		{
			case 0:vIMUCalculation(&MPUData,&KalmanData,&CompassData);break;    //����1��������̬�� 2ms
			case 1:{                                               //����2���˳��������� 100ms
				static u8 outflag=0;
				if(outflag==1&&xRemote.Ch5_SWA<REMOTE_MIDDLE){MyDrone.State=Lock;outflag=0;}else{outflag=0;}
				if(xRemote.Ch5_SWA<REMOTE_MIDDLE){outflag++;}
				}break;
			case 2:vDroneControl();break;    //����3��������� 5ms
			default: break;
		}
	 }
	 vSendDataPolling();    //������λ����������
	}
}

/**
 * @brief ���˻��������
 * @param None
 * @return None
 */
static void vDroneSatetyCheck()
{
	if((fabs(KalmanData.roll)>45.0f||fabs(KalmanData.pitch)>45.0f) && (fabs(MPUData.acc[0])>8.0f||fabs(MPUData.acc[1])>8.0f))
	{
		MyDrone.State=Lock;
	}
}

/**
 * @brief ���˻���������
 * @param None
 * @return None
 */
static void vDroneControl()
{
	uMotor1PWM=0,uMotor2PWM=0,uMotor3PWM=0,uMotor4PWM=0;
	//�ǶȻ�
	vPIDCalculate(&PID_Roll_Angle,xRemote.fAngelRoll,KalmanData.roll);
	vPIDCalculate(&PID_Pitch_Angle,xRemote.fAngelPitch,KalmanData.pitch);
	
	//���ٶȻ�
	vPIDCalculate(&PID_Roll_Rate,PID_Roll_Angle.Output,MPUData.gyrohandle[0]*RADTODEG);
	vPIDCalculate(&PID_Pitch_Rate,PID_Pitch_Angle.Output,MPUData.gyrohandle[1]*RADTODEG);
	vPIDCalculate(&PID_Yaw_Rate,xRemote.fAngelYaw,MPUData.gyrohandle[2]*RADTODEG);
	
	if(xRemote.Thro>REMOTE_THRO_LIMIT_DOWN&&MyDrone.State==unLock)
	{
		uMotor1PWM = xRemote.Thro + PID_Roll_Rate.Output - PID_Pitch_Rate.Output - PID_Yaw_Rate.Output;
		uMotor2PWM = xRemote.Thro - PID_Roll_Rate.Output - PID_Pitch_Rate.Output + PID_Yaw_Rate.Output;
		uMotor3PWM = xRemote.Thro - PID_Roll_Rate.Output + PID_Pitch_Rate.Output - PID_Yaw_Rate.Output;
		uMotor4PWM = xRemote.Thro + PID_Roll_Rate.Output + PID_Pitch_Rate.Output + PID_Yaw_Rate.Output;
	}
	vMotorPWMSet(uMotor1PWM,uMotor2PWM,uMotor3PWM,uMotor4PWM);
}

/**
 * @brief PID������ʼ��
 * @param None
 * @return None
 */
static void vPIDParaInit()
{
	//�ǶȻ�
	PID_Roll_Angle.Kp = 1.8;
	PID_Roll_Angle.Ki = 0.008;
	PID_Roll_Angle.Kd = 0.05;
	PID_Pitch_Angle.Kp = 1.8;
	PID_Pitch_Angle.Ki = 0.008;
	PID_Pitch_Angle.Kd = 0.05;
	PID_Yaw_Angle.Kp = 3.5;
	PID_Yaw_Angle.Ki = 0;
	PID_Yaw_Angle.Kd = 1;
	
	//���ٶȻ�
	PID_Roll_Rate.Kp = 0.93;
	PID_Roll_Rate.Ki = 0.005;
	PID_Roll_Rate.Kd = 0.86;
	PID_Pitch_Rate.Kp = 0.93;
	PID_Pitch_Rate.Ki = 0.005;
	PID_Pitch_Rate.Kd = 0.86;
	PID_Yaw_Rate.Kp = 2;
	PID_Yaw_Rate.Ki = 0.05;
	PID_Yaw_Rate.Kd = 1;
	
	//�ǶȻ��������Ʋ���
	PID_Roll_Angle.Integral=0;        //����ֵ��ʼ����0
	PID_Roll_Angle.IntIimit=1000;     //�����޷�
	PID_Roll_Angle.IntRange=100;      //������ֻ��ִ�
	PID_Roll_Rate.Integral=0;        //����ֵ��ʼ����0
	PID_Roll_Rate.IntIimit=2000;     //�����޷�
	PID_Roll_Rate.IntRange=250;      //������ֻ��ִ�
	
	PID_Pitch_Angle.Integral=0;        //����ֵ��ʼ����0
	PID_Pitch_Angle.IntIimit=1000;     //�����޷�
	PID_Pitch_Angle.IntRange=100;      //������ֻ��ִ�
	PID_Pitch_Rate.Integral=0;        //����ֵ��ʼ����0
	PID_Pitch_Rate.IntIimit=2000;     //�����޷�
	PID_Pitch_Rate.IntRange=250;      //������ֻ��ִ�
	
	PID_Yaw_Angle.Integral=0;        //����ֵ��ʼ����0
	PID_Yaw_Angle.IntIimit=200;     //�����޷�
	PID_Yaw_Angle.IntRange=150;      //������ֻ��ִ�
	PID_Yaw_Rate.Integral=0;        //����ֵ��ʼ����0
	PID_Yaw_Rate.IntIimit=1200;     //�����޷�
	PID_Yaw_Rate.IntRange=35;      //������ֻ��ִ�
}

/**
 * @brief ��̬����
 * @param MPU6050Data_TypeDef *xMPUData,FilterData_Type*xFilterData
 * @return None
 */
static void vIMUCalculation(MPU6050Data_TypeDef *xMPUData,KalmanData_Type*KalmanData,AK8975Data_Type*CompassData)
{
	MPU6050_GetGyroAccData(xMPUData);    //��ý��ٶȺͼ��ٶ�����
	MPU6050_RawDataHandle(xMPUData);     //�򵥴���ԭʼ����
	AK8975_GetMagData(CompassData);
	for(u8 i=0;i<3;i++)    //���ݴ���
	{
		KalmanData->raw_acc[i]=xMPUData->acchandle[i];
		KalmanData->raw_gyro[i]=xMPUData->gyrohandle[i];
		KalmanData->raw_mag[i]=CompassData->mag[i];
	}
	Kalman(KalmanData);    //�����ں�
}

static void vDroneCalibration(void)
{
	u8 CalibrateTask=0;
	u8 TotalCalibrateTask=2;
	u8 flag=0;
	u8 mag_flag=0;
	u8 mag_mode=0;
	u8 DataToSend[56]={0};
	while(1)
	{
		vRemoteDataHandle(&xRemote); 
		if(xRemote.Ch3_Thro<1600){flag=0;}
		if(xRemote.Ch3_Thro>1800&&flag==0)
		{
			CalibrateTask++;
			if(CalibrateTask>TotalCalibrateTask){break;}
			flag++;
		}
		switch(CalibrateTask)
		{
			case 1:
			{
				xLED.status=AlwaysOn;
				xLED.color=GREE;
				MPU6050_Calibration(&MPUData);
				break;
			}
			case 2:
			{
				xLED.color=YELLOW;
				xLED.status=Flash;
				if(xRemote.Ch1_Roll<1600){mag_flag=0;}
				if(xRemote.Ch1_Roll>1800&&mag_flag==0)
				{
					mag_mode++;
					mag_flag++;
					if(mag_mode>2){mag_mode=0;}
				}
				switch(mag_mode)
				{
					case 0:xLED.FlashTime=900;AK8975_MagXYCalibration(&CompassData);break;
					case 1:xLED.FlashTime=500;AK8975_MagZCalibration(&CompassData);break;
					case 2:xLED.FlashTime=200;AK8975_GetGlobal0z(&CompassData);break;
				}
				break;
			}
		}
		vLEDDisplay(&xLED);
	}
	for(u8 i=0;i<3;i++)
	{
		for(u8 j=0;j<4;j++)
		{
			DataToSend[4*i+j]=((s32)(MPUData.acc_offset[i]*SAVEFIGURE))>>(8*(3-j));
			DataToSend[12+4*i+j]=((s32)(MPUData.gyro_offset[i]*SAVEFIGURE))>>(8*(3-j));
			DataToSend[24+4*i+j]=((s32)(CompassData.mag_offset[i]*SAVEFIGURE))>>(8*(3-j));
			DataToSend[36+4*i+j]=((s32)(CompassData.mag_an[i]*SAVEFIGURE))>>(8*(3-j));

		}
	}
	for(u8 i=0;i<4;i++)
	{
		DataToSend[48+i]=((s32)(CompassData.Global_B0x*SAVEFIGURE))>>(8*(3-i));
		DataToSend[52+i]=((s32)(CompassData.Global_B0z*SAVEFIGURE))>>(8*(3-i));
	}

	for(u8 i=0;i<6;i++)
	{
		AT24C02_PageWrite(i+1,(DataToSend+8*i));
		delay_ms(2);
	}
	MyDrone.State=Lock;
	MyDrone.Mode=Normal;
	AT24C02_ByteWrite(0 ,0xaa);
}

static void vDroneReadCalData(void)    //���˻���У׼����
{
	u8 temp=0;
	AT24C02_NByteRandomRead(0x00 ,1,&temp);
	if(temp==0xaa)
		{
			u8 test[8]={0};
			s32 *numtest;
			u8 DataReceive[56]={0};
			s32 u8Tos32[14]={0};
			AT24C02_NByteRandomRead(8 ,8,test);
			AT24C02_NByteRandomRead(8 ,56,DataReceive);
			for(u8 i=0;i<14;i++)
			{
				u8Tos32[i]=(DataReceive[4*i]<<24)|(DataReceive[4*i+1])<<16|(DataReceive[4*i+2])<<8|(DataReceive[4*i+3]);
			}
			for(u8 i=0;i<3;i++)
			{				
					MPUData.acc_offset[i]=((float)u8Tos32[i])/SAVEFIGURE;
					MPUData.gyro_offset[i]=((float)u8Tos32[i+3])/SAVEFIGURE;
					CompassData.mag_offset[i]=((float)u8Tos32[i+6])/SAVEFIGURE;
					CompassData.mag_an[i]=((float)u8Tos32[i+9])/SAVEFIGURE;
			}
			CompassData.Global_B0x=((float)u8Tos32[12])/SAVEFIGURE;
			CompassData.Global_B0z=((float)u8Tos32[13])/SAVEFIGURE;
			MyDrone.Mode=Normal;
		}
		else
		{
			for(u8 i=0;i<3;i++)
			{
				xLED.status = AlwaysOff;    //�����˸
				vLEDDisplay(&xLED);
				delay_ms(500);
				xLED.status = AlwaysOn;    //�����˸
				xLED.color = RED;
				vLEDDisplay(&xLED);
				delay_ms(500);
			}
			xLED.status = AlwaysOff;    //�����˸
			vLEDDisplay(&xLED);
			MyDrone.State=Lock;
		}
}

