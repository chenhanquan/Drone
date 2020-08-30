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


extern MPU6050Data_TypeDef MPUData;
extern FilterData_Type FilterData;
extern LED_Type xLED;
extern Remote_Type xRemote;
extern Drone_Type MyDrone;

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
static void vIMUCalculation(MPU6050Data_TypeDef *xMPUData,FilterData_Type*xFilterData);    //��̬����

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
	vMotorInit();
	
	//������ʼ��
	MyDrone.State=Lock;
	vPIDParaInit();
	vQuadInit(&FilterData);        //��Ԫ�����ݳ�ʼ��
}

/**
 * @brief ���˻���������
 * @param None
 * @return None
 */
void vDroneLock()
{
	xLED.FlashTime=150;
	xLED.status = Flash;    //LED��˸
	xLED.color = RED;
	vMotorLock();    //�������
	while(1)
	{
		vLEDDisplay(&xLED);             //LED����
		vRemoteDataHandle(&xRemote);    //ң�ظ���
		vSendDataPolling();             //������λ������
		if(xRemote.Ch5_SWA>REMOTE_MIDDLE&&xRemote.Thro<REMOTE_THRO_LIMIT_DOWN){MyDrone.State=unLock;break;}    //�˳�����״̬
	}
}

/**
 * @brief ���˻���������
 * @param None
 * @return None
 */
void vDroneunLock()
{
	
  MPU6050_Calibration(&MPUData); //У׼
	xLED.status = AlwaysOn;        //LED����
	xLED.color = BLUE;
	vLEDDisplay(&xLED);
	while(1)
	{
		static u32 uTaskLastTime[CONTROL_TASK]={0};     //��¼ÿ��������һ��ִ�е�ʱ��
	  u8 uTaskDuringTime[CONTROL_TASK]={2,100,5};     //ÿ������Ŀ���ʱ��������λms
		vDroneSatetyCheck();                            //���彡�����
		vRemoteDataHandle(&xRemote);                    //ң�����ݸ���
		if(MyDrone.State==Lock){break;}                 //��������ѭ��
		for(u8 i=0;i<CONTROL_TASK;i++)                  //��ѯ����
 	 {
		if((uSystickCount-uTaskLastTime[i])<uTaskDuringTime[i]) continue;    //�ж������Ƿ�����ʱ��
		uTaskLastTime[i]=uSystickCount;    //��¼����ִ�е�ʱ���
		switch(i)
		{
			case 0:vIMUCalculation(&MPUData,&FilterData);break;    //����1��������̬�� 2ms
			case 1:{                                               //����2���˳��������� 100ms
				static u8 outflag=0;
				if(outflag==1&&xRemote.Ch5_SWA<REMOTE_MIDDLE){MyDrone.State=Lock;outflag=0;}else{outflag=0;}
				if(xRemote.Ch5_SWA<REMOTE_MIDDLE){outflag++;}
				}break;
			case 2:vDroneControl();break;    //����3��������� 5ms
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
	if((fabs(FilterData.roll)>45.0f||fabs(FilterData.pitch)>45.0f) && (fabs(MPUData.acc[0])>8.0f||fabs(MPUData.acc[1])>8.0f))
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
	vPIDCalculate(&PID_Roll_Angle,xRemote.fAngelRoll,FilterData.roll);
	vPIDCalculate(&PID_Pitch_Angle,xRemote.fAngelPitch,FilterData.pitch);
	
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
static void vIMUCalculation(MPU6050Data_TypeDef *xMPUData,FilterData_Type*xFilterData)
{
	MPU6050_GetGyroAccData(xMPUData);    //��ý��ٶȺͼ��ٶ�����
	MPU6050_RawDataHandle(xMPUData);     //�򵥴���ԭʼ����
	for(u8 i=0;i<3;i++)    //���ݴ���
	{
		xFilterData->acc[i]=xMPUData->acchandle[i];
		xFilterData->gyro[i]=xMPUData->gyrohandle[i];
	}
	vComFilter(xFilterData);    //�����ں�
}
