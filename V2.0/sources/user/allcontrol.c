/**
  ******************************************************************************
  * @file    allcontrol.c
  * @author  CHEN HANQUAN
  * @version V1.0
  * @date    16-August-2020
  * @brief   无人机控制系统程序主干
  * @note    
	******************************************************************************
  */
#include "allcontrol.h"
#include "math.h"

#define REMOTE_MIDDLE 1500           //遥控中值
#define REMOTE_THRO_LIMIT_DOWN 99   //油门下限
#define RADTODEG 3.1415926f         //弧度到角度转换
#define CONTROL_TASK 3              //运行任务的数量
#define SAVEFIGURE 100000000         //保存数据位数
#define CALIBRATIONDURINGTIME  30   //校准时间  单位秒


extern MPU6050Data_TypeDef MPUData;
extern AK8975Data_Type CompassData;    //磁力计数据保存结构体
extern LED_Type xLED;
extern Remote_Type xRemote;
extern Drone_Type MyDrone;
extern KalmanData_Type KalmanData;

u16 uMotor1PWM=0,uMotor2PWM=0,uMotor3PWM=0,uMotor4PWM=0;

//外环角度环
PID_Type PID_Roll_Angle;
PID_Type PID_Pitch_Angle;
PID_Type PID_Yaw_Angle;
//内环角速度环
PID_Type PID_Roll_Rate;
PID_Type PID_Pitch_Rate;
PID_Type PID_Yaw_Rate;

static void vPIDParaInit(void);         //PID参数初始化
static void vDroneSatetyCheck(void);    //无人机安全监测
static void vDroneControl(void);        //驱动控制
static void vDroneCalibration(void);    //无人机校准
static void vDroneReadCalData(void);    //无人机读校准数据
static void vIMUCalculation(MPU6050Data_TypeDef *xMPUData,KalmanData_Type*KalmanData,AK8975Data_Type*CompassData);    //姿态解算

/**
 * @brief  所有初始化
 * @param None
 * @return None
 */
void vAllInit()
{
	//硬件初始化
	vPPMInit();
	USART1_init(115200);
	vLEDInit();
	I2c_Soft_Init();
	MPU6050_Init();
	AK8975_Init();
	vMotorInit();
	
	//参数初始化
	MyDrone.State=Lock;
	vPIDParaInit();

}

/**
 * @brief 无人机锁定任务
 * @param None
 * @return None
 */
void vDroneLock()
{
	vMotorLock();    //电机锁定
	while(1)
	{
		xLED.status = AlwaysOn;    //红灯常亮
		xLED.color = RED;
		vLEDDisplay(&xLED);             //LED更新
		vRemoteDataHandle(&xRemote);    //遥控更新
		vSendDataPolling();             //匿名上位机发送
		if(xRemote.Ch5_SWA>REMOTE_MIDDLE&&xRemote.Thro<REMOTE_THRO_LIMIT_DOWN){MyDrone.State=unLock;break;}    //退出锁定状态
		if(xRemote.Ch1_Roll>1800&&xRemote.Ch2_Pitch>1800&&xRemote.Ch3_Thro>1800&&xRemote.Ch4_Yaw>1800&&MyDrone.State==Lock){MyDrone.Mode=Calibration;vDroneCalibration();}    //进入校准
	}
}

/**
 * @brief 无人机解锁任务，利用非阻塞延时方法控制任务运行时间
 * @param None
 * @return None
 */
void vDroneunLock()
{
	static u32 uTaskLastTime[CONTROL_TASK]={0};     //记录每个任务上一次执行的时间
	static u8 uTaskDuringTime[CONTROL_TASK]={2,100,5};     //每个任务的控制时间间隔，单位ms
	xLED.status = AlwaysOn;        //LED常亮
	xLED.color = BLUE;
	vLEDDisplay(&xLED);
	vDroneReadCalData();    //无人机读校准数据
	while(1)
	{
		vDroneSatetyCheck();                            //机体健康监测
		vRemoteDataHandle(&xRemote);                    //遥控数据更新
		if(MyDrone.State==Lock){break;}                 //锁定跳出循环
		for(u8 i=0;i<CONTROL_TASK;i++)                  //轮询任务
 	 {
		if((uSystickCount-uTaskLastTime[i])<uTaskDuringTime[i]) continue;    //判断任务是否到运行时间
		uTaskLastTime[i]=uSystickCount;    //记录本次执行的时间点
		switch(i)
		{
			case 0:vIMUCalculation(&MPUData,&KalmanData,&CompassData);break;    //任务1：计算姿态角 2ms
			case 1:{                                               //任务2：退出程序消抖 100ms
				static u8 outflag=0;
				if(outflag==1&&xRemote.Ch5_SWA<REMOTE_MIDDLE){MyDrone.State=Lock;outflag=0;}else{outflag=0;}
				if(xRemote.Ch5_SWA<REMOTE_MIDDLE){outflag++;}
				}break;
			case 2:vDroneControl();break;    //任务3：电机控制 5ms
			default: break;
		}
	 }
	 vSendDataPolling();    //匿名上位机发送数据
	}
}

/**
 * @brief 无人机健康监测
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
 * @brief 无人机驱动控制
 * @param None
 * @return None
 */
static void vDroneControl()
{
	uMotor1PWM=0,uMotor2PWM=0,uMotor3PWM=0,uMotor4PWM=0;
	//角度环
	vPIDCalculate(&PID_Roll_Angle,xRemote.fAngelRoll,KalmanData.roll);
	vPIDCalculate(&PID_Pitch_Angle,xRemote.fAngelPitch,KalmanData.pitch);
	
	//角速度环
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
 * @brief PID参数初始化
 * @param None
 * @return None
 */
static void vPIDParaInit()
{
	//角度环
	PID_Roll_Angle.Kp = 1.8;
	PID_Roll_Angle.Ki = 0.008;
	PID_Roll_Angle.Kd = 0.05;
	PID_Pitch_Angle.Kp = 1.8;
	PID_Pitch_Angle.Ki = 0.008;
	PID_Pitch_Angle.Kd = 0.05;
	PID_Yaw_Angle.Kp = 3.5;
	PID_Yaw_Angle.Ki = 0;
	PID_Yaw_Angle.Kd = 1;
	
	//角速度环
	PID_Roll_Rate.Kp = 0.93;
	PID_Roll_Rate.Ki = 0.005;
	PID_Roll_Rate.Kd = 0.86;
	PID_Pitch_Rate.Kp = 0.93;
	PID_Pitch_Rate.Ki = 0.005;
	PID_Pitch_Rate.Kd = 0.86;
	PID_Yaw_Rate.Kp = 2;
	PID_Yaw_Rate.Ki = 0.05;
	PID_Yaw_Rate.Kd = 1;
	
	//角度环其他控制参数
	PID_Roll_Angle.Integral=0;        //积分值初始化清0
	PID_Roll_Angle.IntIimit=1000;     //积分限幅
	PID_Roll_Angle.IntRange=100;      //分离积分积分带
	PID_Roll_Rate.Integral=0;        //积分值初始化清0
	PID_Roll_Rate.IntIimit=2000;     //积分限幅
	PID_Roll_Rate.IntRange=250;      //分离积分积分带
	
	PID_Pitch_Angle.Integral=0;        //积分值初始化清0
	PID_Pitch_Angle.IntIimit=1000;     //积分限幅
	PID_Pitch_Angle.IntRange=100;      //分离积分积分带
	PID_Pitch_Rate.Integral=0;        //积分值初始化清0
	PID_Pitch_Rate.IntIimit=2000;     //积分限幅
	PID_Pitch_Rate.IntRange=250;      //分离积分积分带
	
	PID_Yaw_Angle.Integral=0;        //积分值初始化清0
	PID_Yaw_Angle.IntIimit=200;     //积分限幅
	PID_Yaw_Angle.IntRange=150;      //分离积分积分带
	PID_Yaw_Rate.Integral=0;        //积分值初始化清0
	PID_Yaw_Rate.IntIimit=1200;     //积分限幅
	PID_Yaw_Rate.IntRange=35;      //分离积分积分带
}

/**
 * @brief 姿态解算
 * @param MPU6050Data_TypeDef *xMPUData,FilterData_Type*xFilterData
 * @return None
 */
static void vIMUCalculation(MPU6050Data_TypeDef *xMPUData,KalmanData_Type*KalmanData,AK8975Data_Type*CompassData)
{
	MPU6050_GetGyroAccData(xMPUData);    //获得角速度和加速度数据
	MPU6050_RawDataHandle(xMPUData);     //简单处理原始数据
	AK8975_GetMagData(CompassData);
	for(u8 i=0;i<3;i++)    //数据传输
	{
		KalmanData->raw_acc[i]=xMPUData->acchandle[i];
		KalmanData->raw_gyro[i]=xMPUData->gyrohandle[i];
		KalmanData->raw_mag[i]=CompassData->mag[i];
	}
	Kalman(KalmanData);    //数据融合
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

static void vDroneReadCalData(void)    //无人机读校准数据
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
				xLED.status = AlwaysOff;    //红灯闪烁
				vLEDDisplay(&xLED);
				delay_ms(500);
				xLED.status = AlwaysOn;    //红灯闪烁
				xLED.color = RED;
				vLEDDisplay(&xLED);
				delay_ms(500);
			}
			xLED.status = AlwaysOff;    //红灯闪烁
			vLEDDisplay(&xLED);
			MyDrone.State=Lock;
		}
}

