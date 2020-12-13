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
#define RADTODEG 57.295779f         //弧度到角度转换
#define CONTROL_TASK 2              //运行任务的数量


//#define BIGTOLITTLE64(x) ((((*x)&0xFF00000000000000)>>56)| \
//												 (((*x)&0x00FF000000000000)>>40)| \
//												 (((*x)&0x0000FF0000000000)>>24)| \
//												 (((*x)&0x000000FF00000000)>>8)| \
//												 (((*x)&0x00000000FF000000)<<8)| \
//												 (((*x)&0x0000000000FF0000)<<24)| \
//												 (((*x)&0x000000000000FF00)<<40)| \
//												 (((*x)&0x00000000000000FF)<<56))


extern MPU6050Data_TypeDef MPUData;
extern AK8975Data_Type CompassData;    //磁力计数据保存结构体
extern LED_Type xLED;
extern Remote_Type xRemote;
extern Drone_Type MyDrone;
//extern KalmanType EKF_Data;

u16 uMotor1PWM=0,uMotor2PWM=0,uMotor3PWM=0,uMotor4PWM=0;

float Q_quad[4];

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
static void vIMUCalculation(void);    //姿态解算

static void StockTest(void);

/**
 * @brief  所有初始化
 * @param None
 * @return None
 */
void vAllInit()
{
	//硬件初始化
	delay_ms(100);
	vPPMInit();
	//USART1_init(115200);
	uart6_init(115200);
	uart2_init(115200);
	
	vLEDInit();
	I2c_Soft_Init();
	MPU6050_Init();
	AK8975_Init();
	SPL06_Init();
	vMotorInit();
	
	//参数初始化
	MyDrone.State=Lock;
	vPIDParaInit();
	vDroneReadCalData();    //无人机读校准数据
	IMU_Init(MyDrone.quad);
}

/**
 * @brief 无人机锁定任务
 * @param None
 * @return None
 */
void vDroneLock()
{
	static u32 uTaskLastTime[CONTROL_TASK]={0};     //记录每个任务上一次执行的时间
	static u8 uTaskDuringTime[CONTROL_TASK]={10,100};     //每个任务的控制时间间隔，单位ms
	xLED.status = AlwaysOn;    //红灯常亮
	xLED.color = RED;
	vLEDDisplay(&xLED);             //LED更新
	vMotorLock();    //电机锁定
	while(1)
	{
		vSendDataPolling();             //匿名上位机发送
		for(u8 i=0;i<CONTROL_TASK;i++)                  //轮询任务
		{
			if((uSystickCount-uTaskLastTime[i])<uTaskDuringTime[i]) continue;    //判断任务是否到运行时间
			uTaskLastTime[i]=uSystickCount;    //记录本次执行的时间点
			switch(i)
			{
				case 0:vIMUCalculation();break;    //EKF_UpdataRAndQ(&EKF_Data,xRemote.Thro);
				case 1:
				{
					vRemoteDataHandle(&xRemote);
					if(xRemote.Ch1_Roll>1800&&xRemote.Ch3_Thro>1800&&xRemote.Ch2_Pitch>1800&&xRemote.Ch4_Yaw>1800){vDroneCalibration();}
					break;}
				default:break;
			}
		}
		if(xRemote.Ch5_SWA>REMOTE_MIDDLE&&xRemote.Thro<REMOTE_THRO_LIMIT_DOWN){MyDrone.State=unLock;break;}    //退出锁定状态
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
	static u8 uTaskDuringTime[CONTROL_TASK]={10,100};     //每个任务的控制时间间隔，单位ms
	xLED.status = AlwaysOn;        //LED常亮
	xLED.color = BLUE;
	vLEDDisplay(&xLED);
	vDroneReadCalData();    //无人机读校准数据
	
	IMU_Init(MyDrone.quad);
	while(1)
	{
		//vDroneSatetyCheck();                            //机体健康监测
		if(xRemote.Ch5_SWA<REMOTE_MIDDLE){MyDrone.State=Lock;}
		if(MyDrone.State==Lock){break;}                 //锁定跳出循环
		for(u8 i=0;i<CONTROL_TASK;i++)                  //轮询任务
 	 {
		if((uSystickCount-uTaskLastTime[i])<uTaskDuringTime[i]) continue;    //判断任务是否到运行时间
		uTaskLastTime[i]=uSystickCount;    //记录本次执行的时间点
		switch(i)
		{
			case 0:
				vIMUCalculation();
				vDroneControl();
				//StockTest();
				break;    //任务1：计算姿态角 2ms
			case 1:vRemoteDataHandle(&xRemote);break;                     //任务2：遥控更新
			default:break;

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
	if((fabs(MyDrone.angle[0])>45.0f||fabs(MyDrone.angle[1])>45.0f) && (fabs(MPUData.acc[0])>8.0f||fabs(MPUData.acc[1])>8.0f))
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
	static float Yaw_angle0=0;
	float Yaw_target=0;
	uMotor1PWM=0,uMotor2PWM=0,uMotor3PWM=0,uMotor4PWM=0;
	
	if(MyDrone.State==unLock&&xRemote.Thro<150){Yaw_angle0=MyDrone.angle[2];xRemote.YawInt=0;}
	Yaw_target=Yaw_angle0+xRemote.YawInt;
	if(Yaw_target>180)Yaw_target-=360;
	if(Yaw_target<-180)Yaw_target+=360;
	if(Yaw_target-MyDrone.angle[2]>270){Yaw_target=-180-(180-Yaw_target);}
	if(Yaw_target-MyDrone.angle[2]<-270){Yaw_target=180+(180+Yaw_target);}
	
	
	//角度环
	PID_AlterI(&PID_Roll_Angle,xRemote.fAngelRoll,MyDrone.angle[0],10.0f,5.0f);
	PID_AlterI(&PID_Pitch_Angle,xRemote.fAngelPitch,MyDrone.angle[1],10.0f,5.0f);
	PID_AlterI(&PID_Yaw_Angle,Yaw_target,MyDrone.angle[2],10.0f,5.0f);
	
	//角速度环
	PID_Normal(&PID_Roll_Rate,PID_Roll_Angle.Output,MPUData.gyro[0]*RADTODEG);
	PID_Normal(&PID_Pitch_Rate,PID_Pitch_Angle.Output,MPUData.gyro[1]*RADTODEG);
	//PID_Normal(&PID_Yaw_Rate,PID_Yaw_Angle.Output,MPUData.gyro[2]*RADTODEG);
	
	if(MyDrone.State==unLock&&xRemote.Thro>150)    //&&xRemote.Thro>150
	{
		uMotor1PWM = xRemote.Thro + PID_Roll_Rate.Output + PID_Pitch_Rate.Output + PID_Yaw_Angle.Output;//PID_Yaw_Rate.Output;
		uMotor2PWM = xRemote.Thro - PID_Roll_Rate.Output + PID_Pitch_Rate.Output - PID_Yaw_Angle.Output;//PID_Yaw_Rate.Output;
		uMotor3PWM = xRemote.Thro - PID_Roll_Rate.Output - PID_Pitch_Rate.Output + PID_Yaw_Angle.Output;//PID_Yaw_Rate.Output;
		uMotor4PWM = xRemote.Thro + PID_Roll_Rate.Output - PID_Pitch_Rate.Output - PID_Yaw_Angle.Output;//PID_Yaw_Rate.Output;
//		uMotor1PWM = xRemote.Thro;    //电机测试
//		uMotor2PWM = xRemote.Thro;
//		uMotor3PWM = xRemote.Thro;
//		uMotor4PWM = xRemote.Thro;
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
	PID_Roll_Angle.Kp = 3.0;   //0.3 1.5
	PID_Roll_Angle.Ki = 0;
	PID_Roll_Angle.Kd = 0;
	PID_Roll_Angle.Kd_freq=5.0f;
	PID_Roll_Angle.Max_Error=40.0f;
	PID_Roll_Angle.IntIimit=200.0f;
	
	PID_Pitch_Angle.Kp = 3.0;//0.3  1.8
	PID_Pitch_Angle.Ki = 0;
	PID_Pitch_Angle.Kd = 0;
	PID_Pitch_Angle.Kd_freq=5.0f;
	PID_Pitch_Angle.Max_Error=40.0f;
	PID_Pitch_Angle.IntIimit=200.0f;
	
//	PID_Roll_Angle.Kp = 0;   //0.3 1.5
//	PID_Roll_Angle.Ki = 0;
//	PID_Roll_Angle.Kd = 0;
//	PID_Roll_Angle.Kd_freq=5.0f;
//	PID_Roll_Angle.Max_Error=40.0f;
//	PID_Roll_Angle.IntIimit=200.0f;
//	
//	PID_Pitch_Angle.Kp = 0;//0.3  1.8
//	PID_Pitch_Angle.Ki = 0;
//	PID_Pitch_Angle.Kd = 0;
//	PID_Pitch_Angle.Kd_freq=5.0f;
//	PID_Pitch_Angle.Max_Error=40.0f;
//	PID_Pitch_Angle.IntIimit=200.0f;
	
	PID_Yaw_Angle.Kp = 2.2;//0.3  1.8
	PID_Yaw_Angle.Ki = 0;
	PID_Yaw_Angle.Kd = 0;
	PID_Yaw_Angle.Kd_freq=5.0f;
	PID_Yaw_Angle.Max_Error=40.0f;
	PID_Yaw_Angle.IntIimit=200.0f;
	
	//角速度环
	PID_Roll_Rate.Kp = 1.4; //3.2 0.93
	PID_Roll_Rate.Ki = 0.1;
	PID_Roll_Rate.Kd = 0.1; //2.5 0.86
	PID_Roll_Rate.Kd_freq=100.0f;
	PID_Roll_Rate.Max_Error=0.0f;
	PID_Roll_Rate.IntIimit=150.0f;
	
	PID_Pitch_Rate.Kp = 1.4;
	PID_Pitch_Rate.Ki = 0.1;
	PID_Pitch_Rate.Kd = 0.1;
	PID_Pitch_Rate.Kd_freq=100.0f;
	PID_Pitch_Rate.Max_Error=0.0f;
	PID_Pitch_Rate.IntIimit=150.0f;
	
//	PID_Roll_Rate.Kp = 0; //3.2 0.93
//	PID_Roll_Rate.Ki = 0;
//	PID_Roll_Rate.Kd = 0; //2.5 0.86
//	PID_Roll_Rate.Kd_freq=100.0f;
//	PID_Roll_Rate.Max_Error=0.0f;
//	PID_Roll_Rate.IntIimit=150.0f;
//	
//	PID_Pitch_Rate.Kp = 0;
//	PID_Pitch_Rate.Ki = 0;
//	PID_Pitch_Rate.Kd = 0;
//	PID_Pitch_Rate.Kd_freq=100.0f;
//	PID_Pitch_Rate.Max_Error=0.0f;
//	PID_Pitch_Rate.IntIimit=150.0f;
//	
//	PID_Yaw_Rate.Kp = 1.45;
//	PID_Yaw_Rate.Ki = 0;
//	PID_Yaw_Rate.Kd = 0;
//	PID_Yaw_Rate.Kd_freq=100.0f;
//	PID_Yaw_Rate.Max_Error=0.0f;
//	PID_Yaw_Rate.IntIimit=150.0f;
	
}

/**
 * @brief 姿态解算
 * @param MPU6050Data_TypeDef *xMPUData,FilterData_Type*xFilterData
 * @return None
 */
static void vIMUCalculation()
{
//	static u32 count=0,count1=0;
//	count=micros();
	MPU6050_GetGyroAccData(&MPUData);    //获得角速度和加速度数
	MyDrone.Height[0]=SPL06_GetHeight();    //当前高度
	if(xRemote.Thro<100||MyDrone.State==Lock){MyDrone.Height[1]=MyDrone.Height[0];}
	
	AK8975_GetMagData(&CompassData);
	DirectIMU(MyDrone.quad,MPUData.acc,MPUData.gyro,CompassData.mag,MyDrone.angle,xRemote.Thro);
	
	Altitude_GetRelativeAltitude(MyDrone.Height,MyDrone.angle,MPUData.acc);
	
//	count1=micros()-count;
//	
//	count=0;
}

static void vDroneCalibration(void)
{
	float gyro_offset[3]={0},acc_offset[3]={0};
	MPU6050_Calibration(gyro_offset,acc_offset);
	
	SaveFloatData(gyro_offset[0],8);
	SaveFloatData(gyro_offset[1],12);
	SaveFloatData(gyro_offset[2],16);
	
	SaveFloatData(acc_offset[0],20);
	SaveFloatData(acc_offset[1],24);
	SaveFloatData(acc_offset[2],28);
}



static void vDroneReadCalData(void)    //无人机读校准数据
{
//	 MPU6050_GyroCalibration(&MPUData);
//	 //**********************加速度计校准*************************
//			MPUData.acc_offset[0]=0.125027969f;
//			MPUData.acc_offset[1]=-0.374655068f;
//			MPUData.acc_offset[2]=0.646567166f;
//			MPUData.acc_scale[0]=1.00452876f;
//			MPUData.acc_scale[1]=0.996431828f;
//			MPUData.acc_scale[2]=0.995548069f;
//			
//		//**********************************************************
//	
	 MPUData.gyro_offset[0]=ReadFloatData(8);
	 MPUData.gyro_offset[1]=ReadFloatData(12);
	 MPUData.gyro_offset[2]=ReadFloatData(16);
	
	 MPUData.acc_offset[0]=ReadFloatData(20);
	 MPUData.acc_offset[1]=ReadFloatData(24);
	 MPUData.acc_offset[2]=ReadFloatData(28);
	
		//***********************磁力计校准*************************
			CompassData.mag_offset[0]=-6.60f;
			CompassData.mag_offset[1]=8.48;
			CompassData.mag_offset[2]=58.35;
			CompassData.mag_an[0]=32.09;
			CompassData.mag_an[1]=33.49;
			CompassData.mag_an[2]=30.35;
			
		//**********************************************************
		
//		  CompassData.mag_offset[0]=-7.8238f;
//			CompassData.mag_offset[1]=-3.6387;
//			CompassData.mag_offset[2]=54.4228;
//			CompassData.mag_an[0]=53.4910;
//			CompassData.mag_an[1]=49.8002;
//			CompassData.mag_an[2]=46.8213;
			
//			CompassData.mag_offset[0]=1.0763;
//			CompassData.mag_offset[1]=14.8003;
//			CompassData.mag_offset[2]=48.1512;
//			CompassData.mag_an[0]=31.4499;
//			CompassData.mag_an[1]=30.7798;
//			CompassData.mag_an[2]=33.6858;
}


static void StockTest()
{
	static u32 tick=0;
	static u8 flag=0;
	static u16 PWM[4]={0};
	if(xRemote.Ch1_Roll>1800)flag=1;
	if(uSystickCount-tick>2000&&flag==1)
	{
		tick=uSystickCount;
		PWM[0]+=100;
		PWM[1]+=100;
		PWM[2]+=100;
		PWM[3]+=100;
		vMotorPWMSet(PWM[0],PWM[1],PWM[2],PWM[3]);
	}
	if(PWM[0]>1000)
	{
		flag=0;
		PWM[0]=0;
		PWM[1]=0;
		PWM[2]=0;
		PWM[3]=0;
		vMotorLock();
	}
}
