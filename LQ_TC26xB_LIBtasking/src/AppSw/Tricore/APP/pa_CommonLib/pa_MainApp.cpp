
extern "C"
{
#include "pa_MainApp.h"
#include "pa_CommonLib/pa_CrossCalc.h"
#include "pa_CommonLib/pa_MotorManager.h"
#include "pa_CommonLib/pa_app/pa_OLED/pa_oled.h"
#include "pa_CommonLib/pa_drv/pa_IIC.h"
#include "pa_CommonLib/pa_app/pa_RDA5804/pa_RDA5807.h"
#include "pa_CommonLib/pa_app/pa_BNO055.h"

#include "stdio.h"
#include "LQ_UART.h"
#include <LQ_ADC.h>
#include <LQ_STM.h>
#include <Ifx_FftF32.h>
#include <LQ_FFT.h>
#include <LQ_GPT12_ENC.h>
#include "LQ_GPIO_LED.h"
#include "string.h"

#include <LQ_Atom_Motor.h>
#include "math.h"
}

#include "pa_CommonLib/pa_PID.h"
#include "pa_CommonLib/pa_UartManager.h"
#include "pa_CommonLib/pa_GlobalCpp.h"
#include "pa_CommonLib/pa_UltrasonicDistance.h"
#include "pa_CommonLib/pa_app/pa_VoiceDataProcessor/pa_VoiceDataProcessor.h"

unsigned short adcValue1;
unsigned short adcValue2;
unsigned short adcValue3;
unsigned short adcValue4;

unsigned short adcValue_fm; //fm

pa_VoiceDataProcessor voiceDataProcessor;

int cnt = 0;
char flag_5ms = 1;
char flag_100ms = 1;
char blockCnt=0;
// bool flag_1s=false;
SpeedOfMotors speedOfMotors;

char err_X = 0;
char err_Y = 0;
char err_Y_History[10] = {0};
short err_Fm = 0;
char pre_err_Y = 0;
int pathLength = 0;
bool directionCorrected = false;
char errY_WhenCorrected = 0;

pa_GlobalCpp globalCpp;

pa_GlobalCpp *pa_GlobalCpp::getGlobalCpp()
{
	return &globalCpp;
}

pa_UltrasonicDistance ultrasonicDistance1;
pa_UltrasonicDistance ultrasonicDistance2;
pa_UltrasonicDistance ultrasonicDistance3;
pa_UltrasonicDistance ultrasonicDistance4;

void initVariable()
{ //初始化变量
	speedOfMotors.speedOfM1 = 0;
	speedOfMotors.speedOfM2 = 0;
	speedOfMotors.speedOfM3 = 0;
	speedOfMotors.speedOfM4 = 0;
	globalCpp.pid_Motor1.setPid(12, 0, 0);
	globalCpp.pid_Motor2.setPid(17, 0, 1);
	globalCpp.pid_Motor3.setPid(17, 0, 1);
	globalCpp.pid_Motor4.setPid(17, 0, 1);
	globalCpp.pid_Direction.setPid(30, 0, 8);
	globalCpp.pid_Direction.setMax(800);
	globalCpp.pid_DirectionCalibration.setPid(750, 0, 100);
	globalCpp.pid_DirectionCalibration.setMax(850);

	// //ultrasonicDistance1.init(1);
	// //ultrasonicDistance2.init(2);
}

void initFuncs()
{
	//限位开关
	PIN_InitConfig(P00_2, PIN_MODE_INPUT_PULLDOWN, 0);
	// ATOM_PWM_InitConfig(ATOMPWM0, 4000, 15000);
	// ATOM_PWM_InitConfig(ATOMPWM1, 5000, 15000);
	// ATOM_PWM_InitConfig(ATOMPWM2, 6000, 15000);
	// ATOM_PWM_InitConfig(ATOMPWM3, 7000, 15000);
	// ATOM_PWM_InitConfig(ATOMPWM4, 8000, 15000);
	// ATOM_PWM_InitConfig(ATOMPWM5, 9000, 15000);
	// ATOM_PWM_InitConfig(ATOMPWM6, 3000, 15000);
	// ATOM_PWM_InitConfig(ATOMPWM7, 2000, 15000);
	//初始化功能
	UART_InitConfig(UART2_RX_P14_3, UART2_TX_P14_2, 115200); //2红色线
	//UART_InitConfig(UART1_RX_P02_3, UART1_TX_P02_2, 115200);
	{
		UART_PutStr(UART2, "init\r\n");
	}
	//pa_BNO055_init();

	ADC_InitConfig(ADC0, 100000); //初始化
	ADC_InitConfig(ADC1, 100000);
	ADC_InitConfig(ADC2, 100000);
	ADC_InitConfig(ADC4, 100000);
	ADC_InitConfig(ADC7, 100000);

	//adc采集以及控制 定时器中断
	STM_InitConfig(STM0, STM_Channel_0, 100, []() { //微秒  //UART_PutStr(UART2,"ssss");
		getadc();									//getadc();
		ultrasonicDistance1.checkEcho();
		ultrasonicDistance2.checkEcho();
		ultrasonicDistance3.checkEcho();
		ultrasonicDistance4.checkEcho();
		// if (cnt % 100 == 0)
		// {

		// }
		if (cnt % 50 == 0)
		{
			if(PIN_Read(P00_2)){
				if(blockCnt<100){
					blockCnt++;
				}
			}else{
				// UART_PutStr(UART2, "no block");
				blockCnt=0;
			}
			// cnt = 0;
			flag_5ms = 1;

			speedOfMotors.speedOfM1 = ENC_GetCounter(ENC3_InPut_P02_6);
			speedOfMotors.speedOfM2 = -ENC_GetCounter(ENC2_InPut_P33_7);
			speedOfMotors.speedOfM3 = -ENC_GetCounter(ENC6_InPut_P20_3);
			speedOfMotors.speedOfM4 = ENC_GetCounter(ENC5_InPut_P10_3);

			if (directionCorrected)
			{
				pathLength += (speedOfMotors.speedOfM1 + speedOfMotors.speedOfM2 + speedOfMotors.speedOfM3 + speedOfMotors.speedOfM4) / 4;
				if (fabs(pathLength) > 40000)
				{
					pathLength = 0;
					turnToRedirectionMode();
				}
			}
		}
		if (cnt % 1000==0)
		{
			// cnt = 0;
			flag_100ms = 1;
		}
		if (cnt == 10000)
		{
			cnt = 0;
			// flag_1s = true;
			PIN_Reverse(LED0p);
            PIN_Reverse(LED1p);
            PIN_Reverse(LED2p);
            PIN_Reverse(LED3p);
		}
		cnt++;
	});

	pa_initMotorPwm();

	ENC_InitConfig(ENC2_InPut_P33_7, ENC2_Dir_P33_6);
	//ENC_InitConfig(ENC3_InPut_P02_6, ENC3_Dir_P02_7);//摄像头冲突，不建议用
	ENC_InitConfig(ENC3_InPut_P02_6, ENC3_Dir_P02_7);
	ENC_InitConfig(ENC5_InPut_P10_3, ENC5_Dir_P10_1);
	ENC_InitConfig(ENC6_InPut_P20_3, ENC6_Dir_P20_0);

	// while(1){
	// 	{
	// 		char buf[30] = {0};
	// 		sprintf(buf, "%5d %5d %5d %5d %d\r\n",
	// 				speedOfMotors.speedOfM1,
	// 				speedOfMotors.speedOfM2,
	// 				speedOfMotors.speedOfM3,
	// 				speedOfMotors.speedOfM4,
	// 				pathLength);
	// 		// sprintf(buf, "%f %5d %2d %2d\r\n",
	// 		// 		dirOut, globalCpp.targetSpeed,err_X,err_Y);

	// 		UART_PutStr(UART2, buf);
	// 	}
	// }
	ultrasonicDistance1.init(1);
	ultrasonicDistance2.init(2);
	ultrasonicDistance3.init(3);
	ultrasonicDistance4.init(4);

	pa_IIC_init();

	UART_PutStr(UART2, "before");

	pa_BNO055_init();

	UART_PutStr(UART2, "after");

	OLED_Init(); //初始化OLED
	OLED_Clear();
	OLED_ShowString(0, 0, "helloWorld!", 8);

	// RDA5807_Init(); //RDA5807初始化

	// unsigned short RXFreq = RDA5807_ReadReg(RDA_R00); //pa_IIC_read8(RDA_WRITE,RDA_R00);
	// // 												  // char RSSI=RDA5807_GetRssi();//显示信号强度0~127
	// // 												  // {
	// char txt[30] = {0};
	// sprintf(txt, "Chip:0x%04X", RXFreq);
	// UART_PutStr(UART2, txt);
	// OLED_ShowString(0, 0, txt, 8);

	// RDA5807_Reset(); //软件复位
	// RDA5807_SetVol(15);
	// RDA5807_SetFreq(9500);

	// unsigned char RSSI = RDA5807_GetRssi(); //显示信号强度0~127
	// sprintf(txt, "RSSI:%02d  ", RSSI);
	// UART_PutStr(UART2, txt);

	// while(1){
	// 	delayms(10);
	// 	bno055_vector_t v = pa_BNO055_getVector();
	// 	if(v.x>=0||v.x<360){
	// 		sprintf(txt,"x:%.2f,y:%.2f,z:%.2f\r\n",v.x,v.y,v.z);
	// 	UART_PutStr(UART2, txt);
	// 	}

	// }
	// 	sprintf(txt,"RSSI:%02d  \r\n",RSSI);
	// 	UART_PutStr(UART2,txt);
	// 	//OLED_ShowString(0,2,txt,8);
	// }

	/*rotate box demo****************************/
	// {unsigned char a[128*8]={0};
	// unsigned char b[128*8]={0};
	// memset(b,1,128*8);
	// short angle=0;
	// while(1){
	// 	memset(a,0,128*8);
	// 	float realAngle=angle/180.0*PI;
	// 	int x=60+sin(realAngle)*20;
	// 	int y=30+cos(realAngle)*20;
	// 	for(int i=-3;i<4;i++){
	// 		for(int j=-3;j<4;j++){
	// 			int x1=x+i;
	// 			int y1=y+j;
	// 			a[(y1/8)*128+x1]|=0x01<<(y1%8);
	// 		}
	// 	}

	// 	fill_picture(a);

	// 	angle+=20;
	// 	if(angle==360)angle=0;
	// }}
	/****************************/

	//   TFTSPI_P8X16Str(1,2,txt,u16WHITE,u16BLACK);
}
short xMoveDelay = 0;
short err_Y_whenBlocked = 0;
float currentAngle = 0;
float targetAngle = 0;
char gameStartFlag=0;
char beaconOn=0;
char lastBeaconOn=0;
void startMainTask()
{
	initVariable();
	initFuncs();
	int cnt5ms = 0;

	while (1)
	{
		//执行控制算法
		if (flag_5ms == 1)
		{
			flag_5ms = 0; //50ms清0
			cnt5ms = cnt5ms + 1;
			if (cnt5ms >= 1000)
			{
				globalCpp.targetSpeed = -globalCpp.targetSpeed;

				cnt5ms = 0;
			}
			// pa_updateMotorPwm(1, 1000);
			// pa_updateMotorPwm(2, 1000);
			// pa_updateMotorPwm(3, 1000);
			// pa_updateMotorPwm(4, 1000);
			beaconOn=voiceDataProcessor.isBeaconOn();
			if(lastBeaconOn==0 && beaconOn==1){
				gameStartFlag=100;
			}
			lastBeaconOn=beaconOn;
			if (beaconOn)
			{

			}
			else
			{
				// {
				// 	char buf[30] = {0};
				// 	sprintf(buf, "%d\r\n",
				// 			voiceDataProcessor.rightFftCount);
				// 	// sprintf(buf, "%f %5d %2d %2d\r\n",
				// 	// 		dirOut, globalCpp.targetSpeed,err_X,err_Y);

				// 	UART_PutStr(UART2, buf);
				// }
				// UART_PutStr(UART2, "signal Not Stable or Beacon Off!!\r\n");
			}

			if ( globalCpp.motorDisable || !voiceDataProcessor.isBeaconOn())
			{

				// float directionErr = (currentAngle - targetAngle);
				// if (directionErr > 180)
				// {
				// 	directionErr -= 360;
				// }
				// else if (directionErr < -180)
				// {
				// 	directionErr += 360;
				// }
				// // float dirOut = globalCpp.pid_Direction.calcPid(err_Y < 0 ? err_X : -err_X); //
				// float dirOut = globalCpp.pid_Direction.calcPid(directionErr);
				float dirOut = 0;
				pa_updateMotorPwm(1, dirOut);
				pa_updateMotorPwm(2, -dirOut);
				pa_updateMotorPwm(3, -dirOut);
				pa_updateMotorPwm(4, dirOut);
				globalCpp.pid_Motor1.iSum = 0;
				globalCpp.pid_Motor2.iSum = 0;
				globalCpp.pid_Motor3.iSum = 0;
				globalCpp.pid_Motor4.iSum = 0;
				// directionCorrected = false;
				turnToRedirectionMode();
			}
			else
			{
				// float dirOut =getMotorRotationValueByErr(err_Y < 0 ? err_X : -err_X);

				// float dirOut = globalCpp.pid_DirectionCalibration.calcPid(err_Y < 0 ? err_X : -err_X); //
				// float dirOut = globalCpp.pid_Direction.calcPid(); //
				static char blockIndex='7';
				float yVelocity = err_Y < 0 ? 1000 : -1000;
				float xVelocity = 0;
				static int obstacleAvoidCnt=0;
				static int obstacleAvoidTrigCnt=0;
				char xMoveDirection=err_Y < 0 ? err_X : -err_X;
				static char errX_WhenAvoidTrigged=0;
				static char blockDir=0;
#define distanceThreshold 8
				if (
					ultrasonicDistance1.distance < distanceThreshold 
					||ultrasonicDistance2.distance < distanceThreshold 
					||ultrasonicDistance3.distance < distanceThreshold 
					||ultrasonicDistance4.distance < distanceThreshold 
					||blockCnt>10
					)
				{
					//<-避障编号////////////////////////////////////////
					if (
					ultrasonicDistance1.distance < distanceThreshold 
					){
						blockIndex='1';
						blockDir=1;
					}
					else if (
					ultrasonicDistance2.distance < distanceThreshold 
					){
						blockIndex='2';
						blockDir=1;
					}
					else if (
					ultrasonicDistance3.distance < distanceThreshold 
					){
						blockIndex='3';
						blockDir=-1;
					}
					else if (
					ultrasonicDistance4.distance < distanceThreshold 
					){
						blockIndex='4';
						blockDir=-1;
					}
					//避障编号->////////////////////////////////////////

					// directionCorrected = false;
					turnToRedirectionMode();
					errX_WhenAvoidTrigged=xMoveDirection;
					yVelocity = errY_WhenCorrected < 0 ? -1000 : 1000;
					if(blockDir!=0){
						yVelocity = blockDir < 0 ? -1000 : 1000;
					}
					xVelocity = 0;//xMoveDirection > 0 ? 1400 : -1400;
					pa_updateMotorPwm(1,
									  (yVelocity - xVelocity));
					pa_updateMotorPwm(2,
									  (yVelocity + xVelocity));
					//float out = -globalCpp.pid_Motor3.calcPid(speedOfMotors.speedOfM3 - dirOut);
					pa_updateMotorPwm(3,
									  (yVelocity - xVelocity));
					// pa_updateMotorPwm(2,300);
					pa_updateMotorPwm(4,
									  (yVelocity + xVelocity));

					
					UART_PutStr(UART2, "obstacling Avoidance ");
					UART_PutStr(UART2, &blockIndex);
					UART_PutStr(UART2, " !!\r\n");
					
					obstacleAvoidTrigCnt++;
					#define obstacleAvoidCntValue 110
					if(obstacleAvoidTrigCnt>4){
						obstacleAvoidCnt=obstacleAvoidCntValue;
						obstacleAvoidTrigCnt=0;
					}
				}else if(obstacleAvoidCnt>0)
				{
					float directionErr = (currentAngle - (targetAngle));
					if (directionErr > 180)
					{
						directionErr -= 360;
					}
					else if (directionErr < -180)
					{
						directionErr += 360;
					}
					// float dirOut = globalCpp.pid_Direction.calcPid(err_Y < 0 ? err_X : -err_X); //
					float dirOut = globalCpp.pid_Direction.calcPid(directionErr);//dirOut = 0;//
					turnToRedirectionMode();
					yVelocity = errY_WhenCorrected < 0 ? -200 : 200;
					if(blockDir!=0){
						yVelocity = blockDir < 0 ? -200 : 200;
					}
					xVelocity = errX_WhenAvoidTrigged > 0 ? 1800 : -1800;
					if((obstacleAvoidCntValue-obstacleAvoidCnt)>30){
						yVelocity=0;
					}else if((obstacleAvoidCntValue-obstacleAvoidCnt)<10){
						xVelocity=0;

						yVelocity*=2.4;
					}
					pa_updateMotorPwm(1,
									  (yVelocity - xVelocity+dirOut));
					pa_updateMotorPwm(2,
									  (yVelocity + xVelocity-dirOut));
					//float out = -globalCpp.pid_Motor3.calcPid(speedOfMotors.speedOfM3 - dirOut);
					pa_updateMotorPwm(3,
									  (yVelocity - xVelocity-dirOut));
					// pa_updateMotorPwm(2,300);
					pa_updateMotorPwm(4,
									  (yVelocity + xVelocity+dirOut));
					//UART_PutStr(UART2, "obstacling Avoidance Constantly!!\r\n");
					UART_PutStr(UART2, "obstacling Avoidance Constantly ");
					UART_PutStr(UART2, &blockIndex);
					UART_PutStr(UART2, " !!\r\n");
					//后面是必须的操作//////////////////////////////////////////////////////////////////////
					obstacleAvoidCnt--;
				}
				else if (directionCorrected)
				{
					float errX_fix=0;
					float directionErr = (currentAngle - (targetAngle-errX_fix*3));
					if (directionErr > 180)
					{
						directionErr -= 360;
					}
					else if (directionErr < -180)
					{
						directionErr += 360;
					}
					// float dirOut = globalCpp.pid_Direction.calcPid(err_Y < 0 ? err_X : -err_X); //
					float dirOut = globalCpp.pid_Direction.calcPid(directionErr);
					yVelocity = errY_WhenCorrected < 0 ? 1600 : -1600;
					pa_updateMotorPwm(1,
									  (yVelocity + dirOut));
					pa_updateMotorPwm(2,
									  (yVelocity - dirOut));
					//float out = -globalCpp.pid_Motor3.calcPid(speedOfMotors.speedOfM3 - dirOut);
					pa_updateMotorPwm(3,
									  (yVelocity - dirOut));
					// pa_updateMotorPwm(2,300);
					pa_updateMotorPwm(4,
									  (yVelocity + dirOut));
					// //速度环///////////////////////////////////////////
					// pa_updateMotorPwm(1,
					// 				  -globalCpp.pid_Motor1.calcPid(speedOfMotors.speedOfM1 - (yVelocity)));
					// pa_updateMotorPwm(2,
					// 				  -globalCpp.pid_Motor2.calcPid(speedOfMotors.speedOfM2 - (yVelocity)));
					// //float out = -globalCpp.pid_Motor3.calcPid(speedOfMotors.speedOfM3 - dirOut);
					// pa_updateMotorPwm(3,
					// 				  -globalCpp.pid_Motor3.calcPid(speedOfMotors.speedOfM3 - (yVelocity)));
					// // pa_updateMotorPwm(2,300);
					// pa_updateMotorPwm(4,
					// 				  -globalCpp.pid_Motor4.calcPid(speedOfMotors.speedOfM4 - (yVelocity)));

					{
						char buf[30] = {0};
						sprintf(buf, "Walking!! %d\r\n",
								pathLength);
						// sprintf(buf, "%f %5d %2d %2d\r\n",
						// 		dirOut, globalCpp.targetSpeed,err_X,err_Y);

						UART_PutStr(UART2, buf);
					}
				}
				else
				{ //先校准方向
					float dirOut = globalCpp.pid_DirectionCalibration.calcPid(err_Y < 0 ? err_X : -err_X);
					yVelocity*=0.4;
					if(!gameStartFlag){
						yVelocity=0;
					}
					pa_updateMotorPwm(1,
									  (dirOut+yVelocity));
					pa_updateMotorPwm(2,
									  (-dirOut+yVelocity));
					//float out = -globalCpp.pid_Motor3.calcPid(speedOfMotors.speedOfM3 - dirOut);
					pa_updateMotorPwm(3,
									  (-dirOut+yVelocity));
					// pa_updateMotorPwm(2,300);
					pa_updateMotorPwm(4,
									  (dirOut+yVelocity));
					
					{
						char buf[30] = {0};
						sprintf(buf, "Redirection!! %d\r\n",
								err_Y < 0 ? err_X : -err_X);
						// sprintf(buf, "%f %5d %2d %2d\r\n",
						// 		dirOut, globalCpp.targetSpeed,err_X,err_Y);

						// UART_PutStr(UART2, buf);
					}
					if (fabs(err_X)<2.5&&fabs(err_X)>0.5)
					{
						targetAngle = currentAngle;
						directionCorrected = true;
						short err_Y1 = voiceDataProcessor.getErrY();
						short err_Y2 = voiceDataProcessor.getErrY();
						short err_Y3 = voiceDataProcessor.getErrY();

						errY_WhenCorrected = (err_Y + err_Y1 + err_Y2+err_Y3) / 3;
					}
				}
				if(gameStartFlag>0){
					gameStartFlag--;
				}
				// if (//ultrasonicDistance1.distance < 10||//ultrasonicDistance2.distance<10)
				// {
				// 	err_Y_whenBlocked = err_Y;
				// 	xMoveDelay = 100;
				// 	yVelocity = 0;
				// 	xVelocity = err_X > 0 ? 1000 : -1000;
				// 	dirOut = 0;
				// }
				// else if (xMoveDelay > 0 && err_Y != err_Y_whenBlocked)
				// {
				// 	xMoveDelay--;
				// 	yVelocity = 0;
				// 	xVelocity = err_X > 0 ? 1000 : -1000;
				// 	dirOut = 0;
				// }
				// else if (err_Y_whenBlocked == err_Y)
				// {
				// 	xMoveDelay = 0;
				// }
//float dirOut=pid_Direction.calcPid(err_Y>0?err_X:-err_X);
// dirOut=300;

// pa_updateMotorPwm(1, -globalCpp.pid_Motor1.calcPid(speedOfMotors.speedOfM1 + dirOut));
// pa_updateMotorPwm(2, -globalCpp.pid_Motor2.calcPid(speedOfMotors.speedOfM2 - dirOut));
// float out = -globalCpp.pid_Motor3.calcPid(speedOfMotors.speedOfM3 - dirOut);
// pa_updateMotorPwm(3, out);
// // pa_updateMotorPwm(2,300);
// pa_updateMotorPwm(4, -globalCpp.pid_Motor4.calcPid(speedOfMotors.speedOfM4 + dirOut));
//#define speedPid
#ifdef speedPid
				xVelocity = 0;
				pa_updateMotorPwm(1,
								  -globalCpp.pid_Motor1.calcPid(speedOfMotors.speedOfM1 - (dirOut + yVelocity - xVelocity)));
				pa_updateMotorPwm(2,
								  -globalCpp.pid_Motor2.calcPid(speedOfMotors.speedOfM2 - (-dirOut + yVelocity + xVelocity)));
				//float out = -globalCpp.pid_Motor3.calcPid(speedOfMotors.speedOfM3 - dirOut);
				pa_updateMotorPwm(3,
								  -globalCpp.pid_Motor3.calcPid(speedOfMotors.speedOfM3 - (-dirOut + yVelocity - xVelocity)));
				// pa_updateMotorPwm(2,300);
				pa_updateMotorPwm(4,
								  -globalCpp.pid_Motor4.calcPid(speedOfMotors.speedOfM4 - (dirOut + yVelocity + xVelocity)));
#endif
				// #ifndef speedPid
				// 				//yVelocity = 0;

				// 				pa_updateMotorPwm(1,
				// 								  (dirOut + yVelocity - xVelocity));
				// 				pa_updateMotorPwm(2,
				// 								  (-dirOut + yVelocity + xVelocity));
				// 				//float out = -globalCpp.pid_Motor3.calcPid(speedOfMotors.speedOfM3 - dirOut);
				// 				pa_updateMotorPwm(3,
				// 								  (-dirOut + yVelocity - xVelocity));
				// 				// pa_updateMotorPwm(2,300);
				// 				pa_updateMotorPwm(4,
				// 								  (dirOut + yVelocity + xVelocity));
				// #endif
				// 				{
				// 					char buf[30] = {0};
				// 					sprintf(buf, "%f\r\n",
				// 							dirOut);
				// 					// sprintf(buf, "%f %5d %2d %2d\r\n",
				// 					// 		dirOut, globalCpp.targetSpeed,err_X,err_Y);

				// 					UART_PutStr(UART2, buf);
				// 				}
			}

			checkUartData();
		}

		if (flag_100ms)
		{
			if(PIN_Read(P00_2)){
				// UART_PutStr(UART2, "blocked!!!");
			}else{
				// UART_PutStr(UART2, "no block");
			}
			
			flag_100ms = 0;
			ultrasonicDistance1.doBeforeTrig();
			ultrasonicDistance2.doBeforeTrig();
			ultrasonicDistance3.doBeforeTrig();
			ultrasonicDistance4.doBeforeTrig();
			//触发超声波
			pa_UltrasonicDistance::trig();

			// UART_PutStr(UART2, "trig\r\n");
			//互相关计算
			err_X = voiceDataProcessor.getErrX(); //crossCorrelation_noFFT(adc_arr1, adc_arr2);
			err_Y = voiceDataProcessor.getErrY(); //crossCorrelation_noFFT(adc_arr3, adc_arr4);
			for (int i = 0; i < 9; i++)
			{
				err_Y_History[i] = err_Y_History[i + 1];
			}
			err_Y_History[10] = err_Y;
			// err_Fm= voiceDataProcessor.getErrFm();
			////计算fft并且检查信标打开,并计算距离
			voiceDataProcessor.checkBeaconOn();
			// UART_PutStr(UART2, "before BNO055_getVector");
			// checkBeaconOn();
			bno055_vector_t v = pa_BNO055_getVector();
			// UART_PutStr(UART2, "after BNO055_getVector");
			if (v.x >= 0 || v.x < 360)
			{
				currentAngle = v.x;
				// 	sprintf(txt,"x:%.2f,y:%.2f,z:%.2f\r\n",v.x,v.y,v.z);
				// UART_PutStr(UART2, txt);
			}
		}

		// checkSignalStable();
		//串口输出内容的选择
		uartOutPutSelect();
	}
}
void turnToRedirectionMode()
{
	directionCorrected = false;
	globalCpp.pid_DirectionCalibration.iSum = 0;
}
void uartOutPutSelect()
{
	switch (globalCpp.micOutPutMode)
	{
	case OutputMode_cross:
	{
		char buf[30] = {0};
		sprintf(buf, "%5d %5d %5d\r\n",
				err_X,
				err_Y,
				// err_Fm
				voiceDataProcessor.distance);

		UART_PutStr(UART2, buf);
	}
	break;
	case OutputMode_Mic:
	{
		//adc值实时***************************************
		{
			char buf[36] = {0};
			sprintf(buf, "%5d %5d %5d %5d %5d\r\n",
					adcValue1,
					adcValue2,
					adcValue3,
					adcValue4,
					adcValue_fm);

			UART_PutStr(UART2, buf);
		}
		//***************************************************
	}
	break;
	case OutputMode_UltrasonicDistance:
	{
		//adc值实时***************************************
		{
			char buf[40] = {0};
			sprintf(buf, "%3d %3d %3d %3d %f\r\n",
					ultrasonicDistance1.distance,
					//ultrasonicDistance1.countingDistance,
					ultrasonicDistance2.distance,
					ultrasonicDistance3.distance,
					ultrasonicDistance4.distance,
					currentAngle);
			UART_PutStr(UART2, buf);
		}
		//***************************************************
		break;
	}
	case OutputMode_motor: //编码器速度
	{
		{
			char buf[30] = {0};
			sprintf(buf, "%5d %5d %5d %5d %d\r\n",
					speedOfMotors.speedOfM1,
					speedOfMotors.speedOfM2,
					speedOfMotors.speedOfM3,
					speedOfMotors.speedOfM4,
					pathLength);
			// sprintf(buf, "%f %5d %2d %2d\r\n",
			// 		dirOut, globalCpp.targetSpeed,err_X,err_Y);

			UART_PutStr(UART2, buf);
		}
		break;
	}
	}
}

// void addValueToArr(unsigned short value1, unsigned short value2, unsigned short value3, unsigned short value4,unsigned short value5)
// {
// 	if (endBuffer)
// 	{
// 		return;
// 	}
// 	// {
// 	// 	char buffer[30] = "";
// 	// 	sprintf(buffer, "%d\r\n",value1);
// 	// 	UART_PutStr(UART2, buffer);
// 	// }

// 	adc_arr1[step] = value1;
// 	adc_arr2[step] = value2;
// 	adc_arr3[step] = value3;
// 	adc_arr4[step] = value4;

// 	// adc_arr1[step] = value1==0?1:value1;
// 	// adc_arr2[step] = value2==0?1:value2;

// 	step += 1;
// 	if (step == adc_arrlen)
// 	{
// 		step = 0;
// 		endBuffer = 1;
// 	}
// 	//step %= adc_arrlen;
// }

void getadc()
{
	adcValue1 = ADC_Read(ADC0) / 20;
	adcValue2 = ADC_Read(ADC4) / 20;

	adcValue3 = ADC_Read(ADC2) / 20;
	adcValue4 =	ADC_Read(ADC7) / 20;

	//adcValue_fm = ADC_Read(ADC7) * 0.08;

	// adcValue_fm = fmAdc<0?0:fmAdc;
	voiceDataProcessor.addAdcValueToArr(adcValue1, adcValue2, adcValue3, adcValue4, adcValue_fm);
	//addValueToArr(adcValue1, adcValue2, adcValue3, adcValue4);

	//paLinearInterpolation(adc1_val,adc2_val,3);
	// adc_arr1[step] = adc1_val;
	// adc_arr2[step] = adc2_val;
	// {
	// 	char buffer[30];
	// 	sprintf(buffer,"%d %d\r\n",adc_arr1[step],adc_arr2[step]);
	// 	UART_PutStr(UART2,buffer);
	// }
	// step+=1;
	// step %= adc_arrlen;
}
// unsigned short adc_arr1_copy[adc_arrlen];
// unsigned short adc_arr2_copy[adc_arrlen];
// bool lockRead = false;
// char crossCorrelation_noFFT(unsigned short arr1[], unsigned short arr2[])
// {
// 	long max = 0, maxpos = 0;
// #define detectRange 10
// 	int i = -detectRange;
// 	if (endBuffer)
// 	{

// 		// 发送收集到的adc*************************************************
// 		// for (int a = 0; a < adc_arrlen; a++)
// 		// {
// 		// 	{
// 		// 		char buffer[30] = "";
// 		// 		sprintf(buffer, "%d %d\r\n", adc_arr1[a], adc_arr2[a]);
// 		// 		UART_PutStr(UART2, buffer);
// 		// 	}
// 		// }
// 		// UART_PutStr(UART2, "done\r\n");
// 		// **********************************************
// 		endBuffer = 0;
// 	}

// 	for (; i < detectRange; i++)
// 	{
// 		long sum = 0;
// 		for (int t = 0; t < adc_arrlen; t++)
// 		{
// 			int first_pos = t;
// 			int second_pos = t + i;
// 			if (second_pos < 0)
// 			{
// 				second_pos += adc_arrlen;
// 			}
// 			else if (second_pos >= adc_arrlen)
// 			{
// 				second_pos -= adc_arrlen;
// 			}
// 			// if (t + i < 0 || t + i > adc_arrlen - 1)
// 			// 	continue;
// 			sum += (long)arr1[first_pos] * arr2[second_pos];
// 		}
// 		if (sum > max)
// 		{
// 			max = sum;
// 			maxpos = i;
// 		}
// 		if (globalCpp.micOutPutMode == OutputMode_crossDetail)
// 		{
// 			char buffer[30] = "";
// 			sprintf(buffer, "%d\r\n", sum);
// 			UART_PutStr(UART2, buffer);
// 		}
// 	}
// 	if (globalCpp.micOutPutMode == OutputMode_crossDetail)
// 	{
// 		UART_PutStr(UART2, "0\r\n");
// 	}
// 	// UART_PutStr(UART2, "0\r\n");
// 	// {
// 	// 	char buffer[30] = "";
// 	// 	sprintf(buffer, "%d\r\n", maxpos);
// 	// 	UART_PutStr(UART2, buffer);
// 	// }
// 	return maxpos;
// }
