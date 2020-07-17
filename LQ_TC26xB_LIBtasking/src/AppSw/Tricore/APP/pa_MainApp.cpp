
extern "C"
{

#include "pa_MainApp.h"
#include "stdio.h"
#include <LQ_UART.h>
#include <LQ_ADC.h>
#include <LQ_STM.h>

#include <LQ_GPT12_ENC.h>
#include "string.h"
#include "math.h"
#include "pa_CommonLib/pa_BNO055.h"
#include "pa_CommonLib/pa_MotorManager.h"
#include <LQ_Atom_Motor.h>
#include "math.h"
}

#include "pa_CommonLib/pa_PID.h"
#include "pa_CommonLib/pa_MecanumModel.h"

#define adc_arrlen 2000
unsigned short adc_arr1[adc_arrlen];
unsigned short adc_arr2[adc_arrlen];

unsigned short step = 0;

int cnt = 0;
char flag_50ms = 1;
SpeedOfMotors speedOfMotors;

pa_MecanumModel mecanumModel;
pa_PID pid_Motor1 = pa_PID(1, 0, 0);
pa_PID pid_Motor2 = pa_PID(1, 0, 0);
pa_PID pid_Motor3 = pa_PID(1, 0, 0);
pa_PID pid_Motor4 = pa_PID(1, 0, 0);

void initVariable()
{ //初始化变量
	speedOfMotors.speedOfM1 = 0;
	speedOfMotors.speedOfM2 = 0;
	speedOfMotors.speedOfM3 = 0;
	speedOfMotors.speedOfM4 = 0;
	pid_Motor1.setPid(0.001, 0, 0);
	pid_Motor2.setPid(0.001, 0, 0);
	pid_Motor3.setPid(0.001, 0, 0);
	pid_Motor4.setPid(0.001, 0, 0);
}
void initFuncs()
{
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
	ADC_InitConfig(ADC3, 100000);

	//adc采集以及控制 定时器中断
	STM_InitConfig(STM0, STM_Channel_0, 100, []() { //微秒  //UART_PutStr(UART2,"ssss");
		getadc();									//getadc();
		cnt++;
		if (cnt % 100 == 0)
		{
			speedOfMotors.speedOfM1 = ENC_GetCounter(ENC3_InPut_P02_6);
			speedOfMotors.speedOfM2 = -ENC_GetCounter(ENC2_InPut_P33_7);
			speedOfMotors.speedOfM3 = -ENC_GetCounter(ENC6_InPut_P20_3);
			speedOfMotors.speedOfM4 = ENC_GetCounter(ENC5_InPut_P10_3);
		}
		if (cnt == 500)
		{
			cnt = 0;
			flag_50ms = 1;
		}
	});

	pa_initMotorPwm();

	ENC_InitConfig(ENC2_InPut_P33_7, ENC2_Dir_P33_6);
	//ENC_InitConfig(ENC3_InPut_P02_6, ENC3_Dir_P02_7);//摄像头冲突，不建议用
	ENC_InitConfig(ENC3_InPut_P02_6, ENC3_Dir_P02_7);
	ENC_InitConfig(ENC5_InPut_P10_3, ENC5_Dir_P10_1);
	ENC_InitConfig(ENC6_InPut_P20_3, ENC6_Dir_P20_0);
}

int tagetSpeed = 100;
void startMainTask()
{
	initVariable();
	initFuncs();

	while (1)
	{
		//执行控制算法
		if (flag_50ms == 1)
		{
			flag_50ms = 0; //50ms清0

			// pa_updateMotorPwm(1,-pid_Motor1.calcPid(speedOfMotors.speedOfM1-tagetSpeed));
			// pa_updateMotorPwm(2,-pid_Motor2.calcPid(speedOfMotors.speedOfM2-tagetSpeed));
			float out = -pid_Motor3.calcPid(speedOfMotors.speedOfM3 - tagetSpeed);
			pa_updateMotorPwm(3, out);
			// pa_updateMotorPwm(2,300);
			// pa_updateMotorPwm(4,-pid_Motor4.calcPid(speedOfMotors.speedOfM4-tagetSpeed));

			checkUartData();
			{
				char buf[30] = {0};
				// sprintf(buf,"%5d %5d %5d %5d\r\n",
				// speedOfMotors.speedOfM1,
				// speedOfMotors.speedOfM2,
				// speedOfMotors.speedOfM3,
				// speedOfMotors.speedOfM4);
				sprintf(buf, "%5d %f\r\n",
						speedOfMotors.speedOfM3, out);

				UART_PutStr(UART2, buf);
			}
		}
		//crossCorrelation_noFFT();
	}
}
void checkUartData()
{
	if (uartHasReceivedData())
	{
		unsigned char recData[30] = {0};
		UART_GetBuff(UART2, recData, UART_GetCount(UART2));

		int index = 2;
		char state1 = 0, state2 = 0;
		char dataValid = 0;
		char xiaoshu = 0;
		float data = 0;
		state1 = recData[0];
		state2 = recData[1];
		while (recData[index])
		{
			if (state1 == 's' && (state2 == 'p' || state2 == 'i' || state2 == 'd'||state2=='s'))
			{
				if (recData[index] <= '9' && recData[index] >= '0')
				{
					if (!xiaoshu)
					{
						data = recData[index] - 48 + data * 10;
					}
					else
					{
						data = data + pow(0.1, xiaoshu) * (recData[index] - 48);
						xiaoshu++;
					}
				}
				else if (recData[index] == '.')
				{
					xiaoshu = 1;
				}
				else if (recData[index] == '!')
				{
					dataValid = 1;
					break;
				}
			}
			else
			{
				break;
			}
			index++;
		}
		if (dataValid)
		{
			if (state1 == 's')
			{
				switch (state2)
				{
				case 'p':
				{
					char buffer[30] = "";
					sprintf(buffer, "set p from %f to %f\r\n", pid_Motor1.kp, data);
					UART_PutStr(UART2, buffer);
				}
					pid_Motor1.kp = data;
					pid_Motor2.kp = data;
					pid_Motor3.kp = data;
					pid_Motor4.kp = data;
					break;
				// case 'i':
				// 	{
				// 		char buffer[30] = "";
				// 		sprintf(buffer, "set i from %f to %f\r\n",pid_Motor1.ki, data);
				// 		UART_PutStr(UART2, buffer);
				// 	}
				// 	pid_Motor1.ki=data;
				// 	pid_Motor2.ki=data;
				// 	pid_Motor3.ki=data;
				// 	pid_Motor4.ki=data;
				// 	break;
				case 'd':
				{
					char buffer[30] = "";
					sprintf(buffer, "set d from %f to %f\r\n", pid_Motor1.kd, data);
					UART_PutStr(UART2, buffer);
				}
					pid_Motor1.kd = data;
					pid_Motor2.kd = data;
					pid_Motor3.kd = data;
					pid_Motor4.kd = data;
					/* code */
					break;
				case 's':
				{
					char buffer[30] = "";
					sprintf(buffer, "set s from %f to %f\r\n", tagetSpeed, data);
					UART_PutStr(UART2, buffer);
				}
					tagetSpeed=data;
					/* code */
					break;
				default:
					break;
				}
			}
		}
		else
		{
			UART_PutStr(UART2, "invalidData:");
			UART_PutStr(UART2, (char *)recData);
			UART_PutStr(UART2, "\r\n");
		}

		changeLED();
	}
}
void paLinearInterpolation(unsigned short to1, unsigned short to2, unsigned short count)
{

	int from1 = adc_arr1[step];
	int from2 = adc_arr2[step];

	int deltaValue1 = to1 - from1;
	int deltaValue2 = to2 - from2;
	for (unsigned short i = 0; i < count; i++)
	{
		addValueToArr(
			(unsigned short)(from1 + (deltaValue1 * (i + 1)) / count),
			(unsigned short)(from2 + (deltaValue2 * (i + 1)) / count));
	}
}
void addValueToArr(unsigned short value1, unsigned short value2)
{

	// {
	// 	char buffer[30] = "";
	// 	sprintf(buffer, "%d\r\n",value1);
	// 	UART_PutStr(UART2, buffer);
	// }
	adc_arr1[step] = value1;
	adc_arr2[step] = value2;

	step += 1;
	step %= adc_arrlen;
}
void getadc()
{
	unsigned short adc1_val = ADC_Read(ADC1) / 20;
	unsigned short adc2_val = ADC_Read(ADC2) / 20;
	addValueToArr(adc1_val, adc2_val);
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
unsigned short adc_arr1_copy[adc_arrlen];
unsigned short adc_arr2_copy[adc_arrlen];
bool lockRead = false;
void crossCorrelation_noFFT()
{
	long max = 0, maxpos = 0;
#define detectRange 20
	int i = -detectRange;

	memcpy(adc_arr1_copy, adc_arr1, adc_arrlen);
	memcpy(adc_arr2_copy, adc_arr2, adc_arrlen);

	for (; i < detectRange; i++)
	{
		long sum = 0;
		for (int t = 0; t < adc_arrlen; t++)
		{
			int first_pos = t;
			int second_pos = t + i;
			if (second_pos < 0)
			{
				second_pos += adc_arrlen;
			}
			else if (second_pos >= adc_arrlen)
			{
				second_pos -= adc_arrlen;
			}
			// if (t + i < 0 || t + i > adc_arrlen - 1)
			// 	continue;
			sum += (long)adc_arr1_copy[first_pos] * adc_arr2_copy[second_pos];
		}
		if (sum > max)
		{
			max = sum;
			maxpos = i;
		}
		// {
		// 	char buffer[30] = "";
		// 	sprintf(buffer, "%d\r\n", sum);
		// 	UART_PutStr(UART2, buffer);
		// }
	}
	// UART_PutStr(UART2, "0\r\n");
	{
		char buffer[30] = "";
		sprintf(buffer, "%d\r\n", maxpos);
		UART_PutStr(UART2, buffer);
	}

	//printf("%d\r\n", maxpos);
}
