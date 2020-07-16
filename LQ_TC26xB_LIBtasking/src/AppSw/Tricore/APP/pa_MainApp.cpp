
extern "C"
{
#include "pa_MainApp.h"
#include "stdio.h"
#include <LQ_UART.h>
#include <LQ_ADC.h>
#include <LQ_STM.h>
#include <LQ_GTM.h>
#include <LQ_GPT12_ENC.h>
#include "string.h"
#include "math.h"
#include "pa_CommonLib/pa_BNO055.h"
}

#include "pa_CommonLib/pa_MecanumModel.h"

#define adc_arrlen 2000
unsigned short adc_arr1[adc_arrlen];
unsigned short adc_arr2[adc_arrlen];

unsigned short step = 0;

#define PWM1		IfxGtm_TOM0_0_TOUT32_P33_10_OUT
#define PWM2		IfxGtm_TOM0_13_TOUT35_P33_13_OUT
#define PWM3		IfxGtm_TOM0_3_TOUT29_P33_7_OUT
#define PWM4		IfxGtm_TOM1_2_TOUT28_P33_6_OUT
#define PWM5		IfxGtm_TOM0_1_TOUT27_P33_5_OUT
#define PWM6		IfxGtm_TOM1_4_TOUT30_P33_8_OUT
#define PWM7		IfxGtm_TOM1_1_TOUT31_P33_9_OUT
#define PWM8		IfxGtm_TOM0_2_TOUT33_P33_11_OUT

int cnt=0;
char flag_50ms=1;
SpeedOfMotors speedOfMotors;

pa_MecanumModel mecanumModel();

void initVariable(){//初始化变量
	speedOfMotors.speedOfM1=0;
	speedOfMotors.speedOfM2=0;
	speedOfMotors.speedOfM3=0;
	speedOfMotors.speedOfM4=0;
}
void initFuncs(){//初始化功能
	UART_InitConfig(UART2_RX_P14_3, UART2_TX_P14_2, 115200);//2红色线
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
		getadc();											  //getadc();
		cnt++;
		if(cnt==500){
			cnt=0;
			flag_50ms=1;
		}
	});

	// //右上
	// TOM_PWM_InitConfig(PWM2, 1000, 10000);//初始化P33_13 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
	// TOM_PWM_InitConfig(PWM3, 0, 10000);//初始化P33_7 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
	

	// TOM_PWM_InitConfig(PWM1, 1000, 10000);//初始化P33_10 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
	// TOM_PWM_InitConfig(PWM4, 0, 10000);//初始化P33_6 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100

	// TOM_PWM_InitConfig(PWM5, 1000, 10000);//初始化P33_10 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
	// TOM_PWM_InitConfig(PWM6, 0, 10000);//初始化P33_13 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100

	// TOM_PWM_InitConfig(PWM7, 1000, 10000);//初始化P33_7 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
	// TOM_PWM_InitConfig(PWM8, 0, 10000);//初始化P33_6 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100

	ENC_InitConfig(ENC2_InPut_P33_7, ENC2_Dir_P33_6);
  //ENC_InitConfig(ENC3_InPut_P02_6, ENC3_Dir_P02_7);//摄像头冲突，不建议用
	ENC_InitConfig(ENC4_InPut_P02_8, ENC4_Dir_P33_5);
	ENC_InitConfig(ENC5_InPut_P10_3, ENC5_Dir_P10_1);
	ENC_InitConfig(ENC6_InPut_P20_3, ENC6_Dir_P20_0);


}

void startMainTask()
{
	initVariable();
	initFuncs();
	while (1)
	{
		//执行控制算法
		if(flag_50ms==1){
			flag_50ms=0;
			speedOfMotors.speedOfM1 = ENC_GetCounter(ENC2_InPut_P33_7);
			speedOfMotors.speedOfM2 = ENC_GetCounter(ENC4_InPut_P02_8);
			speedOfMotors.speedOfM3 = ENC_GetCounter(ENC5_InPut_P10_3);
			speedOfMotors.speedOfM4 = ENC_GetCounter(ENC6_InPut_P20_3);

			{
				char buf[30] = {0};
				sprintf(buf,"%5d %5d %5d %5d\r\n",
				speedOfMotors.speedOfM1,
				speedOfMotors.speedOfM2,
				speedOfMotors.speedOfM3,
				speedOfMotors.speedOfM4);
				UART_PutStr(UART2, buf);
			}
			
		}
		//crossCorrelation_noFFT();
	}
}
void paLinearInterpolation(unsigned short to1,unsigned short to2,unsigned short count){
	
	int from1=adc_arr1[step];
	int from2=adc_arr2[step];

	int deltaValue1=to1-from1;
	int deltaValue2=to2-from2;
	for(unsigned short i=0;i<count;i++){
		addValueToArr(
			(unsigned short)(from1+(deltaValue1*(i+1))/count),
			(unsigned short)(from2+(deltaValue2*(i+1))/count)
		);
	}

}
void addValueToArr(unsigned short value1,unsigned short value2){
	
	// {
	// 	char buffer[30] = "";
	// 	sprintf(buffer, "%d\r\n",value1);
	// 	UART_PutStr(UART2, buffer);
	// }
	adc_arr1[step]=value1;
	adc_arr2[step]=value2;

	step+=1;
	step %= adc_arrlen;
}
void getadc()
{
	unsigned short adc1_val=ADC_Read(ADC1) / 20;
	unsigned short adc2_val=ADC_Read(ADC2) / 20;
	 addValueToArr(adc1_val,adc2_val);
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
bool lockRead =false;
void crossCorrelation_noFFT()
{
	long max = 0, maxpos = 0;
	#define detectRange 20
	int i = -detectRange;
	
	memcpy(adc_arr1_copy,adc_arr1,adc_arrlen);
	memcpy(adc_arr2_copy,adc_arr2,adc_arrlen);

	for (; i < detectRange; i++)
	{
		long sum = 0;
		for (int t = 0; t < adc_arrlen; t++)
		{
			int first_pos=t;
			int second_pos=t+i;
			if(second_pos<0){
				second_pos+=adc_arrlen;
			}else if(second_pos>=adc_arrlen){
				second_pos-=adc_arrlen;
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
		sprintf(buffer, "%d\r\n",  maxpos);
		UART_PutStr(UART2, buffer);
	}

	//printf("%d\r\n", maxpos);
}
