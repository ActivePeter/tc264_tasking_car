
extern "C"
{
#include "pa_MainApp.h"
#include "stdio.h"
#include <LQ_UART.h>
#include <LQ_ADC.h>
#include <LQ_STM.h>
#include <LQ_GTM.h>
#include "string.h"
#include "math.h"
#include "pa_CommonLib/pa_BNO055.h"
}

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
					
void startMainTask()
{
	UART_InitConfig(UART2_RX_P14_3, UART2_TX_P14_2, 115200);//2红色线
	{
		UART_PutStr(UART2, "init\r\n");
	}
	pa_BNO055_init();
	ADC_InitConfig(ADC0, 100000); //初始化
	ADC_InitConfig(ADC1, 100000);
	ADC_InitConfig(ADC2, 100000);
	ADC_InitConfig(ADC3, 100000);
	
	STM_InitConfig(STM0, STM_Channel_0, 100, []() { //微秒  //UART_PutStr(UART2,"ssss");
		getadc();											  //getadc();
	});

	//右上
	TOM_PWM_InitConfig(PWM2, 1000, 10000);//初始化P33_13 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
	TOM_PWM_InitConfig(PWM3, 0, 10000);//初始化P33_7 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
	

	TOM_PWM_InitConfig(PWM1, 1000, 10000);//初始化P33_10 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
	TOM_PWM_InitConfig(PWM4, 0, 10000);//初始化P33_6 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100

	TOM_PWM_InitConfig(PWM5, 1000, 10000);//初始化P33_10 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
	TOM_PWM_InitConfig(PWM6, 0, 10000);//初始化P33_13 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100

	TOM_PWM_InitConfig(PWM7, 1000, 10000);//初始化P33_7 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
	TOM_PWM_InitConfig(PWM8, 0, 10000);//初始化P33_6 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
	while (1)
	{
		crossCorrelation_noFFT();
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
