#include "pa_VoiceDataProcessor.h"
extern "C"
{
#include <Ifx_FftF32.h>
#include "pa_CommonLib/pa_app/pa_OLED/pa_oled.h"
#include "stdio.h"
#include "LQ_UART.h"
#include "pa_CommonLib/pa_drv/pa_Common.h"
}

pa_VoiceDataProcessor::pa_VoiceDataProcessor() {}

char pa_VoiceDataProcessor::getErrX()
{
    adcLocked=true;
    return crossCorrelation_noFFT(adc_arr1, adc_arr2);
}
char pa_VoiceDataProcessor::getErrY()
{
    return crossCorrelation_noFFT(adc_arr3, adc_arr4);
}
short pa_VoiceDataProcessor::getErrFm()
{
    return crossCorrelation_noFFT(adc_arr1, adc_arrFm);
}
char pa_VoiceDataProcessor::isBeaconOn()
{
    return rightFftCount >= 30;
}
short pa_VoiceDataProcessor::crossCorrelation_noFFT(unsigned short arr1[], unsigned short arr2[])
{

    long max = 0;
    short maxpos = 0;
#define detectRange 10
    int i = -detectRange;
    // i=0;
    // if (endBuffer)
    // {

    // 	// 发送收集到的adc*************************************************
    // 	// for (int a = 0; a < adc_arrlen; a++)
    // 	// {
    // 	// 	{
    // 	// 		char buffer[30] = "";
    // 	// 		sprintf(buffer, "%d %d\r\n", adc_arr1[a], adc_arr2[a]);
    // 	// 		UART_PutStr(UART2, buffer);
    // 	// 	}
    // 	// }
    // 	// UART_PutStr(UART2, "done\r\n");
    // 	// **********************************************
    // 	endBuffer = 0;
    // }

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
            sum += (long)arr1[first_pos] * arr2[second_pos];
        }
        if (sum > max)
        {
            max = sum;
            maxpos = i;
        }
        // if (globalCpp.micOutPutMode == OutputMode_crossDetail)
        // {
        // 	char buffer[30] = "";
        // 	sprintf(buffer, "%d\r\n", sum);
        // 	UART_PutStr(UART2, buffer);
        // }
    }
    // if (globalCpp.micOutPutMode == OutputMode_crossDetail)
    // for (int i = 0; i < 300; i++){
    // 	UART_PutStr(UART2, "0\r\n");
    // }

    // UART_PutStr(UART2, "0\r\n");
    // {
    // 	char buffer[30] = "";
    // 	sprintf(buffer, "%d\r\n", maxpos);
    // 	UART_PutStr(UART2, buffer);
    // }
    return maxpos;
}

void pa_VoiceDataProcessor::addAdcValueToArr(unsigned short value1, unsigned short value2, unsigned short value3, unsigned short value4, unsigned short value5)
{
    // if(adcLocked)return;
    adc_arr1[step] = value1;
    adc_arr2[step] = value2;
    adc_arr3[step] = value3;
    adc_arr4[step] = value4;
    adc_arrFm[step] =value5;
    // adc_arr1[step] = value1==0?1:value1;
    // adc_arr2[step] = value2==0?1:value2;

    step += 1;
    if (step == adc_arrlen)
    {
        step = 0;
        // endBuffer = 1;
    }
}

void pa_VoiceDataProcessor::checkBeaconOn()
{
    {
        //FFT变换结果，必须对齐256位，整数倍
        cfloat32 fftIn[adc_arrlen];
        cfloat32 fftOutMic[adc_arrlen];
        cfloat32 fftOutFm[adc_arrlen];
        
        for (int i = 0; i < adc_arrlen; i++)
        {
            fftIn[i].imag = 0;
            fftIn[i].real = ((float)adc_arr1[i]+adc_arr2[i])/2;
        }
        /* 初始化为 FFT  注意 这里FFT输出结果是实际结果的64倍  */
        Ifx_FftF32_radix2(fftOutMic, fftIn, adc_arrlen);

        for (int i = 0; i < adc_arrlen; i++)
        {
            fftIn[i].imag = 0;
            fftIn[i].real = adc_arrFm[i];
        }
        adcLocked=false;
        Ifx_FftF32_radix2(fftOutFm, fftIn, adc_arrlen);

        // for(int i=60;i<300;i++){
        // 	if(fabs(fftOutMic[i].real)>1000)
        // 	{
        // 		count++;
        // 	}
        // 	{
        // 		char buf[30] = {0};
        // 		sprintf(buf, "%f\r\n",
        // 				fabs(fftOutMic[i].real));

        // 		UART_PutStr(UART2, buf);
        // 	}
        // }
        short count = 0;
        for (int i = 101; i < 300; i++)
        {
            if (fabs(fftOutMic[i + 62].real) > 1000)
            {
                count++;
            }
        }
        for (int i = 82; i < 101; i++)
        {
            if (fabs(fftOutMic[i + 62].real) > -52 * i + 6316)
            {
                count++;
            }
        }
        for (int i = 0; i < 82; i++)
        {
            if (fabs(fftOutMic[i + 62].real) > 24 * i)
            {
                count++;
            }
        }
        for (int i = 0; i < adc_arrlen; i++) {
		fftIn[i].real = (fftOutMic[i].real*fftOutFm[i].real + fftOutMic[i].imag*fftOutFm[i].imag)/2048;//不除2048就溢出了
		fftIn[i].imag = (fftOutMic[i].real*fftOutFm[i].imag - fftOutMic[i].imag*fftOutFm[i].real)/2048;
		}
        //ifft给z
	    Ifx_FftF32_radix2I(fftOutFm, fftIn, 2048);

        int maxpos = 0;
        unsigned int maxval = 0;
        //寻找最大值
        for (unsigned short i = 0; i < 300; i++) {    //0~150，即为0~5m
            if (maxval < fftOutFm[i].real) {
                maxval = fftOutFm[i].real;
                maxpos = i;
            }
            // {
            //     char buffer[30] = "";
            //     sprintf(buffer, "%d\r\n", (int)fftOutFm[i].real);
            //     UART_PutStr(UART2, buffer);
            // }
        }
        // {
        //         char buffer[30] = "";
        //         sprintf(buffer, "%d %d\r\n", maxval,maxval);
        //         UART_PutStr(UART2, buffer);
        //     }
        // lastmaxval=maxval;
        // for (unsigned short i = 0; i < 300; i++) {    //0~150，即为0~5m
        //     {
                
        //         UART_PutStr(UART2, "0\r\n");
        //     }
        // }
        
        distance=maxpos;
        // for(int i=0;i<170;i++){
        // 	{
        // 		UART_PutStr(UART2, "0\r\n");
        // 	}
        // }

        // {
        // 	char buf[30] = {0};
        // 	sprintf(buf, "%d\r\n",
        // 			count);

        // 	UART_PutStr(UART2, buf);
        // }
        rightFftCount = count;
        if (lastrightFftCount != rightFftCount)
        {
            if (!isBeaconOn())
            {
                OLED_ShowString(0, 0, "BeaconOff     ", 8);
            }
            else
            {
                OLED_ShowString(0, 0, "BeaconOn      ", 8);
            }
        }
        lastrightFftCount = rightFftCount;
    }
}