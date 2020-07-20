#include "pa_SoundLocationCalculator.h"
extern "C"
{
#include "stdio.h"
#include <LQ_UART.h>
#include "string.h"
#include "math.h"
}

char pa_SoundLocationCalculator::crossCorrelation_noFFT(unsigned short adc_arr1[],unsigned short adc_arr2[],short adc_arrlen){
    long max = 0, maxpos = 0;
#define detectRange 10
	int i = -detectRange;
	// if (endBuffer)
	// {
	// 	// memcpy(adc_arr1_copy, adc_arr1, adc_arrlen);
	// 	// memcpy(adc_arr2_copy, adc_arr2, adc_arrlen);
	// 	// // 发送收集到的adc*************************************************
	// 	// // for (int a = 0; a < adc_arrlen; a++)
	// 	// // {
	// 	// // 	{
	// 	// // 		char buffer[30] = "";
	// 	// // 		sprintf(buffer, "%d %d\r\n", adc_arr1[a], adc_arr2[a]);
	// 	// // 		UART_PutStr(UART2, buffer);
	// 	// // 	}
	// 	// // }
	// 	// // UART_PutStr(UART2, "done\r\n");
	// 	// // **********************************************
    //     for (; i < detectRange; i++)
    //     {
    //         long sum = 0;
    //         for (int t = 0; t < adc_arrlen; t++)
    //         {
    //             int first_pos = t;
    //             int second_pos = t + i;
    //             if (second_pos < 0)
    //             {
    //                 second_pos += adc_arrlen;
    //             }
    //             else if (second_pos >= adc_arrlen)
    //             {
    //                 second_pos -= adc_arrlen;
    //             }
    //             // if (t + i < 0 || t + i > adc_arrlen - 1)
    //             // 	continue;
    //             sum += (long)adc_arr1[first_pos] * adc_arr2[second_pos];
    //         }
    //         if (sum > max)
    //         {
    //             max = sum;
    //             maxpos = i;
    //         }
    //         // {
    //         // 	char buffer[30] = "";
    //         // 	sprintf(buffer, "%d\r\n", sum);
    //         // 	UART_PutStr(UART2, buffer);
    //         // }
    //     }
	// 	endBuffer = 0;
	}

	
	// UART_PutStr(UART2, "0\r\n");
	{
		char buffer[30] = "";
		sprintf(buffer, "%d\r\n", maxpos);
		UART_PutStr(UART2, buffer);
	}
}