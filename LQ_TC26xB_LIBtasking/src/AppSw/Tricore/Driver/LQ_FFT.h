/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�TC264DA���İ�
����    д��ZYF/chiusir
��E-mail  ��chiusir@163.com
������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
�������¡�2020��4��10��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��dev.env.��Hightec4.9.3/Tasking6.3�����ϰ汾
��Target �� TC264DA
��Crystal�� 20.000Mhz
��SYS PLL�� 200MHz
________________________________________________________________
ע��  Ӳ��FFT��ҪTC264 DA��оƬ���д˹���
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef _LQ_FFT_H_
#define _LQ_FFT_H_

#include "Cpu/Std/Platform_Types.h"
#include "Fft/Std/IfxFft.h"
#include "stdint.h"
#include <Ifx_Types.h>
#include <IfxCpu.h>
#include <IfxCpu_Irq.h>
#include <IfxDma_cfg.h>
#include <IfxFft.h>
#include <IfxFft_Fft.h>
#include <IfxFft_reg.h>
#include <IfxLmu_reg.h>
#include <math.h>
#include <Platform_Types.h>
#include <stdint.h>

/** FFT�жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  FFT_INPUT_PRIORITY    250

/** FFT�жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  FFT_OUTPUT_PRIORITY   251

/** FFT�жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  FFT_INTRA_PRIORITY    252

/** FFT�жϹ��ĸ��ں˹��� ��Χ��0��CPU0   1��CPU1   3��DMA*/
#define  FFT_VECTABNUM         0




void FFT_InitConfig(uint32 InDataAddr, uint32 OutDataAddr, IfxFft_Length len);
void IFFT_InitConfig(uint32 InDataAddr, uint32 OutDataAddr, IfxFft_Length len);
void FFT_Start(void);


#endif /* 0_APPSW_TRICORE_APP_LQ_FFT_H_ */
