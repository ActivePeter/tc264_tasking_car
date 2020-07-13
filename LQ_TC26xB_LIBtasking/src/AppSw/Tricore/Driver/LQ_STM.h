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

����iLLD_1_0_1_11_0�ײ����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#ifndef _LQ_STM_H_
#define _LQ_STM_H_

#include "Cpu/Std/Platform_Types.h"
#include "Stm/Std/IfxStm.h"
#include "stdint.h"
#include <Bsp.h>
#include <CompilerTasking.h>
#include <Ifx_Types.h>
#include <IfxCpu.h>
#include <IfxCpu_IntrinsicsTasking.h>
#include <IfxCpu_Irq.h>
#include <IfxStm_cfg.h>
#include <IfxStm_reg.h>
#include <IfxStm_regdef.h>
/**
 * 	STMģ��ö��
 */
typedef enum
{
	STM0 = 0,
	STM1
}STM_t;

/**
 * 	STMͨ��ö��
 */
typedef enum
{
	STM_Channel_0 = 0,
	STM_Channel_1
}STM_Channel_t;

/** STM��ʱ���ж� STM0  channel0 �жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  STM0_CH0_PRIORITY    110

/** STM��ʱ���ж� STM0  channel1 �жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  STM0_CH1_PRIORITY    111

/** STM��ʱ���ж� STM0  �жϹ��ĸ��ں˹��� ��Χ��0��CPU0   1��CPU1   3��DMA*/
#define  STM0_VECTABNUM       1


/** STM��ʱ���ж� STM1  channel0 �жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  STM1_CH0_PRIORITY    112

/** STM��ʱ���ж� STM1  channel1 �жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  STM1_CH1_PRIORITY    113

/** STM��ʱ���ж� STM1   �жϹ��ĸ��ں˹��� ��Χ��0��CPU0   1��CPU1   3��DMA*/
#define  STM1_VECTABNUM       0


extern IfxStm_CompareConfig g_StmCompareConfig[4];
void STM_InitConfig(STM_t STM, STM_Channel_t channel, uint32 us,void (*task)());
void STM_DelayUs(STM_t stm, uint32 us);
uint32 STM_GetNowUs(STM_t stm);

void delayms(unsigned short stmms);
#endif /* 0_APPSW_TRICORE_APP_LQ_STM_H_ */
