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

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#ifndef _LQ_GTMPWM_H_
#define _LQ_GTMPWM_H_

#include "IfxGtm_PinMap.h"
#include <Platform_Types.h>
#include <stdint.h>
#include <IfxGtm.h>
#include <IfxGtm_Atom_Pwm.h>
#include <IfxGtm_cfg.h>
#include <IfxGtm_Cmu.h>
#include <IfxGtm_PinMap.h>
#include <IfxGtm_reg.h>
#include <IfxGtm_Tim_In.h>
#include <IfxGtm_Tom.h>
#include <IfxGtm_Tom_Pwm.h>
#include <IfxPort.h>

// ATOM ʱ�� 100MHz
#define ATOM_PWM_CLK  100000000

// ATOM ���ռ�ձ� �������޸�
#define ATOM_PWM_MAX  10000.0f


// TOM ʱ�� 6.25MHz/2
#define TOM_PWM_CLK  6250000

// TOM ���ռ�ձ� �������޸�
#define TOM_PWM_MAX  10000.0f



////////////////ATOM_PWM//////////////////////
void ATOM_PWM_InitConfig(IfxGtm_Atom_ToutMap pin, uint32_t duty, uint32_t pwmFreq_Hz);
void ATOM_PWM_SetDuty(IfxGtm_Atom_ToutMap pin, uint32_t duty, uint32_t pwmFreq_Hz);



////////////////TOM_TIM//////////////////////
void TOM_PWM_InitConfig(IfxGtm_Tom_ToutMap pin, uint32_t duty, uint32_t pwmFreq_Hz);
void TOM_PWM_SetDuty(IfxGtm_Tom_ToutMap pin, uint32_t duty, uint32_t pwmFreq_Hz);


void TIM_InitConfig(IfxGtm_Tim_TinMap pin);
void TIM_GetPwm(IfxGtm_Tim_TinMap pin, float32 *Period, float32 *Duty);


#endif /* 0_APPSW_TRICORE_APP_LQ_GTM6PWM_H_ */
