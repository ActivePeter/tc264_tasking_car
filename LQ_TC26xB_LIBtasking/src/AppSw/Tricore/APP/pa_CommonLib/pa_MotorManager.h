#ifndef __pa_MotorManager_H_
#define __pa_MotorManager_H_

#include <LQ_GTM.h>
#define PWM1 IfxGtm_TOM0_12_TOUT72_P15_1_OUT
#define PWM2 IfxGtm_TOM0_0_TOUT32_P33_10_OUT

#define PWM3 IfxGtm_TOM0_14_TOUT74_P15_3_OUT
#define PWM4 IfxGtm_TOM0_1_TOUT78_P15_7_OUT

#define PWM5 IfxGtm_TOM0_2_TOUT33_P33_11_OUT
#define PWM6 IfxGtm_TOM1_1_TOUT31_P33_9_OUT

#define PWM7  IfxGtm_TOM1_4_TOUT30_P33_8_OUT
#define PWM8  IfxGtm_TOM0_13_TOUT73_P15_2_OUT

void pa_initMotorPwm();
void pa_updateMotorPwm(char index,float speed);

#endif