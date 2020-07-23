/****************************************
 * 放基础函数
 * 延时。
 * millis。
 * 输出
 * 
 * 
 * 
 *****************************************/
#ifndef _pa_Common_H
#define _pa_Common_H
#include "../_pa_ChooseMCU.h"
void pa_delayUs(unsigned short us);
void pa_delayMs(unsigned short ms);
void pa_millis();
void pa_gotoErrorBlink();
#endif