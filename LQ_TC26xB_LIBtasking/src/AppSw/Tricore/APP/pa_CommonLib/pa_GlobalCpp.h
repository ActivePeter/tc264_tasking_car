#ifndef __pa_GlobalCpp_H_
#define __pa_GlobalCpp_H_
#include "pa_PID.h"

#define OutputMode_Mic 1
#define OutputMode_cross 2
#define OutputMode_motor 3
#define OutputMode_crossDetail 4
#define OutputMode_UltrasonicDistance 5

class pa_GlobalCpp
{
public:
    pa_GlobalCpp();
    int targetSpeed=0;
    pa_PID pid_Motor1;
    pa_PID pid_Motor2;
    pa_PID pid_Motor3;
    pa_PID pid_Motor4;
    pa_PID pid_Direction;
    pa_PID pid_DirectionCalibration;
    static pa_GlobalCpp * getGlobalCpp();
    char motorDisable=0;
    char micOutPutMode=0;

private:
    
};


#endif