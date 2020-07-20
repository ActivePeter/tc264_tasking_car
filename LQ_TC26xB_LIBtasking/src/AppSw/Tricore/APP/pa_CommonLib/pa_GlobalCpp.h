#ifndef __pa_GlobalCpp_H_
#define __pa_GlobalCpp_H_
#include "pa_PID.h"


class pa_GlobalCpp
{
public:
    pa_GlobalCpp();
    int targetSpeed=0;
    pa_PID * pid_Motor1;
    pa_PID * pid_Motor2;
    pa_PID * pid_Motor3;
    pa_PID * pid_Motor4;
    static pa_GlobalCpp getGlobalCpp();
private:
    
};


#endif