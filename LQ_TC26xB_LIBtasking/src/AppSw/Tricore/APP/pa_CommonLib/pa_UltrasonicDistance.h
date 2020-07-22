#ifndef __pa_UltrasonicDistance_H_
#define __pa_UltrasonicDistance_H_

#include "pa_Common.h"

extern "C"
{
    #ifdef LQ_TC264
        #include <LQ_GPIO.h>
    #endif
}

class pa_UltrasonicDistance
{
public:
    pa_UltrasonicDistance();
    void trig();
    void checkEcho();
    char distance=-1;
    
    void init(char index);

    char getEcho();
    void setTrig(char pinState);
    
private:
    char countingDistance=0;
    char prePinState=0;
    char moduleIndex=0;
};

#endif