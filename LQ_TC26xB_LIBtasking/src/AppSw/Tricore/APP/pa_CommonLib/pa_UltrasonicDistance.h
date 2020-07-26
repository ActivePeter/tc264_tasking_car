#ifndef __pa_UltrasonicDistance_H_
#define __pa_UltrasonicDistance_H_



extern "C"
{
    #include "pa_drv/pa_Common.h"
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

    
    
private:
    char countingDistance=0;
    char prePinState=0;
    char moduleIndex=0;
    char getEcho();
    void setTrig(char pinState);
};

#endif