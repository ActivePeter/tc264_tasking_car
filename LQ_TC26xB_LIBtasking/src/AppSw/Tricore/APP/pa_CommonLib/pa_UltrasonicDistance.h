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
    static void trig();
    void doBeforeTrig();
    void checkEcho();
    char distance=-1;
    
    void init(char index);
    char countingDistance=0;
    
    
private:
    static bool trigInited;
    char prePinState=0;
    char moduleIndex=0;
    char getEcho();
    static void setTrig(char pinState);
    
};

#endif