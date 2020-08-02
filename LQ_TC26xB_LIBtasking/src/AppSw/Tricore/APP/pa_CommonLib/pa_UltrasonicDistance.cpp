#include "pa_UltrasonicDistance.h"

pa_UltrasonicDistance::pa_UltrasonicDistance(){
    
}

bool pa_UltrasonicDistance::trigInited=false;

void pa_UltrasonicDistance::trig(){
    
    //10us高电平
    setTrig(1);
    pa_delayUs(12);
    setTrig(0);
}

void pa_UltrasonicDistance::doBeforeTrig(){
    this->countingDistance=0;
}

void pa_UltrasonicDistance::checkEcho(){
    char pinState=getEcho();
    //echo高电平
    if(pinState){
        if(countingDistance<100){
            this->countingDistance++;
        }
    }
    if(!pinState&&prePinState){//当前低 之前高。表明信号结束
        this->distance=this->countingDistance;
    }
    this->prePinState=pinState;
}

void pa_UltrasonicDistance::init(char index){
    if(!trigInited){
        trigInited=true;
        PIN_InitConfig(P11_9, PIN_MODE_OUTPUT, 0);
    }
    moduleIndex=index;
    switch (moduleIndex)
    {
    case 1:
        PIN_InitConfig(P11_6, PIN_MODE_INPUT, 0);
        /* code */
        break;
    case 2:
        // PIN_InitConfig(P11_11, PIN_MODE_OUTPUT, 0);
        PIN_InitConfig(P11_12, PIN_MODE_INPUT, 0);
        /* code */
        break;
    case 3:
        PIN_InitConfig(P11_11, PIN_MODE_INPUT, 0);
        /* code */
        break;
    case 4:
        PIN_InitConfig(P11_10,PIN_MODE_INPUT, 0);
        break;
    }
}
char pa_UltrasonicDistance::getEcho(){
    switch (moduleIndex)
    {
    case 1:
        return PIN_Read(P11_6);
        /* code */
        break;
    case 2:
        return PIN_Read(P11_12);
        /* code */
        break;
    case 3:
        return PIN_Read(P11_11);
        /* code */
        break;
    case 4:
        return PIN_Read(P11_10);
        /* code */
        break;
    
    }
    return -50;
}
void pa_UltrasonicDistance::setTrig(char pinState){
    PIN_Write(P11_9,pinState);
}