#include "pa_UltrasonicDistance.h"

pa_UltrasonicDistance::pa_UltrasonicDistance(){
    
}

void pa_UltrasonicDistance::trig(){
    this->countingDistance=0;
    //10us高电平
    setTrig(1);
    pa_delayUs(12);
    setTrig(0);
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
    moduleIndex=index;
    switch (moduleIndex)
    {
    case 1:
        PIN_InitConfig(P11_10, PIN_MODE_OUTPUT, 0);
        PIN_InitConfig(P11_6, PIN_MODE_INPUT, 0);
        
        /* code */
        break;
    case 2:
        PIN_InitConfig(P11_11, PIN_MODE_OUTPUT, 0);
        PIN_InitConfig(P11_12, PIN_MODE_INPUT, 0);
        /* code */
        break;
    case 3:
        /* code */
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
        /* code */
        break;
    
    }
}
void pa_UltrasonicDistance::setTrig(char pinState){
    switch (moduleIndex)
    {
    case 1:
        PIN_Write(P11_10,pinState);
        /* code */
        break;
    case 2:
        PIN_Write(P11_11,pinState);
        /* code */
        break;
    case 3:
        /* code */
        break;
    
    }
}