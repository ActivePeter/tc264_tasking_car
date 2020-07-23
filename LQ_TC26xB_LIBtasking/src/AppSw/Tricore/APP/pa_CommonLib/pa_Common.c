#include "pa_Common.h"

#ifdef LQ_TC264
#include <LQ_GPIO.h>
#include <LQ_GPIO_LED.h>
#include <LQ_STM.h>
void pa_delayMs(unsigned short ms){
    delayms(ms);
} 

void pa_delayUs(unsigned short us){
    STM_DelayUs(STM0, us);
}

void pa_millis(){

}

void pa_gotoErrorBlink(){
    while (1)
    {
        pa_delayMs(500);
        PIN_Reverse(LED0p);
    }
    
}

#endif

