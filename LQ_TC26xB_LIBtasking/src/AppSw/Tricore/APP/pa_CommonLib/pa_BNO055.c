#include "pa_BNO055.h"
#include "pa_Common.h"
#define pa_BNO055_chosenAddress BNO055_ADDRESS_B
void pa_BNO055_init(){
    unsigned char id= pa_IIC_read8(pa_BNO055_chosenAddress,BNO055_CHIP_ID_ADDR);
    if (id != BNO055_ID) {
    pa_delayMs(1000); // hold on for boot
    id = pa_IIC_read8(pa_BNO055_chosenAddress,BNO055_CHIP_ID_ADDR);
    if (id != BNO055_ID) {
        pa_gotoErrorBlink();
      return; // still not? ok bail
    }
  }
}