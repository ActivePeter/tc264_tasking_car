#include "pa_BNO055.h"

#define pa_BNO055_chosenAddress (0x29)

pa_IICSettingStruct BNO055_IICSettingStruct;

void pa_BNO055_init(){
    pa_IIC_init();
    pa_delayMs(1000);
    unsigned char id= pa_IIC_read8(pa_BNO055_chosenAddress<<1,BNO055_CHIP_ID_ADDR,BNO055_IICSettingStruct);
    if (id != BNO055_ID) {
    pa_delayMs(1000); // hold on for boot
    id = pa_IIC_read8(pa_BNO055_chosenAddress<<1,BNO055_CHIP_ID_ADDR,BNO055_IICSettingStruct);
    if (id != BNO055_ID) {
        pa_gotoErrorBlink();
      return; // still not? ok bail
    }
  }
}