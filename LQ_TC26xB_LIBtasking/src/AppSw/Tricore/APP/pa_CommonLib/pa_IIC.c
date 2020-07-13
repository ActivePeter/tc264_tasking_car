#include "pa_IIC.h"

//龙湫tc264
#ifdef LQ_TC264
    #include <LQ_SOFTI2C.h>
    void pa_IIC_write8(unsigned char addr,unsigned char reg,unsigned char value){
        IIC_WriteByteToSlave(addr, reg, value);
    }
    unsigned char pa_IIC_read8(unsigned char addr,unsigned char reg){
        return MPU_Read_Byte(addr,reg);
    }
#endif
