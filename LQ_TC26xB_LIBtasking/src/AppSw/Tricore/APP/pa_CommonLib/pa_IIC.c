#include "pa_IIC.h"

char iic_inited = 0;

//龙湫tc264
#ifdef LQ_TC264
    #include <LQ_SOFTI2C.h>
    void pa_IIC_init(){
        if(iic_inited==0){
            IIC_Init();
            iic_inited=1;
        }
        
    }
    void pa_IIC_write8(unsigned char addr,unsigned char reg,unsigned char value){
        IIC_WriteByteToSlave(addr, reg, value);
    }
    unsigned char pa_IIC_read8(unsigned char addr,unsigned char reg){
        unsigned char value[1];
        IIC_ReadMultByteFromSlave(addr, reg, 1, value);
        return value[0];
    }
#endif
