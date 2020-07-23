#ifndef _pa_IIC_H
#define _pa_IIC_H
#include "_pa_ChooseMCU.h"
    void pa_IIC_init();
    void pa_IIC_write8(unsigned char addr,unsigned char reg,unsigned char value);
    unsigned char pa_IIC_read8(unsigned char addr,unsigned char reg);
    void pa_IIC_writeLen(unsigned char addr, unsigned char reg, unsigned char length, unsigned char* data_t);
#endif