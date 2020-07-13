/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�TC264DA���İ�
����    д��ZYF/chiusir
��E-mail  ��chiusir@163.com
������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
�������¡�2020��4��10��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��dev.env.��Hightec4.9.3/Tasking6.3�����ϰ汾
��Target �� TC264DA
��Crystal�� 20.000Mhz
��SYS PLL�� 200MHz
________________________________________________________________

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <LQ_I2C_9AX.h>
#include <LQ_OLED096.h>
#include <LQ_SOFTI2C.h>
#include <LQ_STM.h>
#include <stdio.h>

/**
  * @brief    �������
  *
  * @param
  *
  * @return
  *
  * @note
  *
  * @example
  *
  * @date     2019/6/13 ������
  */
void Test_9AX(void)
{
    char  txt[30];
	short aacx,aacy,aacz;	         //���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;        //������ԭʼ����
    short magx, magy, magz;         //�شż�ԭʼ����

    OLED_Init();                      //LCD��ʼ��
    OLED_CLS();                       //LCD����

    IIC_Init();


    OLED_P8x16Str(15,0,"LQ 9AX Test");

    if(LQ9AX_Init())
    {

    	OLED_P8x16Str(0,2,"9AX Test Fail");
#pragma warning 557         // ���ξ���
		while (1);
#pragma warning default     // �򿪾���
    }

    while(1)
    {
        FX_8700_GetACCRaw(&aacx,&aacy,&aacz);	  //�õ����ٶȴ���������
        FX_8700_GetMAGRaw(&magx, &magy, &magz);
        FX_21002_GetRaw(&gyrox, &gyroy, &gyroz);

        sprintf((char*)txt,"ax:%06d",aacx);
        OLED_P6x8Str(0,2,txt);
        sprintf((char*)txt,"ay:%06d",aacy);
        OLED_P6x8Str(0,3,txt);
        sprintf((char*)txt,"az:%06d",aacz);
        OLED_P6x8Str(0,4,txt);
        sprintf((char*)txt,"gx:%06d",gyrox);
        OLED_P6x8Str(0,5,txt);
        sprintf((char*)txt,"gy:%06d",gyroy);
        OLED_P6x8Str(0,6,txt);
        sprintf((char*)txt,"gz:%06d",gyroz);
        OLED_P6x8Str(0,7,txt);
        sprintf((char*)txt,"MX:%5d ",magx);
        OLED_P6x8Str(60,5,txt);
        sprintf((char*)txt,"MY:%5d ",magy);
        OLED_P6x8Str(60,6,txt);
        sprintf((char*)txt,"MZ:%5d ",magz);
        OLED_P6x8Str(60,7,txt);

        delayms(100);

    }

}


/**
  * @brief    ����ȷ��ʱ
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @example  delayms_9ax(100);
  *
  * @date     2019/4/22 ����һ
*/
void delayms_9ax(signed short ms)
{
	volatile uint32_t i = 0;
	while(ms--)
	{
		for (i = 0; i < 30000; ++i)
		{
			__asm("NOP"); /* delay */
		}
	}
}



/*!
  * @brief    ����Ĵ�����ʼ��
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      LQ9AX_Init();
  *
  * @date     2019/6/13 ������
  */
unsigned char LQ9AX_Init(void)
{
    FX_8700_Active(0);
    delayms_9ax(10);

    if(FX_8700_Check())
    {
//        PRINTF("\n FX_8700 Init Fail \n");
        return 1;
    }
    FX_8700_SetRate(800, 2);   // 800Hz  �شźͼ��ٶȼƶ�ʹ��
    FX_8700_SetRange(4);       //�� 4g
    FX_8700_Active(1);


    FX_21002_Active(0);
    delayms_9ax(10);

    if(FX_21002_Check())
    {
//        PRINTF("\n FX_2100 Init Fail \n");
        return 2;
    }
    FX_21002_SetRate(800);     // 800Hz  ���ٶȼ�
    FX_21002_SetRange(2000);   //�� 2000dps
    FX_21002_SetLP(2);         //��ͨ�˲�
    FX_21002_Active(1);
    delayms_9ax(10);
    return 0;
}










/*!
  * @brief    ����FXOS8700 �Ƿ񼤻�
  *
  * @param    enable :  1������  0������
  *
  * @return   0 : �ɹ�  ���� �� ʧ��
  *
  * @note     ��
  *
  * @see      FX_8700_Active(1); //ʹ��
  *
  * @date     2019/6/13 ������
  */
unsigned char FX_8700_Active(unsigned char enable)
{
    /* �����߷ֱ���ģʽ */
    FX_Write_Byte(FXOS8700_ADDR, FXOS8701CQ_CTRL_REG2, 0x02);

    unsigned char reg = FX_Read_Byte(FXOS8700_ADDR, FXOS8701CQ_CTRL_REG1);
    if(enable)
    {
        reg = reg | FXOS8701CQ_CTRL_REG1_ACTIVE;
    }
    else
    {
        reg = reg & ~FXOS8701CQ_CTRL_REG1_ACTIVE;
    }
    return FX_Write_Byte(FXOS8700_ADDR, FXOS8701CQ_CTRL_REG1, reg);
}

/*!
  * @brief    ��ȡ���ٶȼ�ԭʼ����
  *
  * @param    ax,ay,az:���ٶȼ�x,y,z���ԭʼ����(������)
  *
  * @return   0 : �ɹ�  ���� �� ʧ��
  *
  * @note     ��
  *
  * @see      int16_t data[3];
  * @see      FX_8700_GetACCRaw(&data[0], &data[1], &data[2]);//��ȡ���ٶ�ԭʼ����
  *
  * @date     2019/6/13 ������
  */
unsigned char FX_8700_GetACCRaw(short *ax,short *ay,short *az)
{
    unsigned char buf[6],res;
    res = FX_Read_Len(FXOS8700_ADDR, FXOS8701CQ_OUT_X_MSB, 6, buf);
    if(res==0)
	{
		*ax = -((signed short)buf[0]<<8)|buf[1];
		*ay = -((signed short)buf[2]<<8)|buf[3];
		*az = ((signed short)buf[4]<<8)|buf[5];
	}
    return res;
}


/*!
  * @brief    ��ȡ�شżƼ�ԭʼ����
  *
  * @param    mx,my,mz:�شż�x,y,z���ԭʼ����(������)
  *
  * @return   0 : �ɹ�  ���� �� ʧ��
  *
  * @note     ��
  *
  * @see      int16_t data[3];
  * @see      FX_8700_GetMAGRaw(&data[0], &data[1], &data[2]);//��ȡ�شż�ԭʼ����
  *
  * @date     2019/6/13 ������
  */
unsigned char FX_8700_GetMAGRaw(short *mx,short *my,short *mz)
{
    unsigned char buf[6],res;
    res = FX_Read_Len(FXOS8700_ADDR, FXOS8701CQ_M_OUT_X_MSB, 6, buf);
    if(res==0)
	{
		*mx = -((signed short)buf[0]<<8)|buf[1];
		*my = -((signed short)buf[2]<<8)|buf[3];
		*mz = ((signed short)buf[4]<<8)|buf[5];
	}
    return res;
}



/*!
  * @brief    ����FXOS8700 ����
  *
  * @param    dps :  �������� ��λ ��ÿ��
  *
  * @return   0 : �ɹ�  ���� �� ʧ��
  *
  * @note     �شżƺͼ��ٶȼ� ������һ���趨�� ��2g;��0.244 mg/LSB   ��4g;��0.488 mg/LSB   ��8g ��0.976 mg/LSB
  *
  * @see      FX_8700_SetRange(4); //�������� ��4g
  *
  * @date     2019/6/13 ������
  */
unsigned char FX_8700_SetRange(unsigned char mg)
{
    unsigned char reg = FX_Read_Byte(FXOS8700_ADDR, FXOS8701CQ_XYZ_DATA_CFG);
    reg = reg & ~XYZ_DATA_CFG_FS_MASK;
    if(mg <= 2)
    {
        reg |= XYZ_DATA_CFG_FS_2G;
    }
    else if(mg <= 4)
    {
        reg |= XYZ_DATA_CFG_FS_4G;
    }
    else
    {
        reg |= XYZ_DATA_CFG_FS_8G;
    }

    return FX_Write_Byte(FXOS8700_ADDR, FXOS8701CQ_XYZ_DATA_CFG, reg);

}


/*!
  * @brief    ����FXOS8700 ��������
  *
  * @param    fps  ����������� ��λ 1��ˢ�¶��ٴ�
  * @param    mode 0: ��ʹ�ü��ٶȼ�  1����ʹ�õشż�  2�����ٶȼƺ͵شżƶ��ã��ᵼ��������ʼ��룩
  *
  * @return   0 : �ɹ�  ���� �� ʧ��
  *
  * @note     ��
  *
  * @see      FX_8700_SetRate(800, 2);   // 800Hz  �شźͼ��ٶȼƶ�ʹ��
  *
  * @date     2019/6/13 ������
  */
unsigned char FX_8700_SetRate(signed short fps, unsigned char mode)
{
    if(mode == 0)
    {
        FX_Write_Byte(FXOS8700_ADDR, FXOS8701CQ_M_CTRL_REG1, 0x00);
    }
    else if(mode == 1)
    {
        FX_Write_Byte(FXOS8700_ADDR, FXOS8701CQ_M_CTRL_REG1, 0x01);
    }
    else
    {
        FX_Write_Byte(FXOS8700_ADDR, FXOS8701CQ_M_CTRL_REG1, 0x40|0x1C|0x03);
    }
    unsigned char reg = FX_Read_Byte(FXOS8700_ADDR, FXOS8701CQ_CTRL_REG1);
    reg &= ~CTRL_REG_DR_MASK;

    if(fps <= 6)
    {
        reg |= CTRL_REG1_DR(6);
    }
    else if(fps <= 12)
    {
        reg |= CTRL_REG1_DR(5);
    }
    else if(fps <= 50)
    {
        reg |= CTRL_REG1_DR(4);
    }
    else if(fps <= 100)
    {
        reg |= CTRL_REG1_DR(3);
    }
    else if(fps <= 200)
    {
        reg |= CTRL_REG1_DR(2);
    }
    else if(fps <= 400)
    {
        reg |= CTRL_REG1_DR(1);
    }
    else
    {
        reg |= CTRL_REG1_DR(0);
    }
    reg &= ~0x02;
    return FX_Write_Byte(FXOS8700_ADDR, FXOS8701CQ_CTRL_REG1, reg);
}


/*!
  * @brief    FXOS8700 �Լ�
  *
  * @param    ��
  *
  * @return   0 : ����   ����������
  *
  * @note     ��
  *
  * @see      FX_8700_Check(); //��ȡID ����Ƿ���������
  *
  * @date     2019/6/13 ������
  */
unsigned char FX_8700_Check(void)
{
    unsigned char ID = 0;

    ID = FX_Read_Byte(FXOS8700_ADDR, FXOS8701CQ_WHOAMI);
    if(FXOS8700CQ_WHOAMI_VAL == ID || FXOS8701CQ_WHOAMI_VAL == ID)
    {
//        PRINTF("\n FXOS8700 is OK\n");
        return 0;
    }
    else
    {
//        PRINTF("\n FXOS8700 is Failed\n\rCheck ID = %x \n" , ID);
    }
    return 1;
}


/*--------------------------------------------------------------------------------------
----------------------------------FXAS21002---------------------------------------------
----------------------------------------------------------------------------------------*/




/*!
  * @brief    ��ȡ���ٶȼ�ԭʼ����
  *
  * @param    gyrox,gyroy,gyroz:������x,y,z���ԭʼ����(������)
  *
  * @return   0 : ����   ����������
  *
  * @note     ��
  *
  * @see      int16_t data[3];
  * @see      FX_21002_GetRaw(&data[0], &data[1], &data[2]);//��ȡ������ԭʼ����
  *
  * @date     2019/6/13 ������
  */
unsigned char FX_21002_GetRaw(short * gyrox, short *gyroy, short *gyroz)
{
    unsigned char buf[6],res;
    res = FX_Read_Len(FXAS21002C_ADDR, FXAS21002C_OUT_X_MSB, 6, buf);
    if(res==0)
	{
		*gyrox = ((signed short)buf[0]<<8)|buf[1];
		*gyroy = ((signed short)buf[2]<<8)|buf[3];
		*gyroz = ((signed short)buf[4]<<8)|buf[5];
	}
    return res;
}


/*!
  * @brief    ����FXAS21002 �Ƿ񼤻�
  *
  * @param    enable :  1������  0������
  *
  * @return   0 : ����   ����������
  *
  * @note     ��
  *
  * @see      FX_21002_Active(1); //ʹ��FX_21002
  *
  * @date     2019/6/13 ������
  */
unsigned char FX_21002_Active(unsigned char enable)
{
    unsigned char reg = FX_Read_Byte(FXAS21002C_ADDR, FXAS21002C_CTRL_REG1);
    if(enable)
    {
        reg = reg | CTRL_REG1_ACTIVE;
    }
    else
    {
        reg = reg & ~CTRL_REG1_READY;
    }
    return FX_Write_Byte(FXAS21002C_ADDR, FXAS21002C_CTRL_REG1, reg);
}



/*!
  * @brief    ����FXAS21002 ��ͨ �˲�
  *
  * @param    BW :  ���ý�ֹƵ�ʣ�����Ƶ�ʿ���
  *
  * @return   0 : ����   ����������
  *
  * @note     ��ͬ����Ƶ����  ��ͨ�˲��������õ�ֵ���±�
  *
  *    mode        ��������    800 Hz 400 Hz 200 Hz  100 Hz 50 Hz  25 Hz 12.5 Hz
  *     0          ��ֹƵ��      15    7.5    3.75   1.875  0.937 0.468  0.234
  *     1          ��ֹƵ��     7.7   3.85   1.925   0.963  0.481 0.241  0.120
  *     2          ��ֹƵ��     3.9   1.95   0.975   0.488  0.244 0.122  0.061
  *     3          ��ֹƵ��    1.98   0.99   0.495   0.248  0.124 0.062  0.031
  *
  * @see      FX_21002_SetHP(0); //���ø�ͨ�˲�
  *
  * @date     2019/6/13 ������
  */
unsigned char FX_21002_SetHP(unsigned char mode)
{
    unsigned char reg = FX_Read_Byte(FXAS21002C_ADDR, FXAS21002C_CTRL_REG0);
    reg = reg | CTRL_REG0_HPF_EN;
    reg |= (mode << CTRL_REG0_SEL_SHIFTS);
    return FX_Write_Byte(FXAS21002C_ADDR, FXAS21002C_CTRL_REG0, reg);
}



/*!
  * @brief    ����FXAS21002 ��ͨ �˲�
  *
  * @param    BW :  ���ý�ֹƵ�ʣ�����Ƶ�ʿ���
  *
  * @return   0 : ����   ����������
  *
  * @note     ��ͬ����Ƶ����  ��ͨ�˲��������õ�ֵ���±�
  *  mode     ��������  800 Hz  400 Hz  200 Hz  100 Hz  50 Hz  25 Hz  12.5 Hz
  *  0        ��ֹƵ��  256     128      64      32     16      8        4
  *  1        ��ֹƵ��  128      64      32      16      8      4       ��
  *  2        ��ֹƵ��   64      32      16       8      4     ��       ��
  *
  * @see      FX_21002_SetLP(0) //����������� 800Hz ���ͨ�˲���ֹƵ�� 256
  *
  * @date     2019/6/13 ������
  */
unsigned char FX_21002_SetLP(unsigned char mode)
{
    unsigned char reg = FX_Read_Byte(FXAS21002C_ADDR, FXAS21002C_CTRL_REG0);
    reg = reg & ~CTRL_REG0_BW_MASK;
    reg |= (mode << CTRL_REG0_BW_SHIFTS);
    return FX_Write_Byte(FXAS21002C_ADDR, FXAS21002C_CTRL_REG0, reg);
}



/*!
  * @brief    ����FXAS21002 ����
  *
  * @param    dps :  �������� ��λ ��ÿ��
  *
  * @return   0 : ����   ����������
  *
  * @note     ��250dps;1,��500dps;2,��1000dps;3,��2000dps
  *
  * @see      FX_21002_SetRange(2000); //�������� ��2000dps
  *
  * @date     2019/6/13 ������
  */
unsigned char FX_21002_SetRange(unsigned short dps)
{
    unsigned char reg = FX_Read_Byte(FXAS21002C_ADDR, FXAS21002C_CTRL_REG0);
    reg = reg & ~CTRL_REG0_FS_MASK;
    if(dps <= 250)
    {
        reg |= CTRL_REG0_FS_250_DPS;
    }
    else if(dps <= 500)
    {
        reg |= CTRL_REG0_FS_500_DPS;
    }
    else if(dps <= 1000)
    {
        reg |= CTRL_REG0_FS_1000_DPS;
    }
    else
    {
        reg |= CTRL_REG0_FS_2000_DPS;
    }

    return FX_Write_Byte(FXAS21002C_ADDR, FXAS21002C_CTRL_REG0, reg);

}


/*!
  * @brief    ����FXAS21002 ��������
  *
  * @param    fps ����������� ��λ 1��ˢ�¶��ٴ�
  *
  * @return   0 : ����   ����������
  *
  * @note     ��
  *
  * @see      FX_21002_SetRate(800); //���ò�����800Hz
  *
  * @date     2019/6/13 ������
  */
unsigned char FX_21002_SetRate(unsigned short fps)
{
    unsigned char reg = FX_Read_Byte(FXAS21002C_ADDR, FXAS21002C_CTRL_REG1);
    reg &= ~CTRL_REG_DR_MASK;

    if(fps <= 13)
    {
        reg |= CTRL_REG_DR_12_5;
    }
    else if(fps <= 25)
    {
        reg |= CTRL_REG_DR_25HZ;
    }
    else if(fps <= 50)
    {
        reg |= CTRL_REG_DR_50HZ;
    }
    else if(fps <= 100)
    {
        reg |= CTRL_REG_DR_100HZ;
    }
    else if(fps <= 200)
    {
        reg |= CTRL_REG_DR_200HZ;
    }
    else if(fps <= 400)
    {
        reg |= CTRL_REG_DR_400HZ;
    }
    else
    {
        reg |= CTRL_REG_DR_800HZ;
    }
    return FX_Write_Byte(FXAS21002C_ADDR, FXAS21002C_CTRL_REG1, reg);
}



/*!
  * @brief    FXAS21002 �Լ�
  *
  * @param    ��
  *
  * @return   0 : ����   ����������
  *
  * @note     ��
  *
  * @see      FX_21002_Check(); //�Լ�
  *
  * @date     2019/6/13 ������
  */
unsigned char FX_21002_Check(void)
{
    unsigned char ID = 0;

    ID = FX_Read_Byte(FXAS21002C_ADDR, FXAS21002C_WHO_AM_I);
    if(FXAS21002C_ID == ID)
    {
//        PRINTF("\n FXAS21002 is OK\n");
        return 0;
    }
    else
    {
//        PRINTF("\n FXAS21002 is Failed\n\rCheck ID = %x \n" , ID);
    }
    return 1;
}








/*--------------------------------------------------------------------------------------
----------------------------------�ײ���������------------------------------------------
----------------------------------------------------------------------------------------*/





/*!
  * @brief    IIC ������
  *
  * @param    addr:������ַ
  * @param    reg :Ҫ��ȡ�ļĴ�����ַ
  * @param    len :Ҫ��ȡ�ĳ���
  * @param    buf :��ȡ�������ݴ洢��
  *
  * @return   0 ����ȡ�ɹ�
  *
  * @note     ��ֲʱ���ʹ��ģ��IIC ��Ҫ�޸�����
  *
  * @see      unsigned char buf[6],res;
  * @see      res = FX_Read_Len(FXAS21002C_ADDR, FXAS21002C_OUT_X_MSB, 6, buf);//��ȡ����������
  *
  * @date     2019/6/12 ������
  */
unsigned char FX_Read_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{
    return IIC_ReadMultByteFromSlave(addr<<1, reg, len, buf);
}



/*!
  * @brief    IIC дһ���Ĵ���
  *
  * @param    addr  :������ַ
  * @param    reg   :Ҫд�ļĴ�����ַ
  * @param    value :Ҫд���ֵ
  *
  * @return   0 ����ȡ�ɹ�
  *
  * @note     ��ֲʱ���ʹ��ģ��IIC ��Ҫ�޸�����
  *
  * @see      FX_Write_Byte(FXAS21002C_ADDR, FXAS21002C_CTRL_REG1, 0x00); //дһ���Ĵ���
  *
  * @date     2019/6/12 ������
  */
unsigned char FX_Write_Byte(unsigned char addr,unsigned char reg,unsigned char value)
{
    return IIC_WriteByteToSlave(addr<<1, reg, value);
}


/*!
  * @brief    IIC ��һ���Ĵ���
  *
  * @param    addr  :������ַ
  * @param    reg   :Ҫ���ļĴ�����ַ
  *
  * @return   0 ����ȡ�ɹ�
  *
  * @note     ��ֲʱ���ʹ��ģ��IIC ��Ҫ�޸�����
  *
  * @see      FX_Read_Byte(FXAS21002C_ADDR, FXAS21002C_WHO_AM_I); //��һ���Ĵ���
  *
  * @date     2019/6/12 ������
  */
unsigned char FX_Read_Byte(unsigned char addr,unsigned char reg)
{
    unsigned char value[1];
    FX_Read_Len(addr, reg, 1, value);
    return value[0];
}
