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

#include <LQ_I2C_VL53.h>
#include <LQ_OLED096.h>
#include <LQ_SOFTI2C.h>
#include <LQ_STM.h>
#include <Platform_Types.h>
#include <stdio.h>

/* ѡ����Ļ */
#define OLED

/**
  * @brief    ����VL53
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @example
  *
  * @date     2019/4/17 ������
  */
void Test_Vl53(void)
{

#ifdef OLED
	OLED_Init();
	OLED_CLS();
	OLED_P8x16Str(5,0,"LQ VL53 Test");
#else
	TFTSPI_Init(1);                //TFT1.8��ʼ��  0��������ʾ  1��������ʾ
    TFTSPI_CLS(u16BLUE);           //����
	TFTSPI_P8X16Str(0,0,"LQ VL53 Test",u16RED,u16BLUE);
#endif
	char txt[16];

    IIC_Init();

    unsigned char VL53_STAR = 0x02;    //0x02 ��������ģʽ    0x01 ���β���ģʽ
    unsigned char dis_buff[2];
    uint16 dis, last_dis = 0;
	/* ��ʼ��� */
    VL53_Write_Byte(VL53ADDR, VL53L0X_REG_SYSRANGE_START, VL53_STAR);
    while(1)
    {
        /* ��ȡ�������� */
		VL53_Read_nByte(VL53ADDR, VL53_REG_DIS, 2, dis_buff);

        /* ת������ */
        dis = (dis_buff[0]<<8) | (dis_buff[1]);

        /* ����2Mû���ϰ� */
        if(dis > 8000)
        {
            dis = 0;
        }

        if(dis == 20)
        {
            dis = last_dis;
        }
        last_dis = dis;
		sprintf(txt, "DIS %5d mm",dis);
#ifdef OLED
		OLED_P8x16Str(0,5,txt);
#else
		TFTSPI_P8X16Str(2,5,txt,u16RED,u16BLUE);
#endif
		delayms(50);


    }

}





/**
  * @brief    VL53 д1���Ĵ���
  *
  * @param    dev��    ��ַ
  * @param    reg��    �Ĵ���
  * @param    data��   д������
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2019/4/29 ����һ
  */
void VL53_Write_Byte(unsigned char dev, unsigned char reg, unsigned char data)
{
    IIC_WriteByteToSlave(dev<<1, reg, data);
}

/**
  * @brief    VL53 ��n���Ĵ���
  *
  * @param    dev��    ��ַ
  * @param    reg��    �Ĵ���
  * @param    length;  ����
  * @param    data��   ָ���Ŷ�����
  *
  * @return
  *
  * @note
  *
  * @see
  *
  * @date     2019/4/29 ����һ
  */
void VL53_Read_nByte(unsigned char dev, unsigned char reg, unsigned char length, unsigned char* data)
{
    IIC_ReadMultByteFromSlave(dev<<1, reg, length, data);
}

