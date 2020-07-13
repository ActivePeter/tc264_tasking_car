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

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ
ͨ�������������۷ֱ���Ϊ188*120��
________________________________________________________________
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 ����ͷ�ܽ�  TC264DA�ܽ�   ˵��              DMA��ʽ�ɼ����ùܽ�ͨ����������
	D0        P02_0       ����ͷ����bit0
	D1        P02_1       ����ͷ����bit1
	D2        P02_2       ����ͷ����bit2
	D3        P02_3       ����ͷ����bit3
	D4        P02_4       ����ͷ����bit4
	D5        P02_5       ����ͷ����bit5
	D6        P02_6       ����ͷ����bit6
	D7        P02_7       ����ͷ����bit7
	CLK       P00_7       ����ʱ��        �ⲿ�жϵ�2�飺P00_4
	VSNC      P00_8       ���ź�             �ⲿ�жϵ�3�飺P15_1
	HSNC      P00_9       ���ź�
    SCL       P11_2       ����ģ��SCCB SCL
    SDA       P11_3       ����ģ��SCCB SDA
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <IfxPort.h>
#include <LQ_GPIO.h>
#include <LQ_MT9V034.h>
#include <LQ_TFT18.h>



/*!
  * @brief    MT9V034�Ĵ�����ʼ��
  *
  * @param    fps  :֡��
  *
  * @return   ��
  *
  * @note     һ��ʹ��50֡
  *
  * @see      MT9V034_Init(50);
  *
  * @date     2019/6/12 ������
  */
void MT9V034_Init(unsigned char fps)
{
	unsigned short data = 0;

	//����ͷSCCB�ӿڳ�ʼ��
	SCCB_Init();

	if(SCCB_RegRead(MT9V034_I2C_ADDR>>1,MT9V034_CHIP_VERSION,&data) == 0)  //��ȡ����ͷ�汾�Ĵ���
	{
		if(data != MT9V034_CHIP_ID)                                        //оƬID����ȷ��˵��û����ȷ��ȡ�����ݣ�������
		{
#ifdef USEOLED
			OLED_P6x8Str(0, 0, "LQ-V034 id error");
#else
			TFTSPI_P8X8Str(0,8,"LQ-V034 id error",u16RED,u16BLUE);
#endif
#pragma warning 557         // ���ξ���
			while (1);
#pragma warning default     // �򿪾���
		}
	}
	else
	{
#ifdef USEOLED
		OLED_P6x8Str(0, 0, "LQ-V034 sccb error");
#else
		TFTSPI_P8X8Str(0,8,"LQ-V034 sccb error",u16RED,u16BLUE);
#endif

#pragma warning 557         // ���ξ���
		while (1);          //����ͷʶ��ʧ�ܣ�ֹͣ����
#pragma warning default     // �򿪾���
	}
	/* �ָ�Ĭ������ */
//	MT9V034_Default_Settings();

    /* ��������ͷͼ��4*4��Ƶ���PCLK 27/4 = 6.75M ,BIT4,5��������:�������Ҿ����� */
	MT9V034_SetFrameResolution(MT9V034_IMAGEH, MT9V034_IMAGEW);

    /* ����֡�� */
    MT9V034_SetFrameRate(fps);

    /* �ع����� */
    MT9V034_SetAutoExposure(CAMERA_AUTO_EXPOSURE);

	SCCB_RegWrite(MT9V034_I2C_ADDR, 0x2C, 0x0004);  //�ο���ѹ����   1.4v
    SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ANALOG_CTRL, MT9V034_ANTI_ECLIPSE_ENABLE);  //����ʴ
    SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_ADC_RES_CTRL_REG, 0x0303);      //0x1C  here is the way to regulate darkness :)
    ////
    SCCB_RegWrite(MT9V034_I2C_ADDR,0x13,0x2D2E);//We also recommended using R0x13 = 0x2D2E with this setting for better column FPN.
    SCCB_RegWrite(MT9V034_I2C_ADDR,0x20,CAMERA_CONTRAST);//0x01C7�ԱȶȲ���ף�0x03C7�Աȶ���� Recommended by design to improve performance in HDR mode and when frame rate is low.
    SCCB_RegWrite(MT9V034_I2C_ADDR,0x24,0x0010);//Corrects pixel negative dark offset when global reset in R0x20[9] is enabled.
    SCCB_RegWrite(MT9V034_I2C_ADDR,0x2B,0x0003);//Improves column FPN.
    SCCB_RegWrite(MT9V034_I2C_ADDR,0x2F,0x0003);//Improves FPN at near-saturation.

    SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_V2_CTRL_REG_A, 0x001A);        //0x32   0x001A
    SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_HDR_ENABLE_REG,0x0103);        //0x0F High Dynamic Range enable,bit is set (R0x0F[1]=1), the sensor uses black level correction values from one green plane, which are applied to all colors.



    /* 0xA5  ͼ������  60  1-64  */
    SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_AGC_AEC_DESIRED_BIN_REG, CAMERA_AUTO_EXPOSURE_BRIGHTNESS);

}

/*!
  * @brief    ֡������
  *
  * @param    fps : ֡��
  *
  * @return
  *
  * @note     ֡����ʵ��ͨ�����ӿհ���ʵ�ֵģ� ������ʵ���֡����������֮��Ĺ�ϵ
  * @note     ֡��	    200    150    100    70.9    50    20    10
  * @note     ������    39     92     199    330     515   1450  3000
  *
  * @see      MT9V034_SetFrameRate(50);
  *
  * @date     2019/7/26 ������
  */
void MT9V034_SetFrameRate(unsigned char fps)
{
    float vertical = 0;
    if(fps > 200)
    {
        vertical = 2;
    }
    else if(fps >= 150)
    {
        vertical = -1.06 * fps + 251;
    }
    else if(fps > 100)
    {
        vertical = -2.14 * fps + 413;
    }
    else if(fps >= 71)
    {
        vertical = -4.5017 * fps + 649.17;
    }
    else if(fps > 50)
    {
        vertical = -8.8517 * fps + 957.58;
    }
    else if(fps > 20)
    {
        vertical = -31.167 * fps + 2073.3;
    }
    else
    {
        vertical = -155 * fps + 4550;
    }
    /* ������ ����֡��ʱ�� 2�C32288 */
    SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_VERTICAL_BLANKING, (unsigned short)vertical);
}

/*!
  * @brief    ����ͼ�񴰿ڴ�С
  *
  * @param    fps : ֡��
  *
  * @return   ��
  *
  * @note     һ��ʹ��Ĭ�ϼ���
  *
  * @see      MT9V034_SetFrameResolution(MT9V034_IMAGEH, MT9V034_IMAGEW);
  *
  * @date     2019/7/26 ������
  */
static void MT9V034_SetFrameResolution(unsigned short height,unsigned short width)
{
    unsigned short data = 0;

    if((height*4) <= MAX_IMAGE_HEIGHT)   //�ж����Ƿ�4��Ƶ
    {
        height *= 4;
        data |= MT9V034_READ_MODE_ROW_BIN_4;
    }
    else if((height*2)<=MAX_IMAGE_HEIGHT )  //�ж����Ƿ�2��Ƶ
    {
        height *= 2;
        data |= MT9V034_READ_MODE_ROW_BIN_2;
    }

    if((width*4)<=MAX_IMAGE_WIDTH )   //�ж����Ƿ�4��Ƶ
    {
        width *= 4;
        data |= MT9V034_READ_MODE_COL_BIN_4;
    }
    else if((width*2)<=MAX_IMAGE_WIDTH )   //�ж����Ƿ�2��Ƶ
    {
        width *= 2;
        data |= MT9V034_READ_MODE_COL_BIN_2;
    }

    //         ˮƽ��ת                     ��ֱ��ת
    data |= (MT9V034_READ_MODE_ROW_FLIP|MT9V034_READ_MODE_COLUMN_FLIP);  //��Ҫ��ת�Ŀ��Դ�ע��
    MTV_IICWriteReg16(MT9V034_READ_MODE, data);       //д�Ĵ����������з�Ƶ

    MTV_IICWriteReg16(MT9V034_WINDOW_WIDTH,  width);  //��ȡͼ�������  �ı�˴�Ҳ�ɸı�ͼ�������С�������ᶪʧ�ӽ�
    MTV_IICWriteReg16(MT9V034_WINDOW_HEIGHT, height); //��ȡͼ�������  �ı�˴�Ҳ�ɸı�ͼ�������С�������ᶪʧ�ӽ�

    MTV_IICWriteReg16(MT9V034_COLUMN_START, MT9V034_COLUMN_START_MIN);  //�п�ʼ
    MTV_IICWriteReg16(MT9V034_ROW_START, MT9V034_ROW_START_MIN);        //�п�ʼ

}

/*!
  * @brief    �����Զ��ع�ʱ��
  *
  * @param    enable �� 1 ʹ���Զ��ع�  �Զ��ع�ʱ��ͨ������ĺ궨���޸�
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      MT9V034_SetAutoExposure(CAMERA_AUTO_EXPOSURE);
  *
  * @date     2019/7/26 ������
  */
void MT9V034_SetAutoExposure(char enable)
{
    unsigned short reg =0;
    SCCB_RegRead(MT9V034_I2C_ADDR, MT9V034_AEC_AGC_ENABLE,&reg);
    if(enable)
    {
        /* �����Զ��ع��Զ����� */
        SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AEC_AGC_ENABLE, reg|MT9V034_AEC_ENABLE|MT9V034_AGC_ENABLE);
        /* ����ع�ʱ�� �޸���������޸ıȽϰ�ʱ��ͼ����������*/
        SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_EXPOSURE_REG, CAMERA_MAX_EXPOSURE_TIME);
        /* ��С�ع�ʱ�� �޸���������޸�����ǿ��ʱ��ͼ����������*/
        SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MIN_EXPOSURE_REG, CAMERA_MIN_EXPOSURE_TIME);
        /* ������� �������� ͼ��ƫ��������±��������ϸ�� ���ǿ��ܲ������ 0-60*/
        SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_GAIN_REG, 20);
        /* 0xB0  ����AEC/AGCֱ��ͼ������Ŀ,22560 ���44000  */
        SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AGC_AEC_PIXEL_COUNT_REG, 22560);

    }
    else
    {
        /* �ر��Զ��ع� */
        SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_AEC_AGC_ENABLE, reg&~(MT9V034_AEC_ENABLE|MT9V034_AGC_ENABLE));

        /* 0xAB  ���ģ������     64 */
        SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_GAIN_REG, 30);

        /* 0x0B �����ع�ʱ�� 0�C32765 */
        SCCB_RegWrite(MT9V034_I2C_ADDR,MT9V034_TOTAL_SHUTTER_WIDTH,CAMERA_EXPOSURE_TIME);

    }

}






/*********************************************************************
 *����ͷSCCB�ײ�����
 *
 ***********************************************************************/


/*!
  * @brief    SCCBд16λ�Ĵ���
  *
  * @param    reg �� �Ĵ���
  * @param    val �� д���ֵ
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      MTV_IICWriteReg16(0x20, 0x0003); //��MT9V034��0x20�Ĵ���д��0x0003
  *
  * @date     2019/7/26 ������
  */
void MTV_IICWriteReg16(unsigned char reg, unsigned short val)
{
	SCCB_RegWrite(MT9V034_I2C_ADDR, reg, val);
}


/*!
  * @brief    SCCB ��ʼ��
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     SCCB IO��LQ_MT9V034.h��ͨ���궨��ѡ��
  *
  * @see      SCCB_Init(); //��ʼ��SCCB  IO
  *
  * @date     2019/7/26 ������
  */
void SCCB_Init(void)
{
  PIN_InitConfig(MT9V034_SCL_PIN, PIN_MODE_OUTPUT, 1);
  PIN_InitConfig(MT9V034_SDA_PIN, PIN_MODE_OUTPUT, 1);
  IfxPort_setPinPadDriver(PIN_GetModule(MT9V034_SCL_PIN), PIN_GetIndex(MT9V034_SCL_PIN), IfxPort_PadDriver_cmosAutomotiveSpeed1);
  IfxPort_setPinPadDriver(PIN_GetModule(MT9V034_SDA_PIN), PIN_GetIndex(MT9V034_SDA_PIN), IfxPort_PadDriver_cmosAutomotiveSpeed1);
  MT9V034_SCL_High;
  MT9V034_SDA_High;
}



/*!
  * @brief    SCCB ��ʱ
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      SCCB_Wait();
  *
  * @date     2019/7/26 ������
  */
void SCCB_Wait(void)
{
	int i;
	for (i = 0; i < 500; ++i)
	{
		__asm("NOP"); /* delay */
	}
}

/*!
  * @brief    SCCB ��ʼʱ��
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      SCCB_Star();
  *
  * @date     2019/7/26 ������
  */
void SCCB_Star(void)
{
	MT9V034_SCL_Out;
	MT9V034_SDA_Out;
	SCCB_Wait();
	MT9V034_SDA_High;
	MT9V034_SCL_High;
	SCCB_Wait();
	MT9V034_SDA_Low;
	SCCB_Wait();
	MT9V034_SCL_Low;
	SCCB_Wait();
}

/*!
  * @brief    SCCB ֹͣʱ��
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      SCCB_Star();
  *
  * @date     2019/7/26 ������
  */
void SCCB_Stop(void)
{
	MT9V034_SCL_Out;
	MT9V034_SDA_Out;
	SCCB_Wait();
	MT9V034_SDA_Low;
	SCCB_Wait();
	MT9V034_SCL_High;
	SCCB_Wait();
	MT9V034_SDA_High;
	SCCB_Wait();
}

/*!
  * @brief    SCCB ����һ���ֽ�
  *
  * @param    Data  �������ֽ�
  *
  * @return   ack Ӧ���ź�
  *
  * @note     ��
  *
  * @see      SCCB_SendByte(0x01);
  *
  * @date     2019/7/26 ������
  */
unsigned char SCCB_SendByte(unsigned char Data)
{
  unsigned char i;
  unsigned char Ack;
  MT9V034_SDA_Out;
  for( i=0; i<8; i++)
  {
    if(Data & 0x80)
    {
    	MT9V034_SDA_High;
    }
    else
    {
    	MT9V034_SDA_Low;
    }
    Data <<= 1;
    SCCB_Wait();
    MT9V034_SCL_High;
    SCCB_Wait();
    MT9V034_SCL_Low;
    SCCB_Wait();
  }
  MT9V034_SDA_High;
  MT9V034_SDA_In;
  SCCB_Wait();

  MT9V034_SCL_High;
  SCCB_Wait();
  Ack = MT9V034_SDA_Data;
  MT9V034_SCL_Low;
  SCCB_Wait();
  return Ack;
}

/*!
  * @brief    SCCB ����һ���ֽ�
  *
  * @param    ��
  *
  * @return   ���յ����ֽ�
  *
  * @note     ��
  *
  * @see      unsigned char data = SCCB_ReadByte();
  *
  * @date     2019/7/26 ������
  */
unsigned char SCCB_ReadByte(void)
{
  unsigned char i;
  volatile unsigned char byte;
  MT9V034_SCL_Out;
  MT9V034_SDA_In; //ʹ������
  for( i=0; i<8; i++)
  {
	MT9V034_SCL_Low;
	SCCB_Wait();
	MT9V034_SCL_High;
	SCCB_Wait();
	byte = (byte<<1)|(MT9V034_SDA_Data & 1);
  }
  MT9V034_SCL_Low;
  MT9V034_SDA_Out;
  SCCB_Wait();
  return byte;
}

/*!
  * @brief    SCCB Ӧ��
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      SCCB_Ack();
  *
  * @date     2019/7/26 ������
  */
static void SCCB_Ack(void)
{
	MT9V034_SCL_Out;
	MT9V034_SDA_Out;
	MT9V034_SCL_Low;
	MT9V034_SDA_Low;
	SCCB_Wait();
	MT9V034_SCL_High;
	SCCB_Wait();
	MT9V034_SCL_Low;
	SCCB_Wait();
}

/*!
  * @brief    SCCB ��Ӧ��
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      SCCB_NAck();
  *
  * @date     2019/7/26 ������
  */
static void SCCB_NAck(void)
{
	MT9V034_SCL_Out;
	MT9V034_SDA_Out;
	MT9V034_SCL_Low;
	SCCB_Wait();
	MT9V034_SDA_High;
	SCCB_Wait();
	MT9V034_SCL_High;
	SCCB_Wait();
	MT9V034_SCL_Low;
	SCCB_Wait();
}

/*!
  * @brief    SCCB д�Ĵ���
  *
  * @param    Device  : ��ַ
  * @param    reg     : �Ĵ���
  * @param    Data    : ����
  *
  * @return   ack
  *
  * @note     ��
  *
  * @see      SCCB_RegWrite(MT9V034_I2C_ADDR, MT9V034_MAX_GAIN_REG, 30);
  *
  * @date     2019/7/26 ������
  */
void SCCB_RegWrite(unsigned char Device,unsigned char reg,unsigned short Data)
{
  unsigned char i;
  unsigned char Ack;

  for( i=0; i<3; i++)
  {
    SCCB_Star();
    Ack = SCCB_SendByte(Device);
    if( Ack == 1 )
    {
      continue;
    }

    Ack = SCCB_SendByte(reg);
    if( Ack == 1 )
    {
      continue;
    }

    SCCB_SendByte((unsigned char)(Data>>8));
    Ack = SCCB_SendByte((unsigned char)Data);
    if( Ack == 1 )
    {
      continue;
    }

    SCCB_Stop();
    if( Ack == 0 ) break;
  }
}

/*!
  * @brief    SCCB ���Ĵ���
  *
  * @param    Device  : ��ַ
  * @param    reg     : �Ĵ���
  * @param    Data    : ��������
  *
  * @return   ack
  *
  * @note     ��
  *
  * @see      SCCB_RegRead(MT9V034_I2C_ADDR>>1,MT9V034_CHIP_VERSION,&data)
  *
  * @date     2019/7/26 ������
  */
unsigned char SCCB_RegRead(unsigned char Device,unsigned char reg,unsigned short *Data)
{
  unsigned char Ack = 0;
  Device = Device<<1;
  SCCB_Star();
  Ack += SCCB_SendByte(Device);

  Ack += SCCB_SendByte(reg);

  SCCB_Star();
  Ack += SCCB_SendByte(Device + 1);

  *Data = SCCB_ReadByte();
  SCCB_Ack();
  *Data = *Data<<8;

  *Data += SCCB_ReadByte();
  SCCB_NAck();

  SCCB_Stop();

  return  Ack;
}



