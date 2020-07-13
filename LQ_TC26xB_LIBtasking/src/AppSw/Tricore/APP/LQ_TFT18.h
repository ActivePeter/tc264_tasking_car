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
#ifndef __LQ_SGP18T_TFTSPI_H__
#define __LQ_SGP18T_TFTSPI_H__	

#include "stdint.h"

/*******************�ӿڶ���******************************/

#define APP_LCD_WIDTH  LCD_WIDTH//Һ�������
#define APP_LCD_HEIGHT LCD_HEIGHT//Һ�����߶�

#define TFT18W        162
#define TFT18H        132

#define u16RED		0xf800
#define u16GREEN	0x07e0
#define u16BLUE	    0x001f
#define u16PURPLE	0xf81f
#define u16YELLOW	0xffe0
#define u16CYAN	    0x07ff 		//����ɫ
#define u16ORANGE	0xfc08
#define u16BLACK	0x0000
#define u16WHITE	0xffff

/*!
  * @brief    TFT18��ʼ��
  *
  * @param    type �� 0:����  1������
  *
  * @return   ��
  *
  * @note     ����޸Ĺܽ� ��Ҫ�޸ĳ�ʼ���Ĺܽ�
  *
  * @see      TFTSPI_Init(1);
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Init(uint8_t type);

/*!
  * @brief    д����
  *
  * @param    cmd ������
  *
  * @return   ��
  *
  * @note     �ڲ�����
  *
  * @see      TFTSPI_Write_Cmd(0xb7); //LCD Driveing control
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Write_Cmd(uint8_t cmd);

/*!
  * @brief    д�ֽ�
  *
  * @param    dat ������
  *
  * @return   ��
  *
  * @note     �ڲ�����
  *
  * @see      TFTSPI_Write_Byte(0x00);	//CRL=0
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Write_Byte(uint8_t dat);


/*!
  * @brief    д����
  *
  * @param    dat ������
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      TFTSPI_Write_Word(0xFFFF);
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Write_Word(uint16_t dat);


/*!
  * @brief    ���¶�λ������Ϣλ��
  *
  * @param    xs ����ʼx
  * @param    ys ����ʼy
  * @param    xe ������x
  * @param    ys ������y
  *
  * @return   ��
  *
  * @note     �ڲ�����
  *
  * @see      TFTSPI_Set_Pos(10, 20, 30, 40);
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Set_Pos(uint8_t xs,uint8_t ys,uint8_t xe,uint8_t ye);

/*!
  * @brief    ���ָ������
  *
  * @param    xs ����ʼx
  * @param    ys ����ʼy
  * @param    xe ������x
  * @param    ys ������y
  * @param    color ��������ɫ
  *
  * @return   ��
  *
  * @note     ��ʼ����ֹ������(0-127)��������(0-159),��ʾ��ɫuint16
  *
  * @see      TFTSPI_Fill_Area(10, 20, 30, 40, u16YELLOW);
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Fill_Area(uint8_t xs,uint8_t ys,uint8_t xe,uint8_t ye,uint16_t color);

/*!
  * @brief    ȫ����ʾ��ɫ����
  *
  * @param    color ��������ɫ
  *
  * @return   ��
  *
  * @note     ��ʼ����ֹ������(0-127)��������(0-159),��ʾ��ɫuint16
  *
  * @see      TFTSPI_CLS(u16YELLOW);
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_CLS(uint16_t color);


/*!
  * @brief    ����������
  *
  * @param    xs ����ʼx
  * @param    ys ����ʼy
  * @param    xe ������x
  * @param    ys ������y
  * @param    color_dat ��������ɫ
  *
  * @return   ��
  *
  * @note     ��ʼ����ֹ������(0-127)��������(0-159),��ʾ��ɫuint16
  *
  * @see      TFTSPI_Draw_Part(10, 20, 30, 40, u16YELLOW);
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Draw_Part(uint8_t xs,uint8_t ys,uint8_t xe,uint8_t ye,uint16_t color_dat);

/*!
  * @brief    �����α߿�
  *
  * @param    xs ����ʼx
  * @param    ys ����ʼy
  * @param    xe ������x
  * @param    ys ������y
  * @param    color_dat ����ɫ
  *
  * @return   ��
  *
  * @note     ��ʼ����ֹ������(0-127)��������(0-159),��ʾ��ɫuint16
  *
  * @see      TFTSPI_Draw_Rectangle(10, 20, 30, 40, u16YELLOW);
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Draw_Rectangle(uint8_t xs,uint8_t ys,uint8_t xe,uint8_t ye,uint16_t color_dat);

/*!
  * @brief    ��Բ
  *
  * @param    x ��Բ��x   (0-127)
  * @param    y ��Բ��y   (0-159)
  * @param    r ���뾶    (0-128)
  * @param    color_dat ����ɫ
  *
  * @return   ��
  *
  * @note     Բ�����겻Ҫ������Ļ��Χ
  *
  * @see      TFTSPI_Draw_Circle(50, 50, 30, u16YELLOW);
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Draw_Circle(uint8_t x,uint8_t y,uint8_t r,uint16_t color_dat);

/*!
  * @brief    ����
  *
  * @param    xs ����ʼx
  * @param    ys ����ʼy
  * @param    xe ������x
  * @param    ys ������y
  * @param    color_dat ����ɫ
  *
  * @return   ��
  *
  * @note     ��ʼ����ֹ������(0-127)��������(0-159),��ʾ��ɫuint16
  *
  * @see      TFTSPI_Draw_Line(10, 20, 30, 40, u16YELLOW);
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Draw_Line(uint8_t xs,uint8_t ys,uint8_t xe,uint8_t ye,uint16_t color_dat);

/*!
  * @brief    ����
  *
  * @param    x ��x
  * @param    y ��y
  * @param    color_dat ����ɫ
  *
  * @return   ��
  *
  * @note     ��ʼ����ֹ������(0-127)��������(0-159),��ʾ��ɫuint16
  *
  * @see      TFTSPI_Draw_Dot(10, 20, u16YELLOW);
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Draw_Dot(uint8_t x,uint8_t y,uint16_t color_dat);

/*!
  * @brief    ��ʾͼƬ
  *
  * @param    xs ����ʼx
  * @param    ys ����ʼy
  * @param    xe ������x
  * @param    ys ������y
  * @param    ppic ��ͼƬ����
  *
  * @return   ��
  *
  * @note     ��ʼ����ֹ������(0-127)��������(0-159),��ʾ��ɫuint16
  *
  * @see
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Show_Pic(uint8_t xs,uint8_t ys,uint8_t xe,uint8_t ye,const char *ppic);

/*!
  * @brief    Һ���ַ������(6*8����)
  *
  * @param    x: 0 - 20 (��)
  * @param    y: 0 - 19 (��)
  * @param    word_color: ������ɫ
  * @param    back_color: ������ɫ
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      TFTSPI_P6X8Str(1, 1, "123456", u16YELLOW, u16RED);
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_P6X8Str(uint8_t x, uint8_t y, char *s_dat,uint16_t word_color,uint16_t back_color);

/*!
  * @brief    Һ���ַ������(8*8����)
  *
  * @param    x:0 - 15 (��)
  * @param    y:0 - 19 (��)
  * @param    word_color: ������ɫ
  * @param    back_color: ������ɫ
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      TFTSPI_P8X8Str(1, 1, "123456", u16YELLOW, u16RED);
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_P8X8Str(uint8_t x, uint8_t y, char *s_dat,uint16_t word_color,uint16_t back_color);

/*!
  * @brief    Һ���ַ������(8*16����)
  *
  * @param    x: x: 0 -15   (��)
  * @param    y: y: 0 -9  	 (��)
  * @param    word_color: ������ɫ
  * @param    back_color: ������ɫ
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      TFTSPI_P8X16Str(1, 1, "123456", u16YELLOW, u16RED);
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_P8X16Str(uint8_t x, uint8_t y, char *s_dat,uint16_t word_color,uint16_t back_color);

/*!
  * @brief    Һ�������ַ������(8*16����)
  *
  * @param    x: 0 - 7	(��)
  * @param    y: 0 -12	(��)
  * @param    word_color: ������ɫ
  * @param    back_color: ������ɫ
  *
  * @return   ��
  *
  * @note     ����ֻ�����ֿ���� �ֿ�û�е���Ҫ�������
  *
  * @see      TFTSPI_P16x12Str(1, 1, "123456", u16YELLOW, u16RED);
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_P16x12Str(uint8_t x, uint8_t y, char *s_dat,uint16_t word_color,uint16_t back_color);

/*!
  * @brief    Һ�������ַ������(16*16����)
  *
  * @param    x: 0 - 7	(��)
  * @param    y: 0 - 9	(��)
  * @param    word_color: ������ɫ
  * @param    back_color: ������ɫ
  *
  * @return   ��
  *
  * @note     ����ֻ�����ֿ���� �ֿ�û�е���Ҫ�������
  *
  * @see      TFTSPI_P16x16Str(1, 1, "123456", u16YELLOW, u16RED);
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_P16x16Str(uint8_t x, uint8_t y, char *s_dat,uint16_t word_color,uint16_t back_color);


/*!
  * @brief    ��ʾͼƬ
  *
  * @param    xs ����ʼx
  * @param    ys ����ʼy
  * @param    xe ������x
  * @param    ys ������y
  * @param    ppic ��ͼƬ����
  *
  * @return   ��
  *
  * @note     ��ʼ����ֹ������(0-127)��������(0-159),��ʾ��ɫuint16
  *
  * @see
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Show_Pic2(uint8_t xs,uint8_t ys,uint8_t w,uint8_t h,const char *ppic) ;


/*!
  * @brief    ��ʾ���ͼ��
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      TFTSPI_Show_Battery_Icon();
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Show_Battery_Icon(void);

/*!
  * @brief    ��ʾ��ǩ
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      TFTSPI_Show_Title();
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Show_Title(void);

/*!
  * @brief    ��ʾ����LOG
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      TFTSPI_Show_Logo();
  *
  * @date     2019/6/13 ������
  */
void TFTSPI_Show_Logo(uint8_t xs,uint8_t ys);

/*!
  * @brief    TFT18�� uint8_t �Ҷ�������ʾ
  *
  * @param    high_start �� ��ʾͼ��ʼλ��
  * @param    wide_start �� ��ʾͼ��ʼλ��
  * @param    high �� ��ʾͼ��߶�
  * @param    wide �� ��ʾͼ����
  * @param    Pixle�� ��ʾͼ�����ݵ�ַ
  *
  * @return   ��
  *
  * @note     ע�� ��Ļ����Ϊ ��0��0��
  *
  * @see
  *
  * @date     2019/12/3 ���ڶ�
  */
void TFTSPI_Road(uint8_t high_start, uint8_t wide_start, uint8_t high, uint8_t wide, uint8_t *Pixle);

/*!
  * @brief    TFT18�� uint8_t ��ֵ��������ʾ
  *
  * @param    high_start �� ��ʾͼ��ʼλ��
  * @param    wide_start �� ��ʾͼ��ʼλ��
  * @param    high �� ��ʾͼ��߶�
  * @param    wide �� ��ʾͼ����
  * @param    Pixle�� ��ʾͼ�����ݵ�ַ
  *
  * @return   ��
  *
  * @note     ע�� ��Ļ����Ϊ ��0��0��
  *
  * @see
  *
  * @date     2019/12/3 ���ڶ�
  */
void TFTSPI_BinRoad(uint8_t high_start, uint8_t wide_start, uint8_t high, uint8_t wide, uint8_t *Pixle);


void Test_TFT18(void);


#endif /*SGP18T_ILI9163B.h*/
