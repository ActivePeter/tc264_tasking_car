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
*  ��    ע��TC264ֻ��15���̶���GPIO֧���ⲿ�ж� ��15���̶�GPIO��Ϊ4�飬ÿ��ֻ��ѡ������һ����Ϊ�ⲿ�жϹܽ�ʹ��
*           0��P15_4  P33_7  P15_5                             1��P14_3  P15_8
*           2��P10_2  P02_1  P00_4  P20_0  P11_10              3��P10_3  P14_1  P02_0  P20_9  P15_1
*			�ⲿ�жϷ�������LQ_GPIO.c��  �ж����ȼ�������LQ_GPIO.h�� ���������޸�
*
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

#include <IfxCpu.h>
#include <IfxPort_reg.h>
#include <LQ_CAMERA.h>
#include <LQ_DMA.h>
#include <LQ_GPIO.h>
#include <LQ_GPIO_LED.h>
#include <LQ_TFT18.h>
#include <LQ_UART.h>
#include <LQ_STM.h>
#include <Platform_Types.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/** ͼ��ԭʼ���ݴ�� */
volatile unsigned char Image_Data[IMAGEH][IMAGEW];

/** ѹ����֮�����ڴ����Ļ��ʾ����  */
unsigned char Image_Use[LCDH][LCDW];

/** ��ֵ��������OLED��ʾ������ */
unsigned char Pixle[LCDH][LCDW];


/*!
  * @brief    ����ͷ��������
  *
  * @param
  *
  * @return
  *
  * @note     ����MT9V034  ע����Ҫʹ��  ������Ű棨��ɫ��ת����
  *
  * @example
  *
  * @date     2019/10/22 ���ڶ�
  */
void Test_CAMERA(void)
{

#ifdef USEOLED
    OLED_Init();
    OLED_CLS();                   //LCD����
#else
    TFTSPI_Init(0);               //TFT1.8��ʼ��0:����  1������
    TFTSPI_CLS(u16BLUE);          //����
#endif

    /* ����ͷ��ʼ�� */
    CAMERA_Init(50);


    while(1)
    {
        if(Camera_Flag == 2)
        {
        	/*  TFT��ʾ���ϱ���λ���ȽϺ�ʱ�ȽϾã� �ر�����ͷ���ж�  ��ֹһ֡ͼ��δ��ʾ������ͼ������ͱ���һ֡���ݸ��� */
        	PIN_ExtiEnable(P15_1, FALSE);

            /* �������ͷ�ɼ���ɱ�־λ */
        	Camera_Flag = 0;

            /* ��ȡ����ʹ�õ����� */
            Get_Use_Image();

#ifdef USEOLED
            /* ��ֵ�� */
            Get_01_Value(0);

            // OLED��̬��ʾ����ͷͼ��
            OLED_Road(LCDH, LCDW, (unsigned char *)Pixle);
#else

#if 1       //��ʾԭʼͼ��
            //TFT1.8��̬��ʾ����ͷͼ��
            TFTSPI_Road(0, 0, LCDH, LCDW, (unsigned char *)Image_Use);

#else       //��ʾ��ֵ��ͼ��

            /* ��ֵ�� */
			Get_01_Value(0);

			// ��ʾ����ͷͼ��
			TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *)Pixle);
#endif
#endif

            //�ϱ����ݸ���λ�� �����ٶȱȽ��� ע����λ��ͼ��������Ϊ120*188
            CAMERA_Reprot();

            /* �������ж� */
            PIN_ExtiEnable(P15_1, TRUE);

            LED_Ctrl(LED0,RVS);

        }
    }
}



/*!
  * @brief    �����ϱ���λ��
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��λ����֡ͷ������������ 
  *
  * @see      CAMERA_Reprot();
  *
  * @date     2019/9/24 ���ڶ�
  */
void CAMERA_Reprot(void)
{
	int j,i;

    UART_PutChar(UART0, 0xfe);  //֡ͷ
    UART_PutChar(UART0, 0xef);  //֡ͷ
    
    for(i = 0; i < IMAGEH; i++)
    {
        for(j = 0; j < IMAGEW; j++)
        {
            if(Image_Data[i][j] == 0xfe )  //��ֹ������֡β
            {
            	Image_Data[i][j] = 0xff;
            }
            UART_PutChar(UART0, Image_Data[i][j]); //��������
            
        }
    }
    UART_PutChar(UART0, 0xef);  //֡β
    UART_PutChar(UART0, 0xfe);  //֡β

}

/*!
  * @brief    ����ͷ��ʼ��
  *
  * @param    fps:  ֡�� 
  *
  * @return   ��
  *
  * @note     ����ͷ��һЩ��������LQ_MT9V034.c�еĺ궨�����޸�
  *
  * @see      CAMERA_Init(50);   //��ʼ��MT9V034  50֡ ע��ʹ�ð�ɫ������Ű�ת����
  *
  * @date     2019/10/22 ���ڶ�
  */
void CAMERA_Init(unsigned char fps)
{
	//�ر�CPU���ж�
	IfxCpu_disableInterrupts();
    
    /* ��ʼ������ͷ ����IO */
    PIN_InitConfig(P02_0, PIN_MODE_INPUT_PULLDOWN, 0);
    PIN_InitConfig(P02_1, PIN_MODE_INPUT_PULLDOWN, 0);
    PIN_InitConfig(P02_2, PIN_MODE_INPUT_PULLDOWN, 0);
    PIN_InitConfig(P02_3, PIN_MODE_INPUT_PULLDOWN, 0);
    PIN_InitConfig(P02_4, PIN_MODE_INPUT_PULLDOWN, 0);
    PIN_InitConfig(P02_5, PIN_MODE_INPUT_PULLDOWN, 0);
    PIN_InitConfig(P02_6, PIN_MODE_INPUT_PULLDOWN, 0);
    PIN_InitConfig(P02_7, PIN_MODE_INPUT_PULLDOWN, 0);

    MT9V034_Init(fps);
 
    /* DMA ����Դ��ʼ�� */
    PIN_Exti(P00_4, PIN_IRQ_MODE_FALLING);

    /* DMA ��ʼ�� */
    DMA_CameraInitConfig((unsigned long)(&MODULE_P02.IN.U), (unsigned long)Image_Data, PIN_INT2_PRIORITY);
    
    /* ʹ�ܳ��ж� */
    PIN_Exti(P15_1, PIN_IRQ_MODE_FALLING);

    /* ��ʼ����� ���ж� */
	IfxCpu_enableInterrupts();
    
}

/**
  * @brief    ��ȡ��Ҫʹ�õ�ͼ������
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      Get_Use_Image();
  *
  * @date     2019/6/25 ���ڶ�
  */
void Get_Use_Image(void)
{ 
#ifdef LQMT9V034    //ʹ������
    
    uint8_t div_h, div_w;
    uint32_t temp_h = 0;
    uint32_t temp_w = 0;
    uint32_t row_start = 0;
    uint32_t lin_start = 0;
    
	div_h = IMAGEH/LCDH;
	div_w = IMAGEW/LCDW;
#pragma warning 549         // ���ξ���
    /* ������ȡͼ�� */
    if(LCDH * div_h != IMAGEH)
    {
        row_start = (IMAGEH - LCDH * div_h)/2;
        temp_h = row_start;
    }
    if(LCDW * div_w != IMAGEW)
    {
        lin_start = (IMAGEW - LCDW * div_w)/2;       
    }
    for(int i = 0; i < LCDH; i++)
    {
        temp_w = lin_start;
        for(int j = 0; j < LCDW; j++)
        {  
            Image_Use[i][j] = Image_Data[temp_h][temp_w];
            temp_w += div_w;
        }
        temp_h += div_h;
    }
#endif
#pragma warning 549         // �򿪾���
}

/*!
  * @brief    ��ֵ��
  *
  * @param    mode  ��  0��ʹ�ô����ֵ    1��ʹ��ƽ����ֵ
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      Get_01_Value(0); //ʹ�ô�򷨶�ֵ��
  *
  * @date     2019/6/25 ���ڶ�
  */
void Get_01_Value(unsigned char mode)
{
  int i = 0,j = 0;
  int Threshold;
  unsigned long  tv=0;
  char txt[16];

  if(mode)
  {
      //�ۼ�
      for(i = 0; i <LCDH; i++)
      {    
          for(j = 0; j <LCDW; j++)
          {                            
              tv+=Image_Use[i][j];   //�ۼ�  
          } 
      }
      Threshold=tv/LCDH/LCDW;        //��ƽ��ֵ,����Խ��ԽС��ȫ��Լ35��������ĻԼ160��һ������´�Լ100
      Threshold=Threshold*7/10+10;   //�˴���ֵ���ã����ݻ����Ĺ������趨 
  }
  else
  {
      Threshold = GetOSTU(Image_Use);//�����ֵ
//      Threshold = (unsigned char)(Threshold * 0.5) + 70;
  } 
#ifdef USEOLED
  sprintf(txt,"%03d",Threshold);//ǰ��Ϊ�����õ���ֵ������Ϊƽ��ֵ  
  OLED_P6x8Str(80,0,txt);
#else
  sprintf(txt,"%03d",Threshold);//ǰ��Ϊ�����õ���ֵ������Ϊƽ��ֵ  
  TFTSPI_P8X8Str(0,15, txt,u16RED,u16BLUE);
#endif
  
  for(i = 0; i < LCDH; i++)
  {
    for(j = 0; j < LCDW; j++)
    {                                
      if(Image_Use[i][j] >Threshold) //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����    
        Pixle[i][j] =1;        
      else                                        
        Pixle[i][j] =0;
    }    
  }
}





/*!
  * @brief    �������ֵ��С 
  *
  * @param    tmImage �� ͼ������
  *
  * @return   ��ֵ
  *
  * @note     �ο���https://blog.csdn.net/zyzhangyue/article/details/45841255
  * @note     https://www.cnblogs.com/moon1992/p/5092726.html
  * @note     https://www.cnblogs.com/zhonghuasong/p/7250540.html     
  * @note     Ostu������������������ͨ��ͳ������ͼ���ֱ��ͼ������ʵ��ȫ����ֵT���Զ�ѡȡ�����㷨����Ϊ��
  * @note     1) �ȼ���ͼ���ֱ��ͼ������ͼ�����е����ص㰴��0~255��256��bin��ͳ������ÿ��bin�����ص�����
  * @note     2) ��һ��ֱ��ͼ��Ҳ����ÿ��bin�����ص����������ܵ����ص�
  * @note     3) i��ʾ�������ֵ��Ҳ��һ���Ҷȼ�����0��ʼ����	1
  * @note     4) ͨ����һ����ֱ��ͼ��ͳ��0~i �Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ���ǰ������) ��ռ����ͼ��ı���w0����ͳ��ǰ�����ص�ƽ���Ҷ�u0��ͳ��i~255�Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ�����������) ��ռ����ͼ��ı���w1����ͳ�Ʊ������ص�ƽ���Ҷ�u1��
  * @note     5) ����ǰ�����غͱ������صķ��� g = w0*w1*(u0-u1) (u0-u1)
  * @note     6) i++��ת��4)��ֱ��iΪ256ʱ��������
  * @note     7) �����g��Ӧ��iֵ��Ϊͼ���ȫ����ֵ
  * @note     ȱ��:OSTU�㷨�ڴ�����ղ����ȵ�ͼ���ʱ��Ч�������Բ��ã���Ϊ���õ���ȫ��������Ϣ��
  * @note     ������ղ�����  https://blog.csdn.net/kk55guang2/article/details/78475414
  * @note     https://blog.csdn.net/kk55guang2/article/details/78490069
  * @note     https://wenku.baidu.com/view/84e5eb271a37f111f0855b2d.html
  *
  * @see      GetOSTU(Image_Use);//�����ֵ
  *
  * @date     2019/6/25 ���ڶ�
  */
int GetOSTU(unsigned char tmImage[LCDH][LCDW])
{ 
	signed short i,j;
	unsigned long Amount = 0;
	unsigned long PixelBack = 0;
	unsigned long PixelIntegralBack = 0;
	unsigned long PixelIntegral = 0;
	signed long PixelIntegralFore = 0;
	signed long PixelFore = 0;
	float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��; 
	signed short MinValue, MaxValue;
	signed short Threshold = 0;
	unsigned char HistoGram[256];              //
	
	for (j = 0; j < 256; j++)  HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ 
	
	for (j = 0; j < LCDH; j++) 
	{ 
		for (i = 0; i < LCDW; i++) 
		{ 
			HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
		} 
	} 
	
	for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //��ȡ��С�Ҷȵ�ֵ
	for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //��ȡ���Ҷȵ�ֵ
	
	if (MaxValue == MinValue)     return MaxValue;         // ͼ����ֻ��һ����ɫ    
	if (MinValue + 1 == MaxValue)  return MinValue;        // ͼ����ֻ�ж�����ɫ
	
	for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  ��������
	
	PixelIntegral = 0;
	for (j = MinValue; j <= MaxValue; j++)
	{
		PixelIntegral += HistoGram[j] * j;//�Ҷ�ֵ����
	}
	SigmaB = -1;
	for (j = MinValue; j < MaxValue; j++)
	{
		PixelBack = PixelBack + HistoGram[j];   //ǰ�����ص���
		PixelFore = Amount - PixelBack;         //�������ص���
		OmegaBack = (float)PixelBack / Amount;//ǰ�����ذٷֱ�
		OmegaFore = (float)PixelFore / Amount;//�������ذٷֱ�
		PixelIntegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
		PixelIntegralFore = PixelIntegral - PixelIntegralBack;//�����Ҷ�ֵ
		MicroBack = (float)PixelIntegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
		MicroFore = (float)PixelIntegralFore / PixelFore;   //�����ҶȰٷֱ�
		Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//������䷽��
		if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
		{
			SigmaB = Sigma;
			Threshold = j;
		}
	}
	return Threshold;                        //���������ֵ;
} 



