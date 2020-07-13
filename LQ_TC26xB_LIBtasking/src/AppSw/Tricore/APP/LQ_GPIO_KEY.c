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

#include <LQ_GPIO.h>
#include <LQ_GPIO_KEY.h>
#include <LQ_GPIO_LED.h>
#include <LQ_STM.h>
#include <LQ_TFT18.h>


/*************************************************************************
*  �������ƣ�void GPIO_KEY_Init(void)
*  ����˵����GPIO��ʼ������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��
*************************************************************************/
void GPIO_KEY_Init(void)
{
	  // ��ʼ��,����ڣ��ߵ�ƽ
	  PIN_InitConfig(KEY0p, PIN_MODE_INPUT, 1);
	  PIN_InitConfig(KEY1p, PIN_MODE_INPUT, 1);
	  PIN_InitConfig(KEY2p, PIN_MODE_INPUT, 1);
	  PIN_InitConfig(DSW0p, PIN_MODE_INPUT, 1);
	  PIN_InitConfig(DSW1p, PIN_MODE_INPUT, 1);
}

#pragma warning 544         // ���ξ���

/*************************************************************************
*  �������ƣ�unsigned char KEY_Read(KEYn_e KEYno)
*  ����˵������ȡ����״̬
*  ����˵����KEYn_e KEYno�������
*  �������أ�����״̬��0/1
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��
*************************************************************************/
unsigned char KEY_Read(KEYn_e KEYno)
{
    switch(KEYno)
    {
      case KEY0:
        return PIN_Read(KEY0p);//ĸ���ϰ���0
      break;

      case KEY1:
        return PIN_Read(KEY1p);//ĸ���ϰ���1
      break;

      case KEY2:
        return PIN_Read(KEY2p);//ĸ���ϰ���2
      break;

      case DSW0:
        return PIN_Read(DSW0p);//ĸ���ϲ��뿪��0
      break;

      case DSW1:
        return PIN_Read(DSW1p);//ĸ���ϲ��뿪��1
      break;
      default:
        return 0XFF;
    }
    return 0;
}
#pragma warning default     // �򿪾���


/*************************************************************************
*  �������ƣ�unsigned char KEY_Read_All(void)
*  ����˵������ȡȫ������״̬
*  ����˵������
*  �������أ��������״̬��0--7����״̬
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע����ȡ��������״̬��������ϼ�ʹ��
*************************************************************************/
unsigned char KEY_Read_All(void)
{
   unsigned char tm=0;

   tm = (PIN_Read(KEY0p)|(PIN_Read(KEY1p)<<1)|(PIN_Read(KEY2p)<<2));//��ȡ��������״̬������
   if(tm==0x07)
	{
	   return 0;
	}
   //while(tm == (PIN_Read(KEY0p)|(PIN_Read(KEY1p)<<1)|(PIN_Read(KEY2p)<<2)));//�ȴ������ͷ�

   return  (~tm)&0X07;
}


/*************************************************************************
*  �������ƣ�void Test_GPIO_KEY(void)
*  ����˵�������Գ���
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע������KEY0/1/2����� LEDȫ��
*************************************************************************/
void Test_GPIO_KEY(void)
 {
	unsigned char k=0xFF;

	GPIO_KEY_Init();
 	while(1)
 	{
 		k= KEY_Read(KEY0)& KEY_Read(KEY1)& KEY_Read(KEY2);

 		if(k==0) LED_Ctrl(LEDALL, ON); //����KEY0/1/2����� LEDȫ��
 		else     LED_Ctrl(LEDALL, OFF);//�ɿ�KEY0/1/2����� LEDȫ��

 		delayms(100);                //��ʱ�ȴ�
 	}
 }


/*************************************************************************
*  �������ƣ�void Test_ComKEY_Tft(void)
*  ����˵����������ϰ������ڲ�����ʾ
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע������״̬������ʾ
*************************************************************************/
void Test_ComKEY_Tft(void)
 {
	unsigned char k=0xFF;

	TFTSPI_Init(0);        //LCD��ʼ��  0:����  1������
	TFTSPI_CLS(u16BLUE);   //��ɫ��Ļ
	TFTSPI_Show_Logo(0,37);//��ʾ����LOGO
	TFTSPI_P16x16Str(0,0,"�����������ܿƼ�",u16RED,u16BLUE);		//�ַ�����ʾ
	TFTSPI_P8X16Str(0,1,"Long Qiu i.s.t.",u16WHITE,u16BLACK);		//�ַ�����ʾ
	delayms(1000);              //��ʱ�ȴ�
	TFTSPI_Init(1);        //LCD��ʼ��  0:����  1������
	TFTSPI_CLS(u16BLACK);   //��ɫ��Ļ
	TFTSPI_P8X16Str(0,1,"KEY Pressed:  ",u16WHITE,u16BLACK);		//�ַ�����ʾ

	GPIO_KEY_Init();
 	while(1)
 	{

 		k= KEY_Read_All();
 		switch(k)
 		{
 		  case NOKEYDOWN:
 			 //TFTSPI_P8X16Str(35,3,"NO key!  ",u16WHITE,u16BLACK);		//�ַ�����ʾ
 		  break;
 		  case KEY0DOWN:
 		 	 TFTSPI_P8X16Str(35,3,"KEY0!    ",u16WHITE,u16BLACK);		//�ַ�����ʾ
 		  break;
 		  case KEY1DOWN:
 		 	 TFTSPI_P8X16Str(35,3,"KEY1!    ",u16WHITE,u16BLACK);		//�ַ�����ʾ
 		  break;
 		  case KEY2DOWN:
 		 	 TFTSPI_P8X16Str(35,3,"KEY2!    ",u16WHITE,u16BLACK);		//�ַ�����ʾ
 		  break;
 		  case KEY01DOWN:
 			 TFTSPI_P8X16Str(35,3,"KEY0&1!  ",u16WHITE,u16BLACK);		//�ַ�����ʾ
 		  break;
 		  case KEY02DOWN:
 		 	 TFTSPI_P8X16Str(35,3,"KEY0&2!  ",u16WHITE,u16BLACK);		//�ַ�����ʾ
 		  break;
 		  case KEY12DOWN:
 		 	 TFTSPI_P8X16Str(35,3,"KEY1&2!  ",u16WHITE,u16BLACK);		//�ַ�����ʾ
 		  break;
 		  case KEY012DOWN:
 		 	 TFTSPI_P8X16Str(35,3,"KEY0&1&2!",u16WHITE,u16BLACK);		//�ַ�����ʾ
 		  break;
 		  default:
 		  break;
 		 }
 		LED_Ctrl(LED0,RVS);        //�ĸ�LEDͬʱ��˸
 		delayms(100);              //��ʱ�ȴ�
 	}
 }



