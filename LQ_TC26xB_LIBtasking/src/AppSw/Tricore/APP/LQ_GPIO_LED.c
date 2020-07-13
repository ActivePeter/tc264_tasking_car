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
#include <LQ_GPIO_LED.h>
#include <LQ_STM.h>

/*************************************************************************
*  �������ƣ�void LED_Init(void)
*  ����˵����GPIO��ʼ������ LED������P10.6��P10.5��P15.4��P15.6��ʼ��
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��
*************************************************************************/
void GPIO_LED_Init(void)
{
	  // ��ʼ��,����ڣ��ߵ�ƽ
	  PIN_InitConfig(LED0p, PIN_MODE_OUTPUT, 0);
	  PIN_InitConfig(LED1p, PIN_MODE_OUTPUT, 0);
	  PIN_InitConfig(LED2p, PIN_MODE_OUTPUT, 0);
	  PIN_InitConfig(LED3p, PIN_MODE_OUTPUT, 0);
}

/*************************************************************************
*  �������ƣ�void LED_Ctrl(LEDn_e LEDno, LEDs_e sta)
*  ����˵����LED����
*  ����˵����LEDn_e LEDno���,LEDs_e sta״̬��/��/��ת
*  �������أ�����״̬��0/1
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��
*************************************************************************/
void LED_Ctrl(LEDn_e LEDno, LEDs_e sta)
{
    switch(LEDno)
    {
    case LED0:
      if(sta==ON)        PIN_Write(LED0p,0);
      else if(sta==OFF) PIN_Write(LED0p,1);
      else if(sta==RVS) PIN_Reverse(LED0p);
    break;

    case LED1:
      if(sta==ON)        PIN_Write(LED1p,0);
      else if(sta==OFF) PIN_Write(LED1p,1);
      else if(sta==RVS) PIN_Reverse(LED1p);
    break;

    case LED2:
      if(sta==ON)        PIN_Write(LED2p,0);
      else if(sta==OFF) PIN_Write(LED2p,1);
      else if(sta==RVS) PIN_Reverse(LED2p);
    break;

    case LED3:
      if(sta==ON)        PIN_Write(LED3p,0);
      else if(sta==OFF) PIN_Write(LED3p,1);
      else if(sta==RVS) PIN_Reverse(LED3p);
    break;
    case LEDALL:
      if(sta==ON)
      {
    	  PIN_Write(LED0p,0);
    	  PIN_Write(LED1p,0);
    	  PIN_Write(LED2p,0);
    	  PIN_Write(LED3p,0);
      }
      else if(sta==OFF)
      {
    	  PIN_Write(LED0p,1);
    	  PIN_Write(LED1p,1);
    	  PIN_Write(LED2p,1);
    	  PIN_Write(LED3p,1);
      }
      else if(sta==RVS)
      {
    	  PIN_Reverse(LED0p);
    	  PIN_Reverse(LED1p);
    	  PIN_Reverse(LED2p);
    	  PIN_Reverse(LED3p);
      }
    break;
    default:
    break;
    }
}
/*************************************************************************
*  �������ƣ�void Test_GPIO_LED(void)
*  ����˵�������Գ���
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע�����İ��ϵ�LED����˸--�ĸ�LEDͬʱ��˸
*************************************************************************/
void Test_GPIO_LED(void)
 {
	GPIO_LED_Init();
 	while(1)
 	{
 		LED_Ctrl(LEDALL,RVS);        //�ĸ�LEDͬʱ��˸
 		delayms(100);                //��ʱ�ȴ�
 	}
 }
