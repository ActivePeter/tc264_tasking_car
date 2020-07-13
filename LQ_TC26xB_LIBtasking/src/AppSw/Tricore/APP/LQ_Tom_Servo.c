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

#include <IfxGtm_PinMap.h>
#include <LQ_GPIO_KEY.h>
#include <LQ_GPIO_LED.h>
#include <LQ_GTM.h>
#include <LQ_STM.h>
#include <LQ_TFT18.h>
#include <LQ_UART.h>
#include <stdio.h>


#define TOMSERVO1         IfxGtm_TOM0_0_TOUT32_P33_10_OUT
#define TOMSERVO2         IfxGtm_TOM0_13_TOUT35_P33_13_OUT

/*************************************************************************
*  �������ƣ�void Test_Servo(void);
*  ����˵�������Զ��ת��
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��22��
*  ��    ע��
*************************************************************************/

void LQ_Tom_Servo_2chPWM(void)
{
	unsigned short duty=0;
	char txt[16];

	TFTSPI_Init(1);        //LCD��ʼ��  0:����  1������
	TFTSPI_CLS(u16BLUE);   //��ɫ��Ļ
	TFTSPI_P8X16Str(0, 3, "Test Servo",u16WHITE,u16BLACK);		//�ַ�����ʾ
	GPIO_KEY_Init();
	TOM_PWM_InitConfig(TOMSERVO1, 5000, 100);//��ʼ��P33_10 ��ΪPWM����� Ƶ��100Hz ռ�ձ� �ٷ�֮(5000/TOM_PWM_MAX)*100
	TOM_PWM_InitConfig(TOMSERVO2, 5000, 100);//��ʼ��P33_13 ��ΪPWM����� Ƶ��100Hz ռ�ձ� �ٷ�֮(5000/TOM_PWM_MAX)*100
	while(1)
	{
		if(KEY_Read(KEY0)==0)//����KEY0����ռ�ձȼ�С�����˳ʱ��ת��
		{
			//if(duty>950)
				duty-=100;
		}
		if(KEY_Read(KEY2)==0)//����KEY2����ռ�ձȼӴ󣬶����ʱ��ת��
		{
			//if(duty<1050)
				duty+=100;
		}
		if(KEY_Read(KEY1)==0)//����KEY1���������λ��ֵ
		{
			duty=1000;
		}

		TOM_PWM_SetDuty(TOMSERVO2, duty, 50);
		TOM_PWM_SetDuty(TOMSERVO1, duty, 50);

		sprintf(txt, "PWM: %05d;", duty);
		TFTSPI_P8X16Str(0, 5, txt,u16WHITE,u16BLACK);		//�ַ�����ʾ
		UART_PutStr(UART0,txt);

    	LED_Ctrl(LED0,RVS);        //��ƽ��ת,LED��˸
		delayms(200);              //��ʱ�ȴ�
	}
}
