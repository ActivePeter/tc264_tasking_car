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

#include <LQ_Atom_Motor.h>
#include <LQ_GPIO_KEY.h>
#include <LQ_GPIO_LED.h>
#include <LQ_GTM.h>
#include <LQ_STM.h>
#include <LQ_TFT18.h>
#include <LQ_UART.h>
#include <stdio.h>



/*************************************************************************
*  �������ƣ�void LQ_Atom_8chPWM(void)
*  ����˵����GTM���Ժ���
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��4��1��
*  ��    ע����ͬƵ���µ�8·�����ƣ��öŰ��߽ӵ�LED�ƿ��Կ��������Ƚ���
*************************************************************************/
void LQ_Atom_8chPWM(void)
{
	unsigned short duty=0;

	ATOM_PWM_InitConfig(ATOMPWM0, 0, 10);
	ATOM_PWM_InitConfig(ATOMPWM1, 0, 100);
	ATOM_PWM_InitConfig(ATOMPWM2, 0, 1000);
	ATOM_PWM_InitConfig(ATOMPWM3, 0, 10000);
	ATOM_PWM_InitConfig(ATOMPWM4, 0, 50);
	ATOM_PWM_InitConfig(ATOMPWM5, 0, 500);
	ATOM_PWM_InitConfig(ATOMPWM6, 0, 5000);
	ATOM_PWM_InitConfig(ATOMPWM7, 0, 50000);

	while(1)
	{
		for(duty=0;duty<100;duty++)
		{
			ATOM_PWM_SetDuty(ATOMPWM0, duty * 100, 10);
			ATOM_PWM_SetDuty(ATOMPWM1, duty * 200, 100);
			ATOM_PWM_SetDuty(ATOMPWM2, duty * 300, 1000);
			ATOM_PWM_SetDuty(ATOMPWM3, duty * 400, 10000);
			ATOM_PWM_SetDuty(ATOMPWM4, duty * 500, 50);
			ATOM_PWM_SetDuty(ATOMPWM5, duty * 600, 500);
			ATOM_PWM_SetDuty(ATOMPWM6, duty * 700, 5000);
			ATOM_PWM_SetDuty(ATOMPWM7, duty * 800, 50000);
			delayms(10);
		}

		//Flash_LED();//���ƺ���P10.5
    	LED_Ctrl(LED0,RVS);        //��ƽ��ת,LED��˸
		delayms(500);              //��ʱ�ȴ�
	}
}
/*************************************************************************
*  �������ƣ�void Test_GTM_ATOM_8chPWM(void)
*  ����˵����GTM���Ժ���
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��4��1��
*  ��    ע�������ĸ����
*************************************************************************/
void LQ_Atom_Motor_8chPWM(void)
{
	unsigned short duty=5000;
	char txt[16];


	TFTSPI_Init(1);        //LCD��ʼ��  0:����  1������
	TFTSPI_CLS(u16BLUE);   //��ɫ��Ļ
	TFTSPI_P8X16Str(0, 3, "Test Motor",u16WHITE,u16BLACK);		//�ַ�����ʾ
	GPIO_KEY_Init();

	ATOM_PWM_InitConfig(ATOMPWM0, 5000, 12500);
	ATOM_PWM_InitConfig(ATOMPWM1, 5000, 12500);
	ATOM_PWM_InitConfig(ATOMPWM2, 5000, 12500);
	ATOM_PWM_InitConfig(ATOMPWM3, 5000, 12500);
	ATOM_PWM_InitConfig(ATOMPWM4, 1000, 12500);
	ATOM_PWM_InitConfig(ATOMPWM5, 1000, 12500);
	ATOM_PWM_InitConfig(ATOMPWM6, 1000, 12500);
	ATOM_PWM_InitConfig(ATOMPWM7, 1000, 12500);

	while(1)
	{
		if(KEY_Read(KEY0)==0)//����KEY0����ռ�ձȼ�С
		{
			if(duty>500)
				duty-=500;
		}
		if(KEY_Read(KEY2)==0)//����KEY2����ռ�ձȼӴ�
		{
			if(duty<ATOM_PWM_MAX)//��ռ�ձ�Ϊ10000
				duty+=500;
		}
		if(KEY_Read(KEY1)==0)//����KEY1����ռ�ձ���ֵ
		{
			duty=5000;
		}

		ATOM_PWM_SetDuty(ATOMPWM0, duty, 12500);//�����ĸ����
		ATOM_PWM_SetDuty(ATOMPWM1, 5000, 12500);
		ATOM_PWM_SetDuty(ATOMPWM2, duty, 12500);
		ATOM_PWM_SetDuty(ATOMPWM3, 5000, 12500);
		//ATOM_PWM_SetDuty(ATOMPWM4, duty, 12500);
		//ATOM_PWM_SetDuty(ATOMPWM5, duty, 12500);
		//ATOM_PWM_SetDuty(ATOMPWM6, duty, 12500);
		//ATOM_PWM_SetDuty(ATOMPWM7, duty, 12500);

		sprintf(txt, "PWM: %05d;", duty);
		TFTSPI_P8X16Str(0, 5, txt,u16WHITE,u16BLACK);		//�ַ�����ʾ
		UART_PutStr(UART0,txt);

    	LED_Ctrl(LED0,RVS);        //��ƽ��ת,LED��˸
		delayms(200);              //��ʱ�ȴ�
	}
}
/*************************************************************************
*  �������ƣ�void LQ_ATom_Servo_2chPWM(void)
*  ����˵����GTM���Ժ���
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��4��1��
*  ��    ע������2��������мǶ�����ܿ���Ŷ���ȱ궨ȷ�Ϻ���š�϶���
*************************************************************************/
void LQ_ATom_Servo_2chPWM(void)
{
	unsigned short duty=1570;
	char txt[16];

	TFTSPI_Init(1);        //LCD��ʼ��  0:����  1������
	TFTSPI_CLS(u16BLUE);   //��ɫ��Ļ
	TFTSPI_P8X16Str(0, 3, "Test Servo",u16WHITE,u16BLACK);		//�ַ�����ʾ
	GPIO_KEY_Init();

	ATOM_PWM_InitConfig(ATOMSERVO1, 1500, 100);//���Ƶ��Ϊ100HZ����ʼֵΪ1.5ms��ֵ
	ATOM_PWM_InitConfig(ATOMSERVO2, 1500, 100);//������۷�ΧΪ��0.5ms--2.5ms�������ʵ�ʱ������ΧС

	while(1)
	{
		if(KEY_Read(KEY0)==0)//����KEY0����ռ�ձȼ�С�����˳ʱ��ת��
		{
			if(duty>1370)
			duty-=10;
		}
		if(KEY_Read(KEY2)==0)//����KEY2����ռ�ձȼӴ󣬶����ʱ��ת��
		{
			if(duty<1770)
			duty+=10;
		}
		if(KEY_Read(KEY1)==0)//����KEY1���������λ��ֵ
		{
			duty=1570;
		}

		ATOM_PWM_SetDuty(ATOMSERVO2, duty, 100);//�����������
		ATOM_PWM_SetDuty(ATOMSERVO1, duty, 100);

		sprintf(txt, "PWM: %05d;", duty);
		TFTSPI_P8X16Str(0, 5, txt,u16WHITE,u16BLACK);		//�ַ�����ʾ
		UART_PutStr(UART0,txt);

    	LED_Ctrl(LED0,RVS);        //��ƽ��ת,LED��˸
		delayms(200);              //��ʱ�ȴ�
	}
}
