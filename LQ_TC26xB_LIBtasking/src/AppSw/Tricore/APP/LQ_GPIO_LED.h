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
#ifndef __LQ_LED_H_
#define __LQ_LED_H_


//����ģ���
typedef enum
{
    LED0=0,  //���İ���LED0
    LED1=1,  //���İ���LED1
    LED2=2,  //ĸ����LED0
	LED3=3,   //ĸ����LED1
	LEDALL=4
} LEDn_e;

typedef enum
{
    ON=0,  //��
    OFF=1, //��
    RVS=2, //��ת
}LEDs_e;

//����Ĺܽ�Ҫ��Ӧʵ�ʰ���
#define LED0p      P10_6   //���İ���LED0 ����
#define LED1p      P10_5   //���İ���LED1 ����
#define LED2p      P20_6   //ĸ����LED0
#define LED3p      P20_7   //ĸ����LED1

/*********************** UART���ܺ��� **************************/
//��ʼ��
void GPIO_LED_Init(void);
void LED_Ctrl(LEDn_e LEDno, LEDs_e sta);
void Test_GPIO_LED(void);


#endif/* 0_APPSW_TRICORE_APP_LQ_ASC_H_ */
