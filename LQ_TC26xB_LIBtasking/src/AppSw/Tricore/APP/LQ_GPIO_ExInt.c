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
��Ƶ��ַ��https://www.bilibili.com/video/BV1xp4y1C7Hu?from=search&seid=15269881299247521732
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <LQ_GPIO.h>
#include <LQ_GPIO_LED.h>
#include <LQ_STM.h>

/*************************************************************************
*  �������ƣ�void Test_GPIO_Extern_Int(void)
*  ����˵���������ⲿ�ж�
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע�����İ��ϵ�LED�̶�ʱ��P10.6��ת��LED����˸��
*            ��P15.8��⵽�½���ʱ�����жϣ�P10.5��ת��LED��˸���жϺ�����LQ_GPIO.C��
*            ���԰�P15.8�öŰ��߽ӵ�P10.6�������жϺ�����������˸
*************************************************************************/
void Test_GPIO_Extern_Int(void)
{
	PIN_Exti(P15_8, PIN_IRQ_MODE_FALLING);//����P15_8 �½��ش����ж�
	while(1)
	{
    	LED_Ctrl(LED0,RVS);        //��ƽ��ת,LED��˸
		delayms(500);              //��ʱ�ȴ�
	}
}

