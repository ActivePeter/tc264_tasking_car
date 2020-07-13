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
#include <LQ_GPIO_LED.h>
#include <LQ_GTM.h>
#include <LQ_STM.h>
#include <LQ_UART.h>
#include <stdio.h>


/*************************************************************************
*  �������ƣ�void Test_GTM_ATOM_PWM(void)
*  ����˵����GTM���Ժ���
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��22��
*  ��    ע��P20.7��ΪPWM����ڣ�P33.12��ΪTIM����ڣ����߶̽Ӻ󣬴���P14.0���͵���λ��
*  Ĭ��Ƶ����125HZ��ռ�ձ�50%
*************************************************************************/
void LQ_TIM_InputCature(void)
{
	char txt[16];
	float measuredPwmFreq_Hz, measuredPwmDutyCycle;
	TIM_InitConfig(IfxGtm_TIM0_0_TIN34_P33_12_IN);//P20_7_OUT  P33_12_IN
	TOM_PWM_InitConfig(IfxGtm_TOM0_11_TOUT63_P20_7_OUT, 5000, 125);//��ʼ��P20_7 ��ΪPWM����� Ƶ��125Hz ռ�ձ� �ٷ�֮(5000/TOM_PWM_MAX)*100
	while(1)
	{
		TIM_GetPwm(IfxGtm_TIM0_0_TIN34_P33_12_IN, &measuredPwmFreq_Hz, &measuredPwmDutyCycle);
		sprintf(txt,"\nHz:%05f;dty:%f  ", measuredPwmFreq_Hz, measuredPwmDutyCycle);

		//���ڷ��͵���λ��
		UART_PutStr(UART0,txt);//�����������λ�����磺Hz:00124;dty:00050���������������Ϊ�����
    	LED_Ctrl(LED0,RVS);        //��ƽ��ת,LED��˸
		delayms(500);              //��ʱ�ȴ�
	}
}
