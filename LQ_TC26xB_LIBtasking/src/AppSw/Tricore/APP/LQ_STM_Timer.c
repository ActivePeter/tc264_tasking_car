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

#include <LQ_GPIO_LED.h>
#include <LQ_STM.h>


 /*************************************************************************
 *  �������ƣ�void Test_STM(void)
 *  ����˵�������Գ���
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��10��
 *  ��    ע�����İ��ϵ�LED����˸���ж�ʱP10.5/P10.6����
 *************************************************************************/
 void LQ_STM_Timer (void)
  {

	 //STM_InitConfig(STM0, STM_Channel_0, 500000);//STM��ʼ��
	// STM_InitConfig(STM0, STM_Channel_1, 1000000);//STM��ʼ��

	//�жϷ������з�תLED
  	while(1)
  	{
    	//LED_Ctrl(LED0,RVS);        //��ƽ��ת,LED��˸
		delayms(500);              //��ʱ�ȴ�
  	}
  }
