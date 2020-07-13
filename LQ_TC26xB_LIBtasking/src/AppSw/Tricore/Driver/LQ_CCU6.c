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
*  ��    ע��TC264 ������CCU6ģ��  ÿ��ģ��������������ʱ��  ������ʱ���ж�
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/



#include <LQ_CCU6.h>
#include <LQ_GPIO_LED.h>
#include <Platform_Types.h>




IFX_INTERRUPT(CCU60_CH0_IRQHandler, CCU60_VECTABNUM, CCU60_CH0_PRIORITY);
IFX_INTERRUPT(CCU60_CH1_IRQHandler, CCU60_VECTABNUM, CCU60_CH1_PRIORITY);
IFX_INTERRUPT(CCU61_CH0_IRQHandler, CCU61_VECTABNUM, CCU61_CH0_PRIORITY);
IFX_INTERRUPT(CCU61_CH1_IRQHandler, CCU61_VECTABNUM, CCU61_CH1_PRIORITY);

/** CCU6�ж�CPU��� */
const uint8 Ccu6IrqVectabNum[2]  = {CCU60_VECTABNUM, CCU61_VECTABNUM};

/** CCU6�ж����ȼ� */
const uint8 Ccu6IrqPriority[4]   = {CCU60_CH0_PRIORITY, CCU60_CH1_PRIORITY, CCU61_CH0_PRIORITY, CCU61_CH1_PRIORITY};

/** CCU6�жϷ�������ַ */
const void *Ccu6IrqFuncPointer[4] = {&CCU60_CH0_IRQHandler, &CCU60_CH1_IRQHandler, &CCU61_CH0_IRQHandler, &CCU61_CH1_IRQHandler};



/***********************************************************************************************/
/********************************CCU6�ⲿ�ж�  ������******************************************/
/***********************************************************************************************/

/*************************************************************************
*  �������ƣ�void CCU60_CH0_IRQHandler(void)
*  ����˵����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��30��
*  ��    ע��CCU60_CH0ʹ�õ��жϷ�����
*************************************************************************/
void CCU60_CH0_IRQHandler(void)
{
	/* ����CPU�ж�  �����жϲ���Ƕ�� */
	IfxCpu_enableInterrupts();

	//����жϱ�־
	IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU60, IfxCcu6_InterruptSource_t12PeriodMatch);

	/* �û����� */
	LED_Ctrl(LED0,RVS);        //��ƽ��ת,LED��˸
}

/*************************************************************************
*  �������ƣ�void CCU60_CH1_IRQHandler(void)
*  ����˵����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��30��
*  ��    ע��CCU60_CH1ʹ�õ��жϷ�����
*************************************************************************/
void CCU60_CH1_IRQHandler(void)
{
	/* ����CPU�ж�  �����жϲ���Ƕ�� */
	IfxCpu_enableInterrupts();

	//����жϱ�־
	IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU60, IfxCcu6_InterruptSource_t13PeriodMatch);

	/* �û����� */
	LED_Ctrl(LED1,RVS);        //��ƽ��ת,LED��˸

}

/*************************************************************************
*  �������ƣ�void CCU61_CH0_IRQHandler(void)
*  ����˵����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��30��
*  ��    ע��CCU61_CH0ʹ�õ��жϷ�����
*************************************************************************/
void CCU61_CH0_IRQHandler(void)
{
	/* ����CPU�ж�  �����жϲ���Ƕ�� */
	IfxCpu_enableInterrupts();

	//����жϱ�־
	IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t12PeriodMatch);

	/* �û����� */
	LED_Ctrl(LED1,RVS);        //��ƽ��ת,LED��˸

}

/*************************************************************************
*  �������ƣ�void CCU61_CH1_IRQHandler(void)
*  ����˵����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��30��
*  ��    ע��CCU61_CH1ʹ�õ��жϷ�����
*************************************************************************/
void CCU61_CH1_IRQHandler(void)
{
	/* ����CPU�ж�  �����жϲ���Ƕ�� */
	IfxCpu_enableInterrupts();

	//����жϱ�־
	IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t13PeriodMatch);

	/* �û����� */
	LED_Ctrl(LED0,RVS);        //��ƽ��ת,LED��˸
}


/*************************************************************************
*  �������ƣ�CCU6_InitConfig CCU6
*  ����˵������ʱ�������жϳ�ʼ��
*  ����˵����ccu6    �� ccu6ģ��            CCU60 �� CCU61
*  ����˵����channel �� ccu6ģ��ͨ��  CCU6_Channel0 �� CCU6_Channel1
*  ����˵����us      �� ccu6ģ��  �ж�����ʱ��  ��λus
*  �������أ���
*  �޸�ʱ�䣺2020��3��30��
*  ��    ע��
*************************************************************************/
void CCU6_InitConfig(CCU6_t ccu6, CCU6_Channel_t channel, uint32 us)
{
	IfxCcu6_Timer_Config timerConfig;

	Ifx_CCU6 * module = IfxCcu6_getAddress((IfxCcu6_Index)ccu6);

	uint8 Index = ccu6 * 2 + channel;

	uint32 period = 0;

	uint64 clk = 0;

	/* �ر��ж� */
	boolean interrupt_state = disableInterrupts();

	IfxCcu6_Timer_initModuleConfig(&timerConfig, module);

	clk = IfxScuCcu_getSpbFrequency();

	/* ����ʱ��Ƶ��  */
	uint8 i = 0;
	while(i++<16)
	{
		period = (uint32)(clk * us / 1000000);
		if(period < 0xffff)
		{
			break;
		}
		else
		{
			clk = clk/2;
		}
	}
	switch(channel)
	{
		case CCU6_Channel0:
			timerConfig.timer = IfxCcu6_TimerId_t12;
			timerConfig.interrupt1.source         = IfxCcu6_InterruptSource_t12PeriodMatch;
			timerConfig.interrupt1.serviceRequest = IfxCcu6_ServiceRequest_1;
			timerConfig.base.t12Frequency 		  = (float)clk;
			timerConfig.base.t12Period 			  = period;                                  //���ö�ʱ�ж�
			timerConfig.clock.t12countingInputMode = IfxCcu6_CountingInputMode_internal;
			timerConfig.timer12.counterValue      = 0;
			timerConfig.interrupt1.typeOfService  = Ccu6IrqVectabNum[ccu6];
			timerConfig.interrupt1.priority       = Ccu6IrqPriority[Index];
			break;

		case CCU6_Channel1:
			timerConfig.timer = IfxCcu6_TimerId_t13;
			timerConfig.interrupt2.source         = IfxCcu6_InterruptSource_t13PeriodMatch;
			timerConfig.interrupt2.serviceRequest = IfxCcu6_ServiceRequest_2;
			timerConfig.base.t13Frequency 		  = (float)clk;
			timerConfig.base.t13Period 			  = period;
			timerConfig.clock.t13countingInputMode = IfxCcu6_CountingInputMode_internal;
			timerConfig.timer13.counterValue      = 0;
			timerConfig.interrupt2.typeOfService  = Ccu6IrqVectabNum[ccu6];
			timerConfig.interrupt2.priority       = Ccu6IrqPriority[Index];
			break;
	}

    timerConfig.trigger.t13InSyncWithT12 = FALSE;

    IfxCcu6_Timer Ccu6Timer;

    IfxCcu6_Timer_initModule(&Ccu6Timer, &timerConfig);

    IfxCpu_Irq_installInterruptHandler((void*)Ccu6IrqFuncPointer[Index], Ccu6IrqPriority[Index]);//�����жϺ������жϺ�

	restoreInterrupts(interrupt_state);

	IfxCcu6_Timer_start(&Ccu6Timer);
}






