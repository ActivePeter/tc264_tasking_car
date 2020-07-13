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

����iLLD_1_0_1_11_0�ײ����
STM��Ϊϵͳ����ȫ�ֶ�ʱ��������Ϊϵͳ�ṩ����ʱ�ӣ����������ȽϼĴ�����
�ɲ��������������жϣ�����������
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/


#include <LQ_GPIO_LED.h>
#include <LQ_STM.h>


/*************************************************************************
*  �������ƣ�
*  ����˵����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��30��
*  ��    ע��STM�ж�
*************************************************************************/
IFX_INTERRUPT(STM0_CH0_IRQHandler, STM0_VECTABNUM, STM0_CH0_PRIORITY);
IFX_INTERRUPT(STM0_CH1_IRQHandler, STM0_VECTABNUM, STM0_CH1_PRIORITY);
IFX_INTERRUPT(STM1_CH0_IRQHandler, STM1_VECTABNUM, STM1_CH0_PRIORITY);
IFX_INTERRUPT(STM1_CH1_IRQHandler, STM1_VECTABNUM, STM1_CH1_PRIORITY);

/** �ⲿ�ж�CPU��� */
const uint8 StmIrqVectabNum[2]  = {STM0_VECTABNUM, STM1_VECTABNUM};

/** �ⲿ�ж����ȼ� */
const uint8 StmIrqPriority[4]   = {STM0_CH0_PRIORITY, STM0_CH1_PRIORITY, STM1_CH0_PRIORITY, STM1_CH1_PRIORITY};

/** �ⲿ�жϷ�������ַ */
const void *StmIrqFuncPointer[4] = {&STM0_CH0_IRQHandler, &STM0_CH1_IRQHandler, &STM1_CH0_IRQHandler, &STM1_CH1_IRQHandler};

void (*tim_task1)();
void (*tim_task2)();
void (*tim_task3)();
void (*tim_task4)();
//	STM��ʼ�����ýṹ��
IfxStm_CompareConfig g_StmCompareConfig[4];

//pa�Լ�����Ļص�����ָ�����


/***********************************************************************************************/
/********************************STM�ⲿ�ж�  ������******************************************/
/***********************************************************************************************/

/*************************************************************************
*  �������ƣ�void STM0_CH0_IRQHandler(void)
*  ����˵����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��30��
*  ��    ע��STM0_CH0ʹ�õ��жϷ�����
*************************************************************************/
void STM0_CH0_IRQHandler(void)
{
	/* ����CPU�ж�  �����жϲ���Ƕ�� */
	IfxCpu_enableInterrupts();

	//����жϱ�־
	IfxStm_clearCompareFlag(&MODULE_STM0, g_StmCompareConfig[0].comparator);

	//�����µ��ж����ã���ʼ�´��ж�
	IfxStm_increaseCompare(&MODULE_STM0, g_StmCompareConfig[0].comparator, g_StmCompareConfig[0].ticks);

	/* �û����� */
	LED_Ctrl(LED0,RVS);        //��ƽ��ת,LED��˸
	(*tim_task1)();
}

/*************************************************************************
*  �������ƣ�void STM0_CH1_IRQHandler(void)
*  ����˵����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��30��
*  ��    ע��STM0_CH1ʹ�õ��жϷ�����
*************************************************************************/
void STM0_CH1_IRQHandler(void)
{
	/* ����CPU�ж�  �����жϲ���Ƕ�� */
	IfxCpu_enableInterrupts();

	//����жϱ�־
	IfxStm_clearCompareFlag(&MODULE_STM0, g_StmCompareConfig[1].comparator);

	//�����µ��ж����ã���ʼ�´��ж�
	IfxStm_increaseCompare(&MODULE_STM0, g_StmCompareConfig[1].comparator, g_StmCompareConfig[1].ticks);

	/* �û����� */
	LED_Ctrl(LED1,RVS);        //��ƽ��ת,LED��˸

}

/*************************************************************************
*  �������ƣ�void STM1_CH0_IRQHandler(void)
*  ����˵����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��30��
*  ��    ע��STM1_CH0ʹ�õ��жϷ�����
*************************************************************************/
void STM1_CH0_IRQHandler(void)
{
	/* ����CPU�ж�  �����жϲ���Ƕ�� */
	IfxCpu_enableInterrupts();

	//����жϱ�־
	IfxStm_clearCompareFlag(&MODULE_STM1, g_StmCompareConfig[2].comparator);

	//�����µ��ж����ã���ʼ�´��ж�
	IfxStm_increaseCompare(&MODULE_STM1, g_StmCompareConfig[2].comparator, g_StmCompareConfig[2].ticks);

	/* �û����� */
	LED_Ctrl(LED1,RVS);        //��ƽ��ת,LED��˸

}

/*************************************************************************
*  �������ƣ�void STM1_CH1_IRQHandler(void)
*  ����˵����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��30��
*  ��    ע��STM1_CH1ʹ�õ��жϷ�����
*************************************************************************/
void STM1_CH1_IRQHandler(void)
{
	/* ����CPU�ж�  �����жϲ���Ƕ�� */
	IfxCpu_enableInterrupts();

	//����жϱ�־
	IfxStm_clearCompareFlag(&MODULE_STM1, g_StmCompareConfig[3].comparator);

	//�����µ��ж����ã���ʼ�´��ж�
	IfxStm_increaseCompare(&MODULE_STM1, g_StmCompareConfig[3].comparator, g_StmCompareConfig[3].ticks);

	/* �û����� */
	LED_Ctrl(LED1,RVS);        //��ƽ��ת,LED��˸
	uint32 i = 100;
	while(i--)
	{
		uint32_t j = 100;
		while(j--) NOP(100);
	}
	LED_Ctrl(LED0,RVS);        //��ƽ��ת,LED��˸
}


 /*************************************************************************
 *  �������ƣ�void STM_Init(STM_t stm, STM_Channel_t channel, uint32 us)
 *  ����˵����STM��ʼ��
 *  ����˵����
 * @param    stm      �� STM���     STM0 �� STM1
 * @param    channel  �� STMͨ��     STM_Channel_0  STM_Channel_1
 * @param    us       �� STM��ʱ�ж� ʱ��
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��10��
 *  ��    ����STM_InitConfig(STM0, STM_Channel_0, 50000);  //STM0 ͨ��0��ʼ�� 50000us ����һ���ж�
 *  ��    ע���ⲿ�жϷ�������LQ_STM.c��  �ж����ȼ�������LQ_STM.h�� ���������޸�
 *************************************************************************/
 void STM_InitConfig(STM_t stm, STM_Channel_t channel, uint32 us,void (*task)())
 {
	
	tim_task1=task;
 	char      interruptState;
 	interruptState = disableInterrupts();

 	Ifx_STM * STM = IfxStm_getAddress((IfxStm_Index)stm);
 	uint8_t Index = (uint8_t)IfxStm_getIndex(STM) * 2 + (uint8_t)channel;

 	IfxStm_CompareConfig *stmCompareConfig = &g_StmCompareConfig[Index];

 	//���ýṹ��ʵ������
 	IfxStm_initCompareConfig(stmCompareConfig);

     //����
 	IfxStm_enableOcdsSuspend(STM);

 	//��ȡSTMģ���΢���ʱ tick ��ֵ
 	sint32 ticks = IfxStm_getTicksFromMicroseconds((Ifx_STM *)STM, us);

 	//�޸�ticks��ֵ��ʹ�ܴ������
 	stmCompareConfig->ticks = ticks;
 	stmCompareConfig->comparator           = (IfxStm_Comparator)channel;
 	stmCompareConfig->comparatorInterrupt  = (IfxStm_ComparatorInterrupt)channel; //�ж�ѡ��
 	stmCompareConfig->triggerPriority      = StmIrqPriority[Index];               //�ж����ȼ�����
 	stmCompareConfig->typeOfService        = StmIrqVectabNum[stm];                //�����ں�

 	//�ȽϹ��ܳ�ʼ��
 	IfxStm_initCompare(STM, stmCompareConfig);
 	IfxCpu_Irq_installInterruptHandler((void*)StmIrqFuncPointer[Index], StmIrqPriority[Index]);//�����жϺ������жϺ�

 	restoreInterrupts(interruptState);
 }


 /*************************************************************************
 *  �������ƣ�void STM_DelayUs(STM_t stm, uint32 us)
 *  ����˵����STM��ʱ
 *  ����˵����
 * @param    stm      �� STM���     STM0 �� STM1
 * @param    us       �� STM��ʱ     ʱ�� us
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��10��
 *  ��    ����STM_DelayUs(STM0, 5000);  //��ʱ5000us
 *  ��    ע��ʹ����ʱ����ʱ����ȷ����ӦSTMģ����ù�STM_Init����
 *************************************************************************/
 void STM_DelayUs(STM_t stm, uint32 us)
 {
 	Ifx_STM * STM = IfxStm_getAddress((IfxStm_Index)stm);
 	uint32 tick = IfxStm_getTicksFromMicroseconds(STM, us);
 	IfxStm_waitTicks(STM, tick);
 }

 /*************************************************************************
 *  �������ƣ�void delayms(unsigned short stmms)
 *  ����˵������ʱ����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��10��
 *  ��    ע��IfxStm_waitTicks(&MODULE_STM0, 1);��Լ10΢��
 *************************************************************************/
 void delayms(unsigned short stmms)	//
 {
   while(stmms--)
   {
 	  IfxStm_waitTicks(&MODULE_STM0, 100000);
   }
 }
 /*************************************************************************
 *  �������ƣ�uint32  STM_GetNowUs(STM_t stm)
 *  ����˵����STM��ȡ��ǰʱ��
 *  ����˵����
 * @param    stm      �� STM���     STM0 �� STM1
 * @param    us       �� STM��ʱ     ʱ�� us
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��10��
 *  ��    ����int32_t NowTime =  STM_GetNowUs(STM0);  //��ȡSTM0 ��ǰʱ��
 *  ��    ע��ʹ����ʱ����ʱ����ȷ����ӦSTMģ����ù�STM_Init����
 *************************************************************************/
 uint32  STM_GetNowUs(STM_t stm)
 {
 	Ifx_STM * STM = IfxStm_getAddress((IfxStm_Index)stm);
 	sint32 freq = (sint32)IfxStm_getFrequency(STM)/1000000;
 	Ifx_TickTime stmNow;
 	char      interruptState;

 	interruptState = disableInterrupts();
 	stmNow         = (Ifx_TickTime)IfxStm_get(STM) & TIME_INFINITE;
 	restoreInterrupts(interruptState);

 	return (uint32_t)(stmNow/freq);
 }




