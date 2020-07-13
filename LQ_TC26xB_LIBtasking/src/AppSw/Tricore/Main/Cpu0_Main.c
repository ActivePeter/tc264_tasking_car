/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�TC264DA���İ�
����    д��ZYF/chiusir
��E-mail  ��chiusir@163.com
������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
�������¡�2020��4��24��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��dev.env.��Hightec4.9.3/Tasking6.3�����ϰ汾
��Target �� TC264DA/TC264D
��Crystal�� 20.000Mhz
��SYS PLL�� 200MHz
________________________________________________________________
����iLLD_1_0_1_11_0�ײ����,

ʹ�����̵�ʱ�򣬽������û�пո��Ӣ��·����
����CIFΪTC264DA�����⣬�����Ĵ������TC264D
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "..\Driver\include.h"//����ģ���ͷ�ļ�

App_Cpu0 g_AppCpu0; /**< \brief CPU 0 global data */


/*************************************************************************
*  �������ƣ�int core0_main (void)
*  ����˵����CPU0������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��
*************************************************************************/
int core0_main (void)
{
	//�ر�CPU���ж�
	IfxCpu_disableInterrupts();

	//�رտ��Ź�����������ÿ��Ź�ι����Ҫ�ر�
	IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
	IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

	//��ȡ����Ƶ��
	g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
	g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
	g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
	g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);

	//����CPU���ж�
	IfxCpu_enableInterrupts();

	//LED������P10.6��P10.5��ʼ��
	GPIO_LED_Init();

	//����P14.0�ܽ����,P14.1���룬������115200
	UART_InitConfig(UART0_RX_P14_1,UART0_TX_P14_0, 115200);

    //��Ƶ��ַ��https://www.bilibili.com/video/BV1xp4y1C7Hu?from=search&seid=15269881299247521732

    //���²��Ժ������ڽ���ѭ�����û��ɵ�������ģ��ĳ�ʼ������д������ʵ���Լ�������
	//________________________________________________________________________________
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    Test_ADC();            //PASS,����ADC����ʱ��  OLED����ʾ ADC����10K��ʱ��
//    Test_ADC_7mic();       //PASS,����ADC\UART0��STM��ʱ�����ƣ�ͨ��AN0--AN7��8��ͨ�����ADCת������
//    LQ_Atom_Motor_8chPWM();//PASS,����GTM_ATOM8·PWM����ͬƵ���µ�8·�����ƣ���Ϊ�������
//    LQ_ATom_Servo_2chPWM();//PASS,����GTM_ATOM��STM��ʱ�����ƣ�P33.10��P33.13��ΪTOM����ڿ��ƶ��
//    Test_GPIO_Extern_Int();//PASS,�����ⲿ��1���ж�P15.8��P10.6��P10.5����
      //Test_GPIO_LED();       //PASS,����GPIO��P10.6��P10.5����
//    Test_GPIO_KEY();       //PASS,�����ⲿ�������룬P22.0--2
//    Test_ComKEY_Tft();     //PASS,�����ⲿ��ϰ������벢TFT1.8��ʾ��P22.0--2
//    LQ_GPT_4mini512();     //PASS,���Ա�������������,OLED��UART���
//    LQ_GPT_4mini512TFT();  //PASS,���Ա�������������,TFT1.8��UART���
//    Test_OLED();           //PASS,����OLED0.96��ʹ��P20.14--10����ʾ�ַ�������̬����
//    LQ_STM_Timer();        //PASS,���Զ�ʱ�жϡ�����
//    Test_TFT18();          //PASS,����TFT1.8����ʹ��P20.14--10����ʾ�ַ�������̬����
//    LQ_TIM_InputCature();  //PASS,����GTM_TOM_TIM��P20.7��ΪPWM����ڣ�P33.12��ΪTIM����ڣ����߶̽Ӻ󣬴���P14.0���͵���λ��
//    Test_Bluetooth();      //PASS,����UART0(P14.0RX/P14.1TX)��
//    Test_EEPROM();         //PASS,�����ڲ�EEPROM��д����
//    Test_Vl53();           //PASS,����VL53  IIC����   P13_1��SCL  P13_2��SDA OLED��ʾԭʼ����
//    Test_9AX();            //PASS,����������� IIC����   P13_1��SCL  P13_2��SDA OLED��ʾԭʼ����
//    Test_MPU6050();        //PASS,����MPU6050����ICM20602IIC����   P13_1��SCL  P13_2��SDA OLED��ʾԭʼ����
//    Test_ICM20602();       //PASS,����ICM20602 SPI����   P15_6��SCL  P00_0��SDA  P15_7��SA  P00_3��CS OLED��ʾԭʼ����
//     Test_CAMERA();         //PASS,����������������ͷ������Ļ����ʾ  LQ_CAMERA.h ��ѡ����Ļ
//    Test_SoftFft();        //PASS,����ILLD������FFT����
//    Test_FFT();            //PASS,����Ӳ��FFT  ע����ҪоƬ��׺��DA�Ĳ���Ӳ��FFT����
//	  Test_RDA5807();         //PASS,����RDA5807������������SCL �� P00_1   SDA �� P00_2����������ѡ̨��
	startMainTask();
	//________________________________________________________________________________
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while(1)//��ѭ��
	{
    	LED_Ctrl(LED0,RVS);        //��ƽ��ת,LED��˸
		delayms(500);              //��ʱ�ȴ�
	}
}



