/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技TC264DA核心板
【编    写】ZYF/chiusir
【E-mail  】chiusir@163.com
【软件版本】V1.1 版权所有，单位使用请先联系授权
【最后更新】2020年4月24日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【dev.env.】Hightec4.9.3/Tasking6.3及以上版本
【Target 】 TC264DA/TC264D
【Crystal】 20.000Mhz
【SYS PLL】 200MHz
________________________________________________________________
基于iLLD_1_0_1_11_0底层程序,

使用例程的时候，建议采用没有空格的英文路径，
除了CIF为TC264DA独有外，其它的代码兼容TC264D
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "..\Driver\include.h"//各个模块的头文件

App_Cpu0 g_AppCpu0; /**< \brief CPU 0 global data */


/*************************************************************************
*  函数名称：int core0_main (void)
*  功能说明：CPU0主函数
*  参数说明：无
*  函数返回：无
*  修改时间：2020年3月10日
*  备    注：
*************************************************************************/
int core0_main (void)
{
	//关闭CPU总中断
	IfxCpu_disableInterrupts();

	//关闭看门狗，如果不设置看门狗喂狗需要关闭
	IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
	IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

	//读取总线频率
	g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
	g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
	g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
	g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);

	//开启CPU总中断
	IfxCpu_enableInterrupts();

	//LED灯所用P10.6和P10.5初始化
	GPIO_LED_Init();

	//串口P14.0管脚输出,P14.1输入，波特率115200
	UART_InitConfig(UART0_RX_P14_1,UART0_TX_P14_0, 115200);

    //视频地址：https://www.bilibili.com/video/BV1xp4y1C7Hu?from=search&seid=15269881299247521732

    //以下测试函数，内建死循环，用户可调用所用模块的初始化及读写函数来实现自己的任务
	//________________________________________________________________________________
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    Test_ADC();            //PASS,测试ADC采样时间  OLED上显示 ADC采样10K次时间
//    Test_ADC_7mic();       //PASS,测试ADC\UART0、STM延时和闪灯，通过AN0--AN7共8个通道输出ADC转换数据
//    LQ_Atom_Motor_8chPWM();//PASS,测试GTM_ATOM8路PWM，不同频率下的8路呼吸灯，作为电机驱动
//    LQ_ATom_Servo_2chPWM();//PASS,测试GTM_ATOM、STM延时和闪灯，P33.10和P33.13作为TOM输出口控制舵机
//    Test_GPIO_Extern_Int();//PASS,测试外部第1组中断P15.8，P10.6和P10.5闪灯
      //Test_GPIO_LED();       //PASS,测试GPIO，P10.6和P10.5闪灯
//    Test_GPIO_KEY();       //PASS,测试外部按键输入，P22.0--2
//    Test_ComKEY_Tft();     //PASS,测试外部组合按键输入并TFT1.8显示，P22.0--2
//    LQ_GPT_4mini512();     //PASS,测试编码器正交解码,OLED和UART输出
//    LQ_GPT_4mini512TFT();  //PASS,测试编码器正交解码,TFT1.8和UART输出
//    Test_OLED();           //PASS,测试OLED0.96屏使用P20.14--10，显示字符串及动态数据
//    LQ_STM_Timer();        //PASS,测试定时中断、闪灯
//    Test_TFT18();          //PASS,测试TFT1.8彩屏使用P20.14--10，显示字符串及动态数据
//    LQ_TIM_InputCature();  //PASS,测试GTM_TOM_TIM、P20.7作为PWM输出口，P33.12作为TIM输入口，两者短接后，串口P14.0发送到上位机
//    Test_Bluetooth();      //PASS,测试UART0(P14.0RX/P14.1TX)，
//    Test_EEPROM();         //PASS,测试内部EEPROM擦写功能
//    Test_Vl53();           //PASS,测试VL53  IIC接线   P13_1接SCL  P13_2接SDA OLED显示原始数据
//    Test_9AX();            //PASS,测试龙邱九轴 IIC接线   P13_1接SCL  P13_2接SDA OLED显示原始数据
//    Test_MPU6050();        //PASS,测试MPU6050或者ICM20602IIC接线   P13_1接SCL  P13_2接SDA OLED显示原始数据
//    Test_ICM20602();       //PASS,测试ICM20602 SPI接线   P15_6接SCL  P00_0接SDA  P15_7接SA  P00_3接CS OLED显示原始数据
//     Test_CAMERA();         //PASS,测试龙邱神眼摄像头并在屏幕上显示  LQ_CAMERA.h 中选择屏幕
//    Test_SoftFft();        //PASS,测试ILLD库的软件FFT函数
//    Test_FFT();            //PASS,测试硬件FFT  注意需要芯片后缀带DA的才有硬件FFT功能
//	  Test_RDA5807();         //PASS,测试RDA5807立体声收音机SCL 接 P00_1   SDA 接 P00_2，按键操作选台等
	startMainTask();
	//________________________________________________________________________________
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while(1)//主循环
	{
    	LED_Ctrl(LED0,RVS);        //电平翻转,LED闪烁
		delayms(500);              //延时等待
	}
}



