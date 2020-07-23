/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技TC264DA核心板
【编    写】ZYF/chiusir
【E-mail  】chiusir@163.com
【软件版本】V1.1 版权所有，单位使用请先联系授权
【最后更新】2020年4月10日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【dev.env.】Hightec4.9.3/Tasking6.3及以上版本
【Target 】 TC264DA
【Crystal】 20.000Mhz
【SYS PLL】 200MHz
________________________________________________________________

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <LQ_GPIO_LED.h>
#include <LQ_STM.h>


 /*************************************************************************
 *  函数名称：void Test_STM(void)
 *  功能说明：测试程序
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年3月10日
 *  备    注：核心板上的LED灯闪烁，中断时P10.5/P10.6闪灯
 *************************************************************************/
 void LQ_STM_Timer (void)
  {

	 //STM_InitConfig(STM0, STM_Channel_0, 500000);//STM初始化
	// STM_InitConfig(STM0, STM_Channel_1, 1000000);//STM初始化

	//中断服务函数中翻转LED
  	while(1)
  	{
    	//LED_Ctrl(LED0,RVS);        //电平翻转,LED闪烁
		delayms(500);              //延时等待
  	}
  }
