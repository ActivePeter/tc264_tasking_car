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
#ifndef __LQ_KEY_H_
#define __LQ_KEY_H_

//����ģ���
typedef enum
{
    KEY0=0,  //ĸ���ϰ���0
    KEY1=1,
    KEY2=2,
	DSW0,    //ĸ���ϲ��뿪��0
	DSW1
} KEYn_e;

typedef enum
{
    LOW=0,  //����
    HIGH=1, //�ɿ�
    FAIL=0xff,//����
}KEYs_e;

typedef enum //
{
	NOKEYDOWN=0,
	KEY0DOWN=0x01,  //ĸ���ϰ���0
	KEY1DOWN=0x02,
	KEY2DOWN=0x04,
	KEY01DOWN=0x03,  //ĸ���ϰ���0
	KEY02DOWN=0x05,
	KEY12DOWN=0x06,
	KEY012DOWN=0x07,
	KEYError=0xFF,

}KEYdown_e;
//����Ĺܽ�Ҫ��Ӧʵ�ʰ���
#define KEY0p      P22_0  //ĸ���ϰ���0
#define KEY1p      P22_1  //ĸ���ϰ���1
#define KEY2p      P22_2  //ĸ���ϰ���2
#define DSW0p      P33_9  //ĸ���ϲ��뿪��0
#define DSW1p      P33_11 //ĸ���ϲ��뿪��1
/*********************** UART���ܺ��� **************************/
//��ʼ��
void GPIO_KEY_Init(void);
unsigned char KEY_Read(KEYn_e KEYno);
unsigned char KEY_Read_All(void);
void Test_GPIO_KEY(void);
void Test_ComKEY_Tft(void);

#endif/* 0_APPSW_TRICORE_APP_LQ_ASC_H_ */
