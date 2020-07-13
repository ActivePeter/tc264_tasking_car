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
ASC�첽����ͨ�ţ�������ΪUART������LIN����ʹ�ã�
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef _LQ_ASC_H_
#define _LQ_ASC_H_

#include <Platform_Types.h>
#include <stdint.h>
#include <Ifx_Fifo.h>
#include <Ifx_Types.h>
#include <IfxAsclin.h>
#include <IfxAsclin_Asc.h>
#include <IfxAsclin_PinMap.h>
#include <IfxCpu.h>
#include <IfxCpu_Irq.h>
#include <IfxPort.h>
#include <IfxPort_regdef.h>


/** UART0�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  UART0_RX_PRIORITY    120

/** UART0�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  UART0_TX_PRIORITY    121

/** UART0�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  UART0_ER_PRIORITY    122

/** UART0�жϷ��������ȼ� �жϹ��ĸ��ں˹��� ��Χ��0��CPU0   1��CPU1   3��DMA*/
#define  UART0_VECTABNUM    0


/** UART1�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  UART1_RX_PRIORITY    130

/** UART1�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  UART1_TX_PRIORITY    131

/** UART1�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  UART1_ER_PRIORITY    132

/** UART1�жϷ��������ȼ� �жϹ��ĸ��ں˹��� ��Χ��0��CPU0   1��CPU1   3��DMA*/
#define  UART1_VECTABNUM    0


/** UART2�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  UART2_RX_PRIORITY    140

/** UART2�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  UART2_TX_PRIORITY    141

/** UART2�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  UART2_ER_PRIORITY    142

/** UART2�жϷ��������ȼ� �жϹ��ĸ��ں˹��� ��Χ��0��CPU0   1��CPU1   3��DMA*/
#define  UART2_VECTABNUM    0


/** UART3�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  UART3_RX_PRIORITY    150

/** UART3�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  UART3_TX_PRIORITY    151

/** UART3�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  UART3_ER_PRIORITY    152

/** UART3�жϷ��������ȼ� �жϹ��ĸ��ں˹��� ��Χ��0��CPU0   1��CPU1   3��DMA*/
#define  UART3_VECTABNUM    0

/**
  * @brief UART ö��
  */
typedef enum
{
	UART0 = 0,
	UART1,
	UART2,
	UART3
}UART_t;

//L.Q UART RX �ܽ�ö��
typedef enum
{
	UART0_RX_P14_1 = 0xB401, UART0_RX_P15_3 = 0xB503,

	UART1_RX_P02_3 = 0xA203, UART1_RX_P11_10= 0xB10A, UART1_RX_P15_1 = 0xB501, UART1_RX_P15_5 = 0xB505, UART1_RX_P20_9 = 0xC009, UART1_RX_P33_13= 0xD30D,

	UART2_RX_P02_0 = 0xA200, UART2_RX_P02_1 = 0xA201, UART2_RX_P10_6 = 0xB006, UART2_RX_P14_3 = 0xB403, UART2_RX_P33_8 = 0xD308,

	UART3_RX_P00_1 = 0xA001, UART3_RX_P15_7 = 0xB507, UART3_RX_P20_3 = 0xC003, UART3_RX_P21_6 = 0xC106

}UART_RX_t;


//L.Q UART TX �ܽ�ö��
typedef enum
{
	UART0_TX_P14_0 = 0xB400, UART0_TX_P14_1 = 0xB401, UART0_TX_P15_2 = 0xB502, UART0_TX_P15_3 = 0xB503,

	UART1_TX_P02_2 = 0xA202, UART1_TX_P11_12= 0xB10C, UART1_TX_P15_0 = 0xB500, UART1_TX_P15_1 = 0xB501, UART1_TX_P15_4 = 0xB504, UART1_TX_P15_5 = 0xB505, UART1_TX_P20_10= 0xC00A, UART1_TX_P33_12= 0xD30C, UART1_TX_P33_13 = 0xD30D,

	UART2_TX_P02_0 = 0xA200, UART2_TX_P10_5 = 0xB005, UART2_TX_P14_2 = 0xB402, UART2_TX_P14_3 = 0xB403, UART2_TX_P33_8 = 0xD308, UART2_TX_P33_9 = 0xD309,

	UART3_TX_P00_0 = 0xA000, UART3_TX_P00_1 = 0xA001, UART3_TX_P15_6 = 0xB506, UART3_TX_P15_7 = 0xB507, UART3_TX_P20_0 = 0xC000, UART3_TX_P20_3 = 0xC003, UART3_TX_P21_7 = 0xC107

}UART_TX_t;


void UART_InitConfig(UART_RX_t RxPin, UART_TX_t TxPin, uint32 baudrate);
void UART_PutChar(UART_t  uratn, char ch);
void UART_PutStr(UART_t  uratn, char *str);
void UART_PutBuff(UART_t  uratn, uint8_t *buff, uint32 len);
uint8 UART_GetCount(UART_t  uratn);
char UART_GetChar(UART_t  uratn);
char UART_GetBuff(UART_t  uratn, uint8_t *data, uint8_t len);

#endif /* 0_APPSW_TRICORE_APP_LQ_ASC_H_ */
