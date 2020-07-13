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
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#ifndef _LQ_SPI_H_
#define _LQ_SPI_H_

#include "Cpu/Std/Platform_Types.h"
#include "stdint.h"
#include <IfxAsclin.h>
#include <IfxAsclin_PinMap.h>
#include <IfxAsclin_Spi.h>
#include <IfxCpu.h>
#include <IfxCpu_Irq.h>
#include <IfxPort.h>
#include <IfxSrc_cfg.h>


/** SPI0�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  SPI0_RX_PRIORITY    160

/** SPI0�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  SPI0_TX_PRIORITY    161

/** SPI0�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  SPI0_ER_PRIORITY    162

/** SPI0�жϷ��������ȼ� �жϹ��ĸ��ں˹��� ��Χ��0��CPU0   1��CPU1   3��DMA*/
#define  SPI0_VECTABNUM    0


/** SPI1�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  SPI1_RX_PRIORITY    170

/** SPI1�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  SPI1_TX_PRIORITY    171

/** SPI1�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  SPI1_ER_PRIORITY    172

/** SPI1�жϷ��������ȼ� �жϹ��ĸ��ں˹��� ��Χ��0��CPU0   1��CPU1   3��DMA*/
#define  SPI1_VECTABNUM    0


/** SPI2�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  SPI2_RX_PRIORITY    180

/** SPI2�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  SPI2_TX_PRIORITY    181

/** SPI2�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  SPI2_ER_PRIORITY    182

/** SPI2�жϷ��������ȼ� �жϹ��ĸ��ں˹��� ��Χ��0��CPU0   1��CPU1   3��DMA*/
#define  SPI2_VECTABNUM    0


/** SPI3�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  SPI3_RX_PRIORITY    190

/** SPI3�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  SPI3_TX_PRIORITY    191

/** SPI3�����жϷ��������ȼ�   ��Χ��1-255   ����Խ�� ���ȼ�Խ��  ע�����ȼ���Ҫ�ظ� */
#define  SPI3_ER_PRIORITY    192

/** SPI3�жϷ��������ȼ� �жϹ��ĸ��ں˹��� ��Χ��0��CPU0   1��CPU1   3��DMA*/
#define  SPI3_VECTABNUM    0

/**
 * SPI CLK�ܽ�ö��
 */
typedef enum
{
	SPI0_CLK_P14_0 = 0x0000, SPI0_CLK_P15_2 = 0x0001,
	SPI1_CLK_P15_0 = 0x0100, SPI1_CLK_P20_10= 0x0101, SPI1_CLK_P33_11= 0x0102, SPI1_CLK_P33_12= 0x0103,
	SPI2_CLK_P02_4 = 0x0200, SPI2_CLK_P10_6 = 0x0201, SPI2_CLK_P14_2 = 0x0202, SPI2_CLK_P33_7 = 0x0203, SPI2_CLK_P33_9 = 0x0204,
	SPI3_CLK_P00_0 = 0x0300, SPI3_CLK_P00_2 = 0x0301, SPI3_CLK_P15_6 = 0x0302, SPI3_CLK_P15_8 = 0x0303, SPI3_CLK_P20_0 = 0x0304, SPI3_CLK_P21_5 = 0x0305
}SPI_CLK_t;

/**
 * SPI MISO/MRST �ܽ�ö��
 */
typedef enum
{
	SPI0_MISO_P14_1 = 0x0000, SPI0_MISO_P15_3 = 0x0001,
	SPI1_MISO_P15_1 = 0x0100, SPI1_MISO_P15_5 = 0x0101, SPI1_MISO_P20_9 = 0x0102,SPI1_MISO_P11_10 = 0x0104, SPI1_MISO_P33_13 = 0x0105, SPI1_MISO_P02_3 = 0x0106,
	SPI2_MISO_P14_3 = 0x0200, SPI2_MISO_P02_1 = 0x0201, SPI2_MISO_P10_6 = 0x0203, SPI2_MISO_P33_8  = 0x0204,SPI2_MISO_P02_0 = 0x0206,
	SPI3_MISO_P15_7 = 0x0300, SPI3_MISO_P20_3 = 0x0302,SPI3_MISO_P00_1  = 0x0304, SPI3_MISO_P21_6  = 0x0305,
}SPI_MISO_t;

/**
 * SPI MISO/MTSR �ܽ�ö��
 */
typedef enum
{
	SPI0_MOSI_P14_0 = 0x0000, SPI0_MOSI_P14_1 = 0x0001, SPI0_MOSI_P15_2 = 0x0002, SPI0_MOSI_P15_3 = 0x0003,
	SPI1_MOSI_P02_2 = 0x0100, SPI1_MOSI_P11_12= 0x0101, SPI1_MOSI_P14_10= 0x0102, SPI1_MOSI_P15_0 = 0x0103, SPI1_MOSI_P15_1 = 0x0104, SPI1_MOSI_P15_4 = 0x0105, SPI1_MOSI_P15_5 = 0x0106, SPI1_MOSI_P20_10= 0x0107, SPI1_MOSI_P33_12 = 0x0108,
	SPI2_MOSI_P10_5 = 0x0200, SPI2_MOSI_P14_2 = 0x0201, SPI2_MOSI_P14_3 = 0x0202, SPI2_MOSI_P33_8 = 0x0203, SPI2_MOSI_P33_9 = 0x0204,
	SPI3_MOSI_P00_0 = 0x0300, SPI3_MOSI_P00_1 = 0x0301, SPI3_MOSI_P15_6 = 0x0302, SPI3_MOSI_P15_7 = 0x0303, SPI3_MOSI_P20_0 = 0x0304, SPI3_MOSI_P20_3 = 0x0305
}SPI_MOSI_t;

/**
 * SPI Ƭѡ�ܽ�ö��
 */
typedef enum
{
	SPI0_CS_Null  = 0x0000,
	SPI1_CS_P14_3 = 0x0100,  SPI1_CS_P20_8 = 0x0101, SPI1_CS_P33_10= 0x0102,
	SPI2_CS_P02_3 = 0x0200,  SPI2_CS_P10_5 = 0x0201, SPI2_CS_P33_6 = 0x0202,
	SPI3_CS_P00_3 = 0x0300,  SPI3_CS_P14_3 = 0x0301, SPI3_CS_P21_2 = 0x0302, SPI3_CS_P21_6 = 0x0303
}SPI_CS_t;

/**
 * SPI ö��
 */
typedef enum
{
	SPI0,
	SPI1,
	SPI2,
	SPI3
}SPI_t;


#define SPI_GetModule(pin)    (uint8)(pin >> 8)
#define SPI_GetIndex(pin)    (uint8)(pin & 0x0f)

void SPI_InitConfig(SPI_CLK_t clkPin, SPI_MISO_t misoPin, SPI_MOSI_t mosiPin, SPI_CS_t csPin, uint32_t baudrate);
uint8 SPI_ReadWriteNByte(SPI_t spi, uint8_t *txData, uint8_t *rxData, uint32_t len);



#endif /* QSPI_DEMO_H_ */
