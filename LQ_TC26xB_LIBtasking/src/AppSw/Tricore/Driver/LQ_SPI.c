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


#include <LQ_SPI.h>


/**
 *  ASC SPIͨ�Žṹ��
 */
IfxAsclin_Spi g_SpiConfig[4];

/** SPI�ж�CPU��� */
const uint8 SpiIrqVectabNum[4] = {SPI0_VECTABNUM, SPI1_VECTABNUM, SPI2_VECTABNUM, SPI3_VECTABNUM};

/** SPI�ж����ȼ� */
const uint8 SpiIrqPriority[12] = {SPI0_RX_PRIORITY, SPI0_TX_PRIORITY, SPI0_ER_PRIORITY, SPI1_RX_PRIORITY, SPI1_TX_PRIORITY, SPI1_ER_PRIORITY,
		                           SPI2_RX_PRIORITY, SPI2_TX_PRIORITY, SPI2_ER_PRIORITY, SPI3_RX_PRIORITY, SPI3_TX_PRIORITY, SPI3_ER_PRIORITY};


/***********************************************************************************************/
/***********************************************************************************************/
/*****************ASC SPI�ж����ȼ� �� ָ���ں����� �û����������޸�************************/
/***********************************************************************************************/
/***********************************************************************************************/
/**
  * @brief    SPI0_RX_IRQHandler�жϷ�����
  * @note     SPI0 �����жϷ�����
  */
void SPI0_RX_IRQHandler(void)
{
	IfxAsclin_Spi_isrReceive(&g_SpiConfig[0]);
	/* �û����� */
}

void SPI0_TX_IRQHandler(void)
{
	IfxAsclin_Spi_isrTransmit(&g_SpiConfig[0]);
	/* �û����� */
}

void SPI0_ER_IRQHandler(void)
{
	IfxAsclin_Spi_isrError(&g_SpiConfig[0]);
	/* �û����� */
}

void SPI1_RX_IRQHandler(void)
{
	IfxAsclin_Spi_isrReceive(&g_SpiConfig[1]);
	/* �û����� */
}

void SPI1_TX_IRQHandler(void)
{
	IfxAsclin_Spi_isrTransmit(&g_SpiConfig[1]);
	/* �û����� */
}

void SPI1_ER_IRQHandler(void)
{
	IfxAsclin_Spi_isrError(&g_SpiConfig[1]);
	/* �û����� */
}

void SPI2_RX_IRQHandler(void)
{
	IfxAsclin_Spi_isrReceive(&g_SpiConfig[2]);
	/* �û����� */
}

void SPI2_TX_IRQHandler(void)
{
	IfxAsclin_Spi_isrTransmit(&g_SpiConfig[2]);
	/* �û����� */
}

void SPI2_ER_IRQHandler(void)
{
	IfxAsclin_Spi_isrError(&g_SpiConfig[2]);
	/* �û����� */
}

void SPI3_RX_IRQHandler(void)
{
	IfxAsclin_Spi_isrReceive(&g_SpiConfig[3]);
	/* �û����� */
}

void SPI3_TX_IRQHandler(void)
{
	IfxAsclin_Spi_isrTransmit(&g_SpiConfig[3]);
	/* �û����� */
}

void SPI3_ER_IRQHandler(void)
{
	IfxAsclin_Spi_isrError(&g_SpiConfig[3]);
	/* �û����� */
}
/* SPI�ж� */
IFX_INTERRUPT(SPI0_RX_IRQHandler, SPI0_VECTABNUM, SPI0_RX_PRIORITY);
IFX_INTERRUPT(SPI1_RX_IRQHandler, SPI1_VECTABNUM, SPI1_RX_PRIORITY);
IFX_INTERRUPT(SPI2_RX_IRQHandler, SPI2_VECTABNUM, SPI2_RX_PRIORITY);
IFX_INTERRUPT(SPI3_RX_IRQHandler, SPI3_VECTABNUM, SPI3_RX_PRIORITY);
IFX_INTERRUPT(SPI0_TX_IRQHandler, SPI0_VECTABNUM, SPI0_TX_PRIORITY);
IFX_INTERRUPT(SPI1_TX_IRQHandler, SPI1_VECTABNUM, SPI1_TX_PRIORITY);
IFX_INTERRUPT(SPI2_TX_IRQHandler, SPI2_VECTABNUM, SPI2_TX_PRIORITY);
IFX_INTERRUPT(SPI3_TX_IRQHandler, SPI3_VECTABNUM, SPI3_TX_PRIORITY);
IFX_INTERRUPT(SPI0_ER_IRQHandler, SPI0_VECTABNUM, SPI0_ER_PRIORITY);
IFX_INTERRUPT(SPI1_ER_IRQHandler, SPI1_VECTABNUM, SPI1_ER_PRIORITY);
IFX_INTERRUPT(SPI2_ER_IRQHandler, SPI2_VECTABNUM, SPI2_ER_PRIORITY);
IFX_INTERRUPT(SPI3_ER_IRQHandler, SPI3_VECTABNUM, SPI3_ER_PRIORITY);

/** SPI�жϷ�������ַ */
const void *SpiIrqFuncPointer[12] = {&SPI0_RX_IRQHandler, &SPI0_TX_IRQHandler, &SPI0_ER_IRQHandler,
									&SPI1_RX_IRQHandler, &SPI1_TX_IRQHandler, &SPI1_ER_IRQHandler,
									&SPI2_RX_IRQHandler, &SPI2_TX_IRQHandler, &SPI2_ER_IRQHandler,
									&SPI3_RX_IRQHandler, &SPI3_TX_IRQHandler, &SPI3_ER_IRQHandler,};
/*************************************************************************
*  �������ƣ�void SPI_InitConfig(SPI_CLK_t clkPin, SPI_MISO_t misoPin, SPI_MOSI_t mosiPin, SPI_CS_t csPin, uint32_t baudrate)
*  ����˵����SPI ����ģʽ��ʼ��
*  ����˵����
  * @param    clkPin      �� SPI  clk�ܽ�
  * @param    misoPin     :  SPI  MISO�ܽ�
  * @param    misoPin     :  SPI  MOSI�ܽ�
  * @param    misoPin     :  SPI  CS�ܽ�
  * @param    baudrate    �� SPI ������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע������ʵ���ǽ�Asc������SPI�����ʹ����Asc0����SPI0����ôAsc0 �Ͳ�����������0
*  			SPI_InitConfig(SPI3_CLK_P15_6, SPI3_MISO_P15_7, SPI3_MOSI_P20_0, SPI3_CS_P21_2, 10000000U); //��ʼ��SPI3 10M����
*************************************************************************/
void SPI_InitConfig(SPI_CLK_t clkPin, SPI_MISO_t misoPin, SPI_MOSI_t mosiPin, SPI_CS_t csPin, uint32_t baudrate)
{
	uint8_t spiNum = SPI_GetModule(clkPin);

	IfxAsclin_Sclk_Out * IfxAsclin_Sclk = IfxAsclin_Sclk_Out_pinTable[spiNum][SPI_GetIndex(clkPin)];
	IfxAsclin_Rx_In    * IfxAsclin_MISO = IfxAsclin_Rx_In_pinTable[spiNum][SPI_GetIndex(misoPin)];
	IfxAsclin_Tx_Out   * IfxAsclin_MOSI = IfxAsclin_Tx_Out_pinTable[spiNum][SPI_GetIndex(mosiPin)];
	IfxAsclin_Slso_Out * IfxAsclin_CS   = IfxAsclin_Slso_Out_pinTable[spiNum][SPI_GetIndex(csPin)];


    /* ���ж�  */
    boolean              interruptState = IfxCpu_disableInterrupts();

    /* create module config */
    IfxAsclin_Spi_Config spiConfig;
    IfxAsclin_Spi_initModuleConfig(&spiConfig, IfxAsclin_Sclk->module);

    spiConfig.frame.shiftDir        = IfxAsclin_ShiftDirection_msbFirst;/* MSB ��ǰ */

    /* set the desired baudrate */
    spiConfig.baudrate.prescaler    = 1;
    spiConfig.baudrate.baudrate     = (float)baudrate;                 /* spi �ٶ� */
    spiConfig.baudrate.oversampling = IfxAsclin_OversamplingFactor_4;

    spiConfig.inputOutput.cpol      = IfxAsclin_ClockPolarity_idleHigh; /* clk ����״̬�ߵ�ƽ */

    /* ISR priorities and interrupt target */
    spiConfig.interrupt.rxPriority    = SpiIrqPriority[spiNum * 3];     /* see also \ref IfxLld_Asclin */
    spiConfig.interrupt.txPriority    = SpiIrqPriority[spiNum * 3 + 1]; /* see also \ref IfxLld_Asclin */
    spiConfig.interrupt.erPriority    = SpiIrqPriority[spiNum * 3 + 2]; /* see also \ref IfxLld_Asclin */
    spiConfig.interrupt.typeOfService = (IfxSrc_Tos)SpiIrqVectabNum[spiNum];

    /* pin configuration */
    const IfxAsclin_Spi_Pins pins = {
    		IfxAsclin_Sclk, IfxPort_OutputMode_pushPull, /* Clock out pin */
			IfxAsclin_MISO,   IfxPort_InputMode_pullUp,    /* Rx pin */
			IfxAsclin_MOSI,   IfxPort_OutputMode_pushPull, /* Tx pin */
			IfxAsclin_CS, IfxPort_OutputMode_pushPull, /* Slave select pin */
        IfxPort_PadDriver_cmosAutomotiveSpeed1
    };
    spiConfig.pins = &pins;

    /* initialize module */
    IfxAsclin_Spi_initModule(&g_SpiConfig[spiNum], &spiConfig);

    //���գ����ͺʹ����ж�����
	IfxCpu_Irq_installInterruptHandler((void*)SpiIrqFuncPointer[spiNum * 3],     (uint32)SpiIrqPriority[spiNum * 3]);
	IfxCpu_Irq_installInterruptHandler((void*)SpiIrqFuncPointer[spiNum * 3 + 1], (uint32)SpiIrqPriority[spiNum * 3 + 1]);
	IfxCpu_Irq_installInterruptHandler((void*)SpiIrqFuncPointer[spiNum * 3 + 2], (uint32)SpiIrqPriority[spiNum * 3 + 2]);

    /* enable interrupts again */
    IfxCpu_restoreInterrupts(interruptState);
}

/*************************************************************************
*  �������ƣ�uint8 SPI_ReadWriteNByte(SPI_t spi, uint8_t *txData, uint8_t *rxData, uint32_t len)
*  ����˵����SPI��дN���ֽ�
*  ����˵����
  * @param    spi     ��  SPI0 - SPI3
  * @param    txData  ��  д���buff
  * @param    rxData  ��  ������buff
  * @param    len     ��  ����
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע������ʵ���ǽ�Asc������SPI�����ʹ����Asc0����SPI0����ôAsc0 �Ͳ�����������0
*  			SPI_ReadWriteNByte(SPI3, buff, buff, 2); //ʹ��SPI3 ��д�����ֽ�
*************************************************************************/
uint8 SPI_ReadWriteNByte(SPI_t spi, uint8_t *txData, uint8_t *rxData, uint32_t len)
{
	/* SPI����ṹ�� */
    IfxAsclin_Spi_exchange(&g_SpiConfig[spi], txData, rxData, len);

    /* �ȴ��������  */
	while (IfxAsclin_Spi_getStatus(&g_SpiConfig[spi]) == IfxAsclin_Spi_Status_busy);

	return 0;
}



/////////////////////////////////////////////////////////////////////////////////////
