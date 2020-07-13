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


#include <LQ_QSPI.h>



/** QSPI�ж�CPU��� */
const uint8 QSpiIrqVectabNum[4] = {QSPI0_VECTABNUM, QSPI1_VECTABNUM, QSPI2_VECTABNUM, QSPI3_VECTABNUM};

/** QSPI�ж����ȼ� */
const uint8 QSpiIrqPriority[12] = {QSPI0_RX_PRIORITY, QSPI0_TX_PRIORITY, QSPI0_ER_PRIORITY, QSPI1_RX_PRIORITY, QSPI1_TX_PRIORITY, QSPI1_ER_PRIORITY,
		                           QSPI2_RX_PRIORITY, QSPI2_TX_PRIORITY, QSPI2_ER_PRIORITY, QSPI3_RX_PRIORITY, QSPI3_TX_PRIORITY, QSPI3_ER_PRIORITY};

/** QSPI ���ýṹ�� */
IfxQspi_SpiMaster         g_QSPIConfig[4];
IfxQspi_SpiMaster_Channel g_QspiMasterChannel[4];
/***********************************************************************************************/
/***********************************************************************************************/
/*****************    QSPI�ж����ȼ� �� ָ���ں����� �û����������޸�*******************************/
/***********************************************************************************************/
/***********************************************************************************************/
/**
  * @brief    QSPI0_RX_IRQHandler�жϷ�����
  * @note     QSPI0 �����жϷ�����
  */
void QSPI0_RX_IRQHandler(void)
{
	IfxQspi_SpiMaster_isrReceive(&g_QSPIConfig[0]);

	/* �û����� */
}

void QSPI0_TX_IRQHandler(void)
{
	IfxQspi_SpiMaster_isrTransmit(&g_QSPIConfig[0]);
	/* �û����� */
}

void QSPI0_ER_IRQHandler(void)
{
	IfxQspi_SpiMaster_isrError(&g_QSPIConfig[0]);
	/* �û����� */
}

void QSPI1_RX_IRQHandler(void)
{
	IfxQspi_SpiMaster_isrReceive(&g_QSPIConfig[1]);
	/* �û����� */
}

void QSPI1_TX_IRQHandler(void)
{
	IfxQspi_SpiMaster_isrTransmit(&g_QSPIConfig[1]);
	/* �û����� */
}

void QSPI1_ER_IRQHandler(void)
{
	IfxQspi_SpiMaster_isrError(&g_QSPIConfig[1]);
	/* �û����� */
}

void QSPI2_RX_IRQHandler(void)
{
	IfxQspi_SpiMaster_isrReceive(&g_QSPIConfig[2]);
	/* �û����� */
}

void QSPI2_TX_IRQHandler(void)
{
	IfxQspi_SpiMaster_isrTransmit(&g_QSPIConfig[2]);
	/* �û����� */
}

void QSPI2_ER_IRQHandler(void)
{
	IfxQspi_SpiMaster_isrError(&g_QSPIConfig[2]);
	/* �û����� */
}

void QSPI3_RX_IRQHandler(void)
{
	IfxQspi_SpiMaster_isrReceive(&g_QSPIConfig[3]);
	/* �û����� */
}

void QSPI3_TX_IRQHandler(void)
{
	IfxQspi_SpiMaster_isrTransmit(&g_QSPIConfig[3]);
	/* �û����� */
}

void QSPI3_ER_IRQHandler(void)
{
	IfxQspi_SpiMaster_isrError(&g_QSPIConfig[3]);
	/* �û����� */
}

/* QSPI�ж� */
IFX_INTERRUPT(QSPI0_RX_IRQHandler, QSPI0_VECTABNUM, QSPI0_RX_PRIORITY);
IFX_INTERRUPT(QSPI1_RX_IRQHandler, QSPI1_VECTABNUM, QSPI1_RX_PRIORITY);
IFX_INTERRUPT(QSPI2_RX_IRQHandler, QSPI2_VECTABNUM, QSPI2_RX_PRIORITY);
IFX_INTERRUPT(QSPI3_RX_IRQHandler, QSPI3_VECTABNUM, QSPI3_RX_PRIORITY);
IFX_INTERRUPT(QSPI0_TX_IRQHandler, QSPI0_VECTABNUM, QSPI0_TX_PRIORITY);
IFX_INTERRUPT(QSPI1_TX_IRQHandler, QSPI1_VECTABNUM, QSPI1_TX_PRIORITY);
IFX_INTERRUPT(QSPI2_TX_IRQHandler, QSPI2_VECTABNUM, QSPI2_TX_PRIORITY);
IFX_INTERRUPT(QSPI3_TX_IRQHandler, QSPI3_VECTABNUM, QSPI3_TX_PRIORITY);
IFX_INTERRUPT(QSPI0_ER_IRQHandler, QSPI0_VECTABNUM, QSPI0_ER_PRIORITY);
IFX_INTERRUPT(QSPI1_ER_IRQHandler, QSPI1_VECTABNUM, QSPI1_ER_PRIORITY);
IFX_INTERRUPT(QSPI2_ER_IRQHandler, QSPI2_VECTABNUM, QSPI2_ER_PRIORITY);
IFX_INTERRUPT(QSPI3_ER_IRQHandler, QSPI3_VECTABNUM, QSPI3_ER_PRIORITY);

/** QSPI�жϷ�������ַ */
const void *QSPIIrqFuncPointer[12] = {&QSPI0_RX_IRQHandler, &QSPI0_TX_IRQHandler, &QSPI0_ER_IRQHandler,
									&QSPI1_RX_IRQHandler, &QSPI1_TX_IRQHandler, &QSPI1_ER_IRQHandler,
									&QSPI2_RX_IRQHandler, &QSPI2_TX_IRQHandler, &QSPI2_ER_IRQHandler,
									&QSPI3_RX_IRQHandler, &QSPI3_TX_IRQHandler, &QSPI3_ER_IRQHandler,};
/*************************************************************************
*  �������ƣ�QSPI_InitConfig(QSPI_CLK_t clkPin, QSPI_MISO_t misoPin, QSPI_MOSI_t mosiPin, QSPI_CS_t cQSPIn, uint32_t baudrate, uint8_t mode)
*  ����˵����QSPI ����ģʽ��ʼ��
*  ����˵����
  * @param    clkPin      �� QSPI  clk�ܽ�
  * @param    misoPin     :  QSPI  MISO�ܽ�
  * @param    misoPin     :  QSPI  MOSI�ܽ�
  * @param    misoPin     :  QSPI  CS�ܽ�
  * @param    baudrate    �� QSPI ������
  * @param      mode            0: CLK����ʱ��Ϊ�͵�ƽ �ڵ�һ��CLK������ ��������
  * @param      mode            1: CLK����ʱ��Ϊ�͵�ƽ �ڵڶ���CLK������ ��������
  * @param      mode            2: CLK����ʱ��Ϊ�ߵ�ƽ �ڵ�һ��CLK������ ��������
  * @param      mode            3: CLK����ʱ��Ϊ�ߵ�ƽ �ڵڶ���CLK������ ��������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��QSPI_InitConfig(QSPI2_CLK_P15_8, QSPI2_MISO_P15_7, QSPI2_MOSI_P15_5, QSPI2_CS_P15_2, 5000000, 3);
*
*************************************************************************/
void QSPI_InitConfig(QSPI_CLK_t clkPin, QSPI_MISO_t misoPin, QSPI_MOSI_t mosiPin, QSPI_CS_t cQSPIn, uint32_t baudrate, uint8_t mode)
{
	uint8_t QSPINum   = QSPI_GetModule(clkPin);

	IfxQspi_Sclk_Out   * QSPI_Sclk = IfxQspi_Sclk_Out_pinTable[QSPINum][QSPI_GetIndex(clkPin)];
	IfxQspi_Mrst_In    * QSPI_MISO = IfxQspi_Mrst_In_pinTable[QSPINum][QSPI_GetIndex(misoPin)];
	IfxQspi_Mtsr_Out   * QSPI_MOSI = IfxQspi_Mtsr_Out_pinTable[QSPINum][QSPI_GetIndex(mosiPin)];
	IfxQspi_Slso_Out   * QSPI_CS   = IfxQspi_Slso_Out_pinTable[QSPINum][QSPI_GetIndex(cQSPIn)][0];

	IfxQspi_SpiMaster_Config        spiMasterConfig;
	IfxQspi_SpiMaster_ChannelConfig spiMasterChannelConfig;


	{
		/* create module config */
		IfxQspi_SpiMaster_initModuleConfig(&spiMasterConfig, IfxQspi_getAddress((IfxQspi_Index)QSPINum));

		/* set the maximum baudrate */
		spiMasterConfig.base.maximumBaudrate = 10000000;

		/* ISR priorities and interrupt target */
		spiMasterConfig.base.rxPriority  = QSpiIrqPriority[QSPINum * 3];
		spiMasterConfig.base.txPriority  = QSpiIrqPriority[QSPINum * 3 + 1];
		spiMasterConfig.base.erPriority  = QSpiIrqPriority[QSPINum * 3 + 2];
		spiMasterConfig.base.isrProvider = (IfxSrc_Tos)QSpiIrqVectabNum[QSPINum];

		/* pin configuration */
		const IfxQspi_SpiMaster_Pins pins = {QSPI_Sclk,                               /* SCLK */
											 IfxPort_OutputMode_pushPull,
											 QSPI_MOSI,  IfxPort_OutputMode_pushPull, /* MTSR */
											 QSPI_MISO,  IfxPort_InputMode_pullDown,  /* MRST */
											 IfxPort_PadDriver_cmosAutomotiveSpeed3                   /* pad driver mode */
		};
		spiMasterConfig.pins = &pins;

		/* initialize module */
		IfxQspi_SpiMaster_initModule(&g_QSPIConfig[QSPINum], &spiMasterConfig);
	}

	{
		/* create channel config */
		IfxQspi_SpiMaster_initChannelConfig(&spiMasterChannelConfig,
			&g_QSPIConfig[QSPINum]);




		const IfxQspi_SpiMaster_Output slsOutput = {QSPI_CS,
													IfxPort_OutputMode_pushPull,
													IfxPort_PadDriver_cmosAutomotiveSpeed1};

		spiMasterChannelConfig.sls.output.pin    = slsOutput.pin;
		spiMasterChannelConfig.sls.output.mode   = slsOutput.mode;
		spiMasterChannelConfig.sls.output.driver = slsOutput.driver;
		spiMasterChannelConfig.base.mode.csInactiveDelay = 100;
		spiMasterChannelConfig.base.mode.csLeadDelay     = 100;                    /* CSǰ����ʱ */
		spiMasterChannelConfig.base.mode.csTrailDelay    = 100;
		spiMasterChannelConfig.base.mode.dataHeading = SpiIf_DataHeading_msbFirst;  /* MSB��ǰ      */
		spiMasterChannelConfig.base.mode.dataWidth = 8;                             /* 8bit     */
		spiMasterChannelConfig.base.baudrate = (float)baudrate;                     /* �����ٶ�    */
		spiMasterChannelConfig.base.mode.csActiveLevel = Ifx_ActiveState_low;       /* CSΪ����Ч*/

	    if(mode / 2)
	    {
	    	spiMasterChannelConfig.base.mode.clockPolarity = SpiIf_ClockPolarity_idleHigh;  //ʱ�ӿ���ʱΪ�ߵ�ƽ
	    }
	    else
	    {
	    	spiMasterChannelConfig.base.mode.clockPolarity = SpiIf_ClockPolarity_idleLow;   //ʱ�ӿ���ʱΪ�͵�ƽ
	    }

	    if(mode % 2)
	    {
	    	spiMasterChannelConfig.base.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnTrailingEdge;  //����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	    }
	    else
	    {
	    	spiMasterChannelConfig.base.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnLeadingEdge;  //����ͬ��ʱ�ӵĵ�һ�������أ��������½������ݱ�����
	    }
	    //���գ����ͺʹ����ж�����
		IfxCpu_Irq_installInterruptHandler((void*)QSPIIrqFuncPointer[QSPINum * 3],     (uint32)QSpiIrqPriority[QSPINum * 3]);
		IfxCpu_Irq_installInterruptHandler((void*)QSPIIrqFuncPointer[QSPINum * 3 + 1], (uint32)QSpiIrqPriority[QSPINum * 3 + 1]);
		IfxCpu_Irq_installInterruptHandler((void*)QSPIIrqFuncPointer[QSPINum * 3 + 2], (uint32)QSpiIrqPriority[QSPINum * 3 + 2]);


		IfxQspi_SpiMaster_initChannel(&g_QspiMasterChannel[QSPINum],
			&spiMasterChannelConfig);
	}

}

/*************************************************************************
*  �������ƣ�uint8 QSPI_ReadWriteNByte(QSPI_t QSPI, uint8_t *txData, uint8_t *rxData, uint32_t len)
*  ����˵����QSPI��дN���ֽ�
*  ����˵����
  * @param    QSPI     ��  QSPI0 - QSPI3
  * @param    txData  ��  д���buff
  * @param    rxData  ��  ������buff
  * @param    len     ��  ����
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��
*  			QSPI_ReadWriteNByte(QSPI3, buff, buff, 2); //ʹ��QSPI3 ��д�����ֽ�
*************************************************************************/
uint8 QSPI_ReadWriteNByte(QSPI_t QSPI, uint8_t *txData, uint8_t *rxData, uint16 len)
{
	/* ��ʼ���� */
	IfxQspi_SpiMaster_exchange(&g_QspiMasterChannel[QSPI], txData, rxData, len);

	/* �ȴ��������  */
	while (IfxQspi_SpiMaster_getStatus(&g_QspiMasterChannel[QSPI]) == SpiIf_Status_busy) ;

	return 0;
}



/////////////////////////////////////////////////////////////////////////////////////
