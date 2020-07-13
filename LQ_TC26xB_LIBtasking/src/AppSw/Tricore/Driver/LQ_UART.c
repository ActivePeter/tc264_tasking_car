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


#include <LQ_UART.h>
#include <LQ_GPIO.h>



#define ASC_TX_BUFFER_SIZE 64        //���ͻ���������
#define ASC_RX_BUFFER_SIZE 64        //���ջ���������


//����ͨ�Žṹ��
IfxAsclin_Asc g_UartConfig[4];

static uint8 s_AscTxBuffer[4][ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
static uint8 s_AscRxBuffer[4][ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];



/* UART�ж� */
IFX_INTERRUPT(UART0_RX_IRQHandler, UART0_VECTABNUM, UART0_RX_PRIORITY);
IFX_INTERRUPT(UART1_RX_IRQHandler, UART1_VECTABNUM, UART1_RX_PRIORITY);
IFX_INTERRUPT(UART2_RX_IRQHandler, UART2_VECTABNUM, UART2_RX_PRIORITY);
IFX_INTERRUPT(UART3_RX_IRQHandler, UART3_VECTABNUM, UART3_RX_PRIORITY);
IFX_INTERRUPT(UART0_TX_IRQHandler, UART0_VECTABNUM, UART0_TX_PRIORITY);
IFX_INTERRUPT(UART1_TX_IRQHandler, UART1_VECTABNUM, UART1_TX_PRIORITY);
IFX_INTERRUPT(UART2_TX_IRQHandler, UART2_VECTABNUM, UART2_TX_PRIORITY);
IFX_INTERRUPT(UART3_TX_IRQHandler, UART3_VECTABNUM, UART3_TX_PRIORITY);
IFX_INTERRUPT(UART0_ER_IRQHandler, UART0_VECTABNUM, UART0_ER_PRIORITY);
IFX_INTERRUPT(UART1_ER_IRQHandler, UART1_VECTABNUM, UART1_ER_PRIORITY);
IFX_INTERRUPT(UART2_ER_IRQHandler, UART2_VECTABNUM, UART2_ER_PRIORITY);
IFX_INTERRUPT(UART3_ER_IRQHandler, UART3_VECTABNUM, UART3_ER_PRIORITY);


/** UART�ж�CPU��� */
const uint8 UartIrqVectabNum[4] = {UART0_VECTABNUM, UART1_VECTABNUM, UART2_VECTABNUM, UART3_VECTABNUM};

/** UART�ж����ȼ� */
const uint8 UartIrqPriority[12] = {UART0_RX_PRIORITY, UART0_TX_PRIORITY, UART0_ER_PRIORITY, UART1_RX_PRIORITY, UART1_TX_PRIORITY, UART1_ER_PRIORITY,
		                           UART2_RX_PRIORITY, UART2_TX_PRIORITY, UART2_ER_PRIORITY, UART3_RX_PRIORITY, UART3_TX_PRIORITY, UART3_ER_PRIORITY};

/** UART�жϷ�������ַ */
const void *UartIrqFuncPointer[12] = {&UART0_RX_IRQHandler, &UART0_TX_IRQHandler, &UART0_ER_IRQHandler,
									   &UART1_RX_IRQHandler, &UART1_TX_IRQHandler, &UART1_ER_IRQHandler,
									   &UART2_RX_IRQHandler, &UART2_TX_IRQHandler, &UART2_ER_IRQHandler,
									   &UART3_RX_IRQHandler, &UART3_TX_IRQHandler, &UART3_ER_IRQHandler,};

/*************************************************************************
*  �������ƣ�void UART0_RX_IRQHandler(void)
*  ����˵����UART0_RX_IRQHandler�жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��30��
*  ��    ע��
*************************************************************************/
void UART0_RX_IRQHandler(void)
{
	IfxAsclin_Asc_isrReceive(&g_UartConfig[0]);

	/* �û����� */

}

void UART0_TX_IRQHandler(void)
{
	IfxAsclin_Asc_isrTransmit(&g_UartConfig[0]);

	/* �û����� */
}

void UART0_ER_IRQHandler(void)
{
	IfxAsclin_Asc_isrError(&g_UartConfig[0]);

	/* �û����� */
}

void UART1_RX_IRQHandler(void)
{
	IfxAsclin_Asc_isrReceive(&g_UartConfig[1]);

	/* �û����� */

}

void UART1_TX_IRQHandler(void)
{
	IfxAsclin_Asc_isrTransmit(&g_UartConfig[1]);

	/* �û����� */
}

void UART1_ER_IRQHandler(void)
{
	IfxAsclin_Asc_isrError(&g_UartConfig[1]);

	/* �û����� */
}

void UART2_RX_IRQHandler(void)
{
	IfxAsclin_Asc_isrReceive(&g_UartConfig[2]);

	/* �û����� */

}

void UART2_TX_IRQHandler(void)
{
	IfxAsclin_Asc_isrTransmit(&g_UartConfig[2]);

	/* �û����� */
}

void UART2_ER_IRQHandler(void)
{
	IfxAsclin_Asc_isrError(&g_UartConfig[2]);

	/* �û����� */
}

void UART3_RX_IRQHandler(void)
{
	IfxAsclin_Asc_isrReceive(&g_UartConfig[3]);

	/* �û����� */

}

void UART3_TX_IRQHandler(void)
{
	IfxAsclin_Asc_isrTransmit(&g_UartConfig[3]);

	/* �û����� */
}

void UART3_ER_IRQHandler(void)
{
	IfxAsclin_Asc_isrError(&g_UartConfig[3]);

	/* �û����� */
}

/*************************************************************************
*  �������ƣ�void UART_InitConfig(UART_RX_t RxPin, UART_TX_t TxPin, uint32 baudrate)
*  ����˵��������ģ���ʼ��
*  ����˵����
  * @param    RxPin   �� ���ڽ��չܽ�
  * @param    TxPin   �� ���ڷ��͹ܽ�
  * @param    baudrate�� ������
*  �������أ��ֽ�
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��UART_InitConfig(UART0_RX_P14_1,UART0_TX_P14_0, 115200);   //��ʼ������0 ������ 115200 ����żУ�� 1ֹͣλ ʹ�ùܽ�P14_0 P14_1
*************************************************************************/
void UART_InitConfig(UART_RX_t RxPin, UART_TX_t TxPin, uint32 baudrate)
{
	int i,j;
	//�ر�CPU�ж�
	IfxCpu_disableInterrupts();

	Ifx_P *portRx = PIN_GetModule(RxPin);
	uint8 pinIndexRx = PIN_GetIndex(RxPin);

	Ifx_P *portTx = PIN_GetModule(TxPin);
	uint8 pinIndexTx = PIN_GetIndex(TxPin);

	IfxAsclin_Rx_In  * IfxAsclin_Rx = NULL_PTR;
	IfxAsclin_Tx_Out * IfxAsclin_Tx = NULL_PTR;

	for( i = 0; i < IFXASCLIN_PINMAP_NUM_MODULES; i++)
	{
		for( j = 0; j < IFXASCLIN_PINMAP_RX_IN_NUM_ITEMS; j++)
		{
			if(IfxAsclin_Rx_In_pinTable[i][j] == NULL_PTR)
			{

			}
			else if((uint32)portRx == (uint32)(IfxAsclin_Rx_In_pinTable[i][j]->pin.port) && pinIndexRx == IfxAsclin_Rx_In_pinTable[i][j]->pin.pinIndex)
			{
				IfxAsclin_Rx = IfxAsclin_Rx_In_pinTable[i][j];
			}
		}

		for(j = 0; j < IFXASCLIN_PINMAP_TX_OUT_NUM_ITEMS; j++)
		{
			if(IfxAsclin_Tx_Out_pinTable[i][j] == NULL_PTR)
			{

			}
			else if((uint32)portTx == (uint32)(IfxAsclin_Tx_Out_pinTable[i][j]->pin.port) && pinIndexTx == IfxAsclin_Tx_Out_pinTable[i][j]->pin.pinIndex)
			{
				IfxAsclin_Tx = IfxAsclin_Tx_Out_pinTable[i][j];
			}
		}
	}

	if(IfxAsclin_Rx->module != IfxAsclin_Tx->module)
	{
#pragma warning 557         // ���ξ���
		while (1);          // ��� RX��TX �Ƿ�ΪͬһUART
#pragma warning default     // �򿪾���
	}

	//�½��������ýṹ��
	IfxAsclin_Asc_Config ascConfig;

	//��ʼ��ģ��
	IfxAsclin_Asc_initModuleConfig(&ascConfig, IfxAsclin_Tx->module);

	ascConfig.baudrate.baudrate  = (float)baudrate;        //������
	ascConfig.frame.frameMode    = IfxAsclin_FrameMode_asc;//����֡ģʽ
	ascConfig.frame.dataLength   = IfxAsclin_DataLength_8; //���ݳ���
	ascConfig.frame.stopBit      = IfxAsclin_StopBit_1;    //ֹͣλ
	ascConfig.frame.shiftDir     = IfxAsclin_ShiftDirection_lsbFirst;//��λ����
	ascConfig.frame.parityBit    = FALSE;//����żУ��

	uint8_t uartNum = IfxAsclin_getIndex(IfxAsclin_Tx->module);

	//�ж����ȼ�����
	ascConfig.interrupt.rxPriority = UartIrqPriority[uartNum * 3];
	ascConfig.interrupt.txPriority = UartIrqPriority[uartNum * 3 + 1];
	ascConfig.interrupt.erPriority = UartIrqPriority[uartNum * 3 + 2];
	ascConfig.interrupt.typeOfService = UartIrqVectabNum[uartNum];

	//���պͷ���FIFO������
	ascConfig.txBuffer     = &s_AscTxBuffer[uartNum][0];
	ascConfig.txBufferSize = ASC_TX_BUFFER_SIZE;
	ascConfig.rxBuffer     = &s_AscRxBuffer[uartNum][0];
	ascConfig.rxBufferSize = ASC_RX_BUFFER_SIZE;

	IfxAsclin_Asc_Pins pins =
	{
		NULL,                     IfxPort_InputMode_pullUp,        /* CTS pin not used */
		IfxAsclin_Rx,             IfxPort_InputMode_pullUp,        /* Rx pin */
		NULL,                     IfxPort_OutputMode_pushPull,     /* RTS pin not used */
		IfxAsclin_Tx,             IfxPort_OutputMode_pushPull,     /* Tx pin */
		IfxPort_PadDriver_cmosAutomotiveSpeed1
	};
	ascConfig.pins = &pins;

	//��������ṹ����Ԥ��Ĳ��������ģ��ĳ�ʼ��
	IfxAsclin_Asc_initModule(&g_UartConfig[uartNum], &ascConfig);

	//���գ����ͺʹ����ж�����
	IfxCpu_Irq_installInterruptHandler((void*)UartIrqFuncPointer[uartNum * 3],     UartIrqPriority[uartNum * 3]);
	IfxCpu_Irq_installInterruptHandler((void*)UartIrqFuncPointer[uartNum * 3 + 1], UartIrqPriority[uartNum * 3 + 1]);
	IfxCpu_Irq_installInterruptHandler((void*)UartIrqFuncPointer[uartNum * 3 + 2], UartIrqPriority[uartNum * 3 + 2]);

	//����CPU�ж�
	IfxCpu_enableInterrupts();
}

/*************************************************************************
*  �������ƣ�void UART_PutChar(UART_t  uratn, char ch)
*  ����˵����UART�����ֽں���,ʹ��ǰ���ȳ�ʼ����Ӧ����
*  ����˵����uratn �� UART0 - UART3/ ch    �� Ҫ��ӡ���ַ�
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��UART_PutChar(UART0, 'a');  //��ӡ�ַ�a
*************************************************************************/
void UART_PutChar(UART_t  uratn, char ch)
{
	IfxAsclin_Asc_blockingWrite(&g_UartConfig[uratn], ch);
}

/*************************************************************************
*  �������ƣ�void UART_PutStr(UART_t  uratn, char *str)
*  ����˵����UART�����ַ�������(�� NULL ֹͣ����),ʹ��ǰ���ȳ�ʼ����Ӧ����
*  ����˵����uratn �� UART0 - UART3/ str   �� Ҫ��ӡ���ַ�����ַ
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��UART_PutStr(UART3, "123456789"); //����9���ֽ�
*************************************************************************/
void UART_PutStr(UART_t  uratn, char *str)
{
   while(*str)
    {
        UART_PutChar(uratn, *str++);
    }
}

/*************************************************************************
*  �������ƣ�void UART_PutBuff(UART_t  uratn, uint8_t *buff, uint32 len)
*  ����˵����UART�����ֽں���,ʹ��ǰ���ȳ�ʼ����Ӧ����
*  ����˵����
* @param    uratn �� UART0 - UART3
* @param    buff  �� Ҫ��ӡ���ַ�����ַ
* @param    len   �� Ҫ��ӡ�ĳ���
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��UART_PutBuff(UART4, "123456789",5);//ʵ�ʷ���5���ֽڡ�1����2����3����4����5��
*************************************************************************/
void UART_PutBuff(UART_t  uratn, uint8_t *buff, uint32 len)
{
    while(len--)
    {
        UART_PutChar(uratn, *buff);
        buff++;
    }
}

/*************************************************************************
*  �������ƣ�Ifx_SizeT UART_GetCount(UART_t  uratn)
*  ����˵������ȡ ���ջ����� ��ŵ�ǰ�������ݸ���
*  ����˵���� uratn �� UART0 - UART3
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��
*************************************************************************/
uint8 UART_GetCount(UART_t  uratn)
{
	return (uint8)IfxAsclin_Asc_getReadCount(&g_UartConfig[uratn]);
}

/*************************************************************************
*  �������ƣ�char UART_GetChar(UART_t  uratn)
*  ����˵����UART��ȡ�ֽ� ʹ��ǰ���ȳ�ʼ����Ӧ���� ����ǰ��ȷ���н��յ����� �����ȴ����ݵ���
*  ����˵����uratn �� UART0 - UART3
*  �������أ���ȡ�ֽ�
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��UART_GetChar(UART3); //����һ���ַ�
*************************************************************************/
char UART_GetChar(UART_t  uratn)
{
	uint8 data=0;
	Ifx_SizeT count = 1;

	/* ��ȡ���յ���һ���ֽ����� ע�����û�н��յ�����һֱ�ȴ� */
	IfxAsclin_Asc_read(&g_UartConfig[uratn], &data, &count, TIME_INFINITE);

	return 	data;
}

/*************************************************************************
*  �������ƣ�char UART_GetBuff(UART_t  uratn, uint8_t *data, uint8_t len)
*  ����˵��������һ�������ַ� ʹ��ǰ���ȳ�ʼ����Ӧ���� ����ǰ��ȷ���н��յ����� ��������ʧ��
*  ����˵����uratn �� UART0 - UART3
*  �������أ�0:��ȡ����  ���� ��ȡʧ��
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��ART_GetChar(UART0, data, 10); //����10���ַ�
*************************************************************************/
char UART_GetBuff(UART_t  uratn, uint8_t *data, uint8_t len)
{
	Ifx_SizeT count = len;

	if(UART_GetCount(uratn) < len)
	{
		return 1;  //�жϵ�ǰ���յ����������� ������ȡ����
	}

	/* ��ȡ���յ�����  */
	IfxAsclin_Asc_read(&g_UartConfig[uratn], data, &count, TIME_INFINITE);
	return 	0;
}








/////////////////////////////////////////////////////////////////////////////////////
