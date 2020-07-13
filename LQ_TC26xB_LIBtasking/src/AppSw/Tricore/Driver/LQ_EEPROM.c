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

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ
eeprom һ��96Kb ��Ϊ12������   ÿ������8K  ÿ��������Ϊ1024ҳ   ÿҳ8���ֽ�
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <LQ_EEPROM.h>


/*************************************************************************
*  �������ƣ�void EEPROM_EraseSector(unsigned char sector)
*  ����˵����eeprom��������
*  ����˵����sector   ��  ������Χ  0-11
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��EEPROM_EraseSector(0);   //��������0
*************************************************************************/
void EEPROM_EraseSector(unsigned char sector)
{
	/* ����������ʼ��ַ */
	unsigned long sector_addr = IfxFlash_dFlashTableEepLog[sector].start;

	unsigned short endinitSfty_pw;

	endinitSfty_pw = IfxScuWdt_getSafetyWatchdogPassword();

	/* �������� */
	IfxScuWdt_clearSafetyEndinit(endinitSfty_pw);
	IfxFlash_eraseSector(sector_addr);
	IfxScuWdt_setSafetyEndinit(endinitSfty_pw);

	IfxFlash_waitUnbusy(0, IfxFlash_FlashType_D0);
}

/*************************************************************************
*  �������ƣ�void EEPROM_EraseSector(unsigned char sector)
*  ����˵����eepromд������
*  ����˵����
  * @param    sector   ��  ����   ��Χ  0-11
  * @param    page     ��  ҳ     ��Χ  0-1023
  * @param    buff     ��  ���д������
  * @param    len      ��  д�����ݸ���
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��EEPROM_Write(0, 0, u32wBuff, 24);   //������0  ��0ҳ  д��24��unsigned long����
*************************************************************************/
void EEPROM_Write(unsigned char sector, unsigned short page, unsigned long * buff, unsigned short len)
{
	int i;
	/* ����������ʼ��ַ */
	unsigned long sector_addr = IfxFlash_dFlashTableEepLog[sector].start;

	unsigned short endinitSfty_pw;

	endinitSfty_pw = IfxScuWdt_getSafetyWatchdogPassword();

	for(i = 0; i < len/2; i++)
	{
		/* Ҫд��ҳ��ַ */
		unsigned long pageAddr = sector_addr + (page + i)* IFXFLASH_DFLASH_PAGE_LENGTH;

		IfxFlash_enterPageMode(pageAddr);

		/* ��æ */
		IfxFlash_waitUnbusy(0, IfxFlash_FlashType_D0);

		/* д�뻺����  */
		IfxFlash_loadPage2X32(pageAddr, buff[2*i], buff[2*i + 1]);

		/* д��eeprom  */
		IfxScuWdt_clearSafetyEndinit(endinitSfty_pw);
		IfxFlash_writePage(pageAddr);
		IfxScuWdt_setSafetyEndinit(endinitSfty_pw);

		IfxFlash_waitUnbusy(0, IfxFlash_FlashType_D0);
	}

	if(len%2)
	{
		/* Ҫд��ҳ��ַ */
		unsigned long pageAddr = sector_addr + (page + len/2)* IFXFLASH_DFLASH_PAGE_LENGTH;

		IfxFlash_enterPageMode(pageAddr);

		/* ��æ */
		IfxFlash_waitUnbusy(0, IfxFlash_FlashType_D0);

		/* д�뻺����  */
		IfxFlash_loadPage2X32(pageAddr, buff[len], 0);

		/* д��eeprom  */
		IfxScuWdt_clearSafetyEndinit(endinitSfty_pw);
		IfxFlash_writePage(pageAddr);
		IfxScuWdt_setSafetyEndinit(endinitSfty_pw);

		IfxFlash_waitUnbusy(0, IfxFlash_FlashType_D0);

	}
}


/*************************************************************************
*  �������ƣ�void EEPROM_Read(unsigned char sector, unsigned short page, unsigned long * rbuff, unsigned short len)
*  ����˵����eeprom��ȡ
*  ����˵����
  * @param    sector   ��  ����   ��Χ  0-11
  * @param    page     ��  ҳ     ��Χ  0-1023
  * @param    buff     ��  ���д������
  * @param    len      ��  д�����ݸ���
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��EEPROM_Read(0, 0, u32rBuff, 24);   //��ȡ����0  ��0ҳ  ��ʼ��24��unsigned long����
*************************************************************************/
void EEPROM_Read(unsigned char sector, unsigned short page, unsigned long * rbuff, unsigned short len)
{
	int i;
	/* ����������ʼ��ַ */
	unsigned long sector_addr = IfxFlash_dFlashTableEepLog[sector].start;

	/* Ҫ��ҳ��ַ */
	volatile unsigned long * pageAddr = (unsigned long*)(sector_addr + page * IFXFLASH_DFLASH_PAGE_LENGTH);

	for(i = 0; i < len; i++)
	{
		rbuff[i] = *(pageAddr++);
	}
}





