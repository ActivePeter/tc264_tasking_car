/*!
  * @file     LQ_EMEM.c
  *
  * @brief    TC264 DA��׺��оƬ����EMEM��512K ram
  *
  * @company  �����������ܿƼ�
  *
  * @author   LQ-005
  *
  * @note     Tab�� 4���ո�
  *
  * @version  V1.0
  *
  * @par URL  http://shop36265907.taobao.com
  *           http://www.lqist.cn
  *
  * @date     2020��4��18��
  */ 



#include <IfxEmem.h>
#include "LQ_EMEM.h"

/*************************************************************************
*  �������ƣ�EMEM_InitConfig(void)
*  ����˵������ʼ��оƬ�ڲ� EMEM�ڴ�  512K
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��TC264 DA��׺��оƬ����EMEM��512K ram
*************************************************************************/
void EMEM_InitConfig(void)
{
	IfxEmem_setClockEnableState(IfxEmem_State_enabled);

	if (IfxEmem_LockedState_locked == IfxEmem_getLockedState())
	{
		/* apply unlock sequence */
		IfxEmem_setUnlockStandbyLockFlag(0x1);
		IfxEmem_setUnlockStandbyLockFlag(0x3);
		IfxEmem_setUnlockStandbyLockFlag(0x7);
		/* wait one cycle for unlock */

		for(int i = 0; i < 1000; i++)
		{
			__asm("NOP");
		}
	}

	IfxEmem_setTileConfigMode(IfxEmem_TileConfigMode_calibMode,IfxEmem_TileNumber_0);
	IfxEmem_setTileConfigMode(IfxEmem_TileConfigMode_calibMode,IfxEmem_TileNumber_1);
	IfxEmem_setTileConfigMode(IfxEmem_TileConfigMode_calibMode,IfxEmem_TileNumber_2);
	IfxEmem_setTileConfigMode(IfxEmem_TileConfigMode_calibMode,IfxEmem_TileNumber_3);
	IfxEmem_setTileConfigMode(IfxEmem_TileConfigMode_calibMode,IfxEmem_TileNumber_4);
	IfxEmem_setTileConfigMode(IfxEmem_TileConfigMode_calibMode,IfxEmem_TileNumber_5);
	IfxEmem_setTileConfigMode(IfxEmem_TileConfigMode_calibMode,IfxEmem_TileNumber_6);
	IfxEmem_setTileConfigMode(IfxEmem_TileConfigMode_calibMode,IfxEmem_TileNumber_7);
}
