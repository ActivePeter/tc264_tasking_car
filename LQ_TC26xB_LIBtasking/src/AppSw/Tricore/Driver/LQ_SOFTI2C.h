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
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef __LQ_SOFTIIC_H
#define __LQ_SOFTIIC_H

#include "Cpu/Std/Platform_Types.h"
#include "stdint.h"
#include "LQ_GPIO.h"

/* ע�� IIC���߹涨��IIC����ʱ SCL��SDA��Ϊ�ߵ�ƽ ����ⲿ������һ������������ */
/* ģ�� IIC��Ҫע�⣬IIC��ַ����һλ ����MPU6050 ģ����ǵ�ַ 0xD0 */
/* �뻻�ñ��IO ֱ���޸ĺ궨�� SOFT_IIC_SCL_PIN �� SOFT_IIC_SDA_PIN ���� */
#define SOFT_IIC_SCL_PIN   P13_1   /*!< P13_1  ��Ϊ SCL */
#define SOFT_IIC_SDA_PIN   P13_2   /*!< P13_2  ��Ϊ SDA */

#define SDA_OUT        PIN_Dir(SOFT_IIC_SDA_PIN, 1);
#define SDA_IN         PIN_Dir(SOFT_IIC_SDA_PIN, 0);

#define IIC_SCL_INIT   PIN_InitConfig(SOFT_IIC_SCL_PIN, PIN_MODE_OUTPUT, 1);
#define IIC_SDA_INIT   PIN_InitConfig(SOFT_IIC_SDA_PIN, PIN_MODE_OUTPUT, 1);

#define IIC_SCL_H      PIN_Write(SOFT_IIC_SCL_PIN, 1);
#define IIC_SCL_L      PIN_Write(SOFT_IIC_SCL_PIN, 0);

#define IIC_SDA_H      PIN_Write(SOFT_IIC_SDA_PIN, 1);
#define IIC_SDA_L      PIN_Write(SOFT_IIC_SDA_PIN, 0);

#define IIC_SDA_READ   PIN_Read(SOFT_IIC_SDA_PIN)

void IIC_Start(void);			        //����IIC��ʼ�ź�
void IIC_Stop(void);	  	            //����IICֹͣ�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				    //IIC������ACK�ź�
uint8 IIC_WaitAck(void); 		        //IIC�ȴ�ACK�ź�
void IIC_SendByte(uint8 data_t);      //IIC����һ���ֽ�
uint8 IIC_ReadByte(uint8 ack);        //IIC��ȡһ���ֽ�


void IIC_Init(void);
uint8 IIC_ReadByteFromSlave(uint8 I2C_Addr,uint8 reg,uint8 *buf);
uint8 IIC_ReadMultByteFromSlave(uint8 dev, uint8 reg, uint8 length, uint8 *data_t);
uint8 IIC_WriteByteToSlave(uint8 I2C_Addr,uint8 reg,uint8 buf);
uint8 IIC_WriteMultByteToSlave(uint8 dev, uint8 reg, uint8 length, uint8* data_t);

#endif
