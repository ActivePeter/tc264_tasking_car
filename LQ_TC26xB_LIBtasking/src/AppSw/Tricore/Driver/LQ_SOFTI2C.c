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

#include <Platform_Types.h>
#include "LQ_SOFTI2C.h"

/*************************************************************************
*  �������ƣ�IIC��ʱ
*  ����˵����ADC��ʼ������
*  ����˵����us      ��  ��ʱʱ��
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  Ӧ�þ�����iic_delay(1);  //�򵥵���ʱ
*  �ڲ����� �޸�������Ե���IIC����
*************************************************************************/
void iic_delay()
{
	/* 200MHz ϵͳʱ���� ģ��IIC�ٶ�Ϊ 400Khz */
	
	uint8  i = 0;
	for(i = 0; i < 30; i++) //�޸�������Ե���IIC����
	{
		__asm("NOP"); /* delay */
	}
}

/*************************************************************************
*  �������ƣ�void IIC_Init(void)
*  ����˵����ģ��IIC��ʼ��
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  Ӧ�þ�����IIC_Init();  //ģ��IIC��ʼ�� IIC�ܽ���LQ_SOFTI2C.h�ж���
*************************************************************************/
void IIC_Init(void)
{			
    IIC_SCL_INIT;
    IIC_SDA_INIT;
    IIC_SCL_H; 
    IIC_SDA_H;
}

/*************************************************************************
*  �������ƣ�void IIC_Start(void)
*  ����˵����ģ��IIC��ʼ�ź�
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  Ӧ�þ�����IIC_Start();
*************************************************************************/
void IIC_Start(void)
{
	SDA_OUT;   //sda����� 
	IIC_SDA_H;	
	IIC_SCL_H;
	iic_delay();
	iic_delay();
	iic_delay();
	iic_delay();
	iic_delay();
 	IIC_SDA_L; //START:when CLK is high,DATA change form high to low 
	iic_delay();
	iic_delay();
	iic_delay();
	IIC_SCL_L; //ǯסI2C���ߣ�׼�����ͻ�������� 
}


/*************************************************************************
*  �������ƣ�void IIC_Stop(void)
*  ����˵����ģ��IICֹͣ�ź�
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  Ӧ�þ�����IIC_Stop();
*************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT; //sda�����
	IIC_SCL_L;
	IIC_SDA_L; //STOP:when CLK is high DATA change form low to high
    iic_delay();
	iic_delay();
	iic_delay();
	IIC_SCL_H; 
    iic_delay();
	iic_delay();
	iic_delay();
	IIC_SDA_H; //����I2C���߽����ź�
    iic_delay();							   	
}


/*************************************************************************
*  �������ƣ�uint8 IIC_WaitAck(void)
*  ����˵����ģ��IIC�ȴ�Ӧ���ź�
*  ����˵������
*  �������أ�1������Ӧ��ʧ��    0������Ӧ��ɹ�
*  �޸�ʱ�䣺2020��3��10��
*  Ӧ�þ������ڲ����� ��ЧӦ�𣺴ӻ���9�� SCL=0 ʱ SDA ���ӻ�����,���� SCL = 1ʱ SDA��ȻΪ��
*************************************************************************/
uint8 IIC_WaitAck(void)
{
	uint8  ucErrTime=0;
	SDA_IN; //SDA����Ϊ����  ���ӻ���һ���͵�ƽ��ΪӦ�� 
	IIC_SDA_H;iic_delay();	   
	IIC_SCL_H;iic_delay();	 
	while(IIC_SDA_READ)
	{
		ucErrTime++;
		if(ucErrTime>100)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L; //ʱ�����0 	   
	return 0;  
} 

/*************************************************************************
*  �������ƣ�void IIC_Ack(void)
*  ����˵����ģ��IIC����ACKӦ��
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  Ӧ�þ������ڲ����� ����������һ���ֽ����ݺ�����������ACK֪ͨ�ӻ�һ���ֽ���������ȷ����
*************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL_L;
	SDA_OUT;
	IIC_SDA_L;
	iic_delay();
	iic_delay();
	iic_delay();
	IIC_SCL_H;
	iic_delay();
	iic_delay();
	iic_delay();
	IIC_SCL_L;
}


/*************************************************************************
*  �������ƣ�void IIC_NAck(void)
*  ����˵����ģ��IIC������ACKӦ��
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  Ӧ�þ������ڲ����� �������������һ���ֽ����ݺ�����������NACK֪ͨ�ӻ����ͽ������ͷ�SDA,�Ա���������ֹͣ�ź�
*************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL_L;
	SDA_OUT;
	IIC_SDA_H;
	iic_delay();
	iic_delay();
	iic_delay();
	IIC_SCL_H;
	iic_delay();
	iic_delay();
	iic_delay();
	IIC_SCL_L;
}					 				     


/*************************************************************************
*  �������ƣ�void IIC_SendByte(uint8 data_t)
*  ����˵����ģ��IIC����һ���ֽ�
*  ����˵����data   :  ���͵��ֽ�
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  Ӧ�þ�����IIC_SendByte(0x12);
*************************************************************************/
void IIC_SendByte(uint8 data_t)
{                        
    uint8  t;
    SDA_OUT; 	    
    IIC_SCL_L; //����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {   
//        IIC_SDA_READ = data_t&0x80;	
        if(data_t&0x80)
        {
            IIC_SDA_H;
        }
        else 
        {
            IIC_SDA_L;
        }
        
        IIC_SCL_H;;
		iic_delay();
        data_t<<=1;
        iic_delay();
		iic_delay();
        IIC_SCL_L;	 
		iic_delay();		
    }
    iic_delay();
} 	 


/*************************************************************************
*  �������ƣ�uint8 IIC_ReadByte(uint8 ack)
*  ����˵����ģ��IIC��ȡһ���ֽ�
*  ����˵����ack=1 ʱ���������ݻ�û������ ack=0 ʱ����������ȫ���������
*  �������أ����յ����ֽ�
*  �޸�ʱ�䣺2020��3��10��
*  Ӧ�þ�����IC_ReadByte(0x12);
*************************************************************************/
uint8 IIC_ReadByte(uint8 ack)
{
	uint8  i,receive=0;
	SDA_IN; //SDA����Ϊ����ģʽ �ȴ����մӻ���������
    for(i=0;i<8;i++ )
	{
        IIC_SCL_L; 
		iic_delay();
		iic_delay();
		iic_delay();
        IIC_SCL_H;
        receive<<=1;
        if(IIC_SDA_READ)receive++; //�ӻ����͵ĵ�ƽ
        iic_delay();
    }					 
    if(ack)
        IIC_Ack(); //����ACK 
    else
        IIC_NAck(); //����nACK  
    return receive;
}

/*************************************************************************
*  �������ƣ�void ADC_init(void)
*  ����˵����ģ��IIC��ȡָ���豸 ָ���Ĵ�����һ��ֵ
*  ����˵����
  * @param    I2C_Addr  Ŀ���豸��ַ
  * @param    reg       Ŀ��Ĵ���
  * @param    buf       ��Ŷ����ֽ�
*  �������أ�1ʧ�� 0�ɹ�
*  �޸�ʱ�䣺2020��3��10��
*  Ӧ�þ�����IIC_ReadByteFromSlave(0xD0, 0x75, &data);   //�� IIC��ַΪ 0xD0������MPU6050���Ĵ���0x75
*************************************************************************/
uint8 IIC_ReadByteFromSlave(uint8 I2C_Addr,uint8 reg,uint8 *buf)
{
	IIC_Start();	
	IIC_SendByte(I2C_Addr);	 //���ʹӻ���ַ
	if(IIC_WaitAck()) //����ӻ�δӦ�������ݷ���ʧ��
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg); //���ͼĴ�����ַ
	IIC_WaitAck();	  
	
	IIC_Start();
	IIC_SendByte(I2C_Addr+1); //�������ģʽ			   
	IIC_WaitAck();
	*buf=IIC_ReadByte(0);	   
    IIC_Stop(); //����һ��ֹͣ����
	return 0;
}
 

/*************************************************************************
*  �������ƣ�void ADC_init(void)
*  ����˵����ģ��IICдָ���豸 ָ���Ĵ�����һ��ֵ
*  ����˵����
  * @param    I2C_Addr  Ŀ���豸��ַ
  * @param    reg       Ŀ��Ĵ���
  * @param    data      д�������
*  �������أ�1ʧ�� 0�ɹ�
*  �޸�ʱ�䣺2020��3��10��
*  Ӧ�þ�����IIC_ReadByteFromSlave(0xD0, 0X6B, 0X80);   //IIC��ַΪ 0xD0������MPU6050���Ĵ���0x6B д��0x80
*************************************************************************/
uint8 IIC_WriteByteToSlave(uint8 I2C_Addr,uint8 reg,uint8 data_t)
{
	IIC_Start();
	IIC_SendByte(I2C_Addr); //���ʹӻ���ַ
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1; //�ӻ���ַд��ʧ��
	}
	IIC_SendByte(reg); //���ͼĴ�����ַ
    IIC_WaitAck();	  
	IIC_SendByte(data_t); 
	if(IIC_WaitAck())
	{
		IIC_Stop(); 
		return 1; //����д��ʧ��
	}
	IIC_Stop(); //����һ��ֹͣ����
    
    //return 1; //status == 0;
	return 0;
}

/*************************************************************************
*  �������ƣ�uint8 IIC_ReadMultByteFromSlave(uint8 dev, uint8 reg, uint8 length, uint8 *data_t)
*  ����˵����ģ��IIC��ȡָ���豸 ָ���Ĵ�����n��ֵ
*  ����˵����
  * @param    dev       Ŀ���豸��ַ
  * @param    reg       Ŀ��Ĵ���
  * @param    length    ��ȡ����
  * @param    data      ��Ŷ�������
*  �������أ�1ʧ�� 0�ɹ�
*  �޸�ʱ�䣺2020��3��10��
*  Ӧ�þ�����IIC_ReadByteFromSlave(0xD0, 0X3B, 14, &data);   //�� 14���ֽ�
*************************************************************************/
uint8 IIC_ReadMultByteFromSlave(uint8 dev, uint8 reg, uint8 length, uint8 *data_t)
{
    uint8  count = 0;
	uint8  temp;
	IIC_Start();
	IIC_SendByte(dev); //���ʹӻ���ַ
	if(IIC_WaitAck())
	{
		IIC_Stop(); 
		return 1; //�ӻ���ַд��ʧ��
	}
	IIC_SendByte(reg); //���ͼĴ�����ַ
    IIC_WaitAck();	  
	IIC_Start();
	IIC_SendByte(dev+1); //�������ģʽ	
	IIC_WaitAck();
    for(count=0;count<length;count++)
	{
		if(count!=(length-1))
            temp = IIC_ReadByte(1); //��ACK�Ķ�����
		else  
            temp = IIC_ReadByte(0); //���һ���ֽ�NACK
        
		data_t[count] = temp;
	}
    IIC_Stop(); //����һ��ֹͣ����
    //return count;
    return 0;
}

/*************************************************************************
*  �������ƣ�uint8 IIC_WriteMultByteToSlave(uint8 dev, uint8 reg, uint8 length, uint8* data_t)
*  ����˵����ģ��IICдָ���豸 ָ���Ĵ�����n��ֵ
*  ����˵����
  * @param    dev       Ŀ���豸��ַ
  * @param    reg       Ŀ��Ĵ���
  * @param    length    д�볤��
  * @param    data      ���д������
*  �������أ� 1ʧ�� 0�ɹ�
*  �޸�ʱ�䣺2020��3��10��
*  Ӧ�þ�����IIC_WriteMultByteToSlave(0xD0, 0X6B, 1, 0X80);   //��Ĵ���0x6Bд��0x80
*************************************************************************/
uint8 IIC_WriteMultByteToSlave(uint8 dev, uint8 reg, uint8 length, uint8* data_t)
{
    
 	uint8  count = 0;
	IIC_Start();
	IIC_SendByte(dev); //���ʹӻ���ַ
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1; //�ӻ���ַд��ʧ��
	}
	IIC_SendByte(reg); //���ͼĴ�����ַ
    IIC_WaitAck();	  
	for(count=0;count<length;count++)
	{
		IIC_SendByte(data_t[count]); 
		if(IIC_WaitAck()) //ÿһ���ֽڶ�Ҫ�ȴӻ�Ӧ��
		{
			IIC_Stop();
			return 1; //����д��ʧ��
		}
	}
	IIC_Stop(); //����һ��ֹͣ����
    
	return 0;
}



