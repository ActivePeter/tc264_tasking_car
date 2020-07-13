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

#include <LQ_I2C_MPU6050.h>
#include <LQ_ICM20602.h>
#include <LQ_OLED096.h>
#include <LQ_SPI.h>
#include <LQ_STM.h>
#include <Platform_Types.h>
#include <stdio.h>

/*!
  * @brief    ����20602 SPI���߷�ʽ
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      Test_ICM20602();
  *
  * @date     2019/4/22 ����һ
*/
void Test_ICM20602(void)
{
    char  txt[30];
	short aacx,aacy,aacz;	         //���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;        //������ԭʼ����
    OLED_Init();                      //LCD��ʼ��
    OLED_CLS();                       //LCD����

    SPI_InitConfig(SPI3_CLK_P15_6, SPI3_MISO_P15_7, SPI3_MOSI_P00_0, SPI3_CS_P00_3, 10000000U);


    OLED_P8x16Str(15,0,"LQ 20602 Test");


    if(ICM20602_Init())
    {

    	OLED_P8x16Str(15,2,"ICM20602 Test Fail");
#pragma warning 557         // ���ξ���
		while (1);
#pragma warning default     // �򿪾���
    }

    while(1)
    {
        ICM_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);	//�õ����ٶȴ���������
        sprintf((char*)txt,"ax:%06d",aacx);
        OLED_P6x8Str(0,2,txt);
        sprintf((char*)txt,"ay:%06d",aacy);
        OLED_P6x8Str(0,3,txt);
        sprintf((char*)txt,"az:%06d",aacz);
        OLED_P6x8Str(0,4,txt);
        sprintf((char*)txt,"gx:%06d",gyrox);
        OLED_P6x8Str(0,5,txt);
        sprintf((char*)txt,"gy:%06d",gyroy);
        OLED_P6x8Str(0,6,txt);
        sprintf((char*)txt,"gz:%06d",gyroz);
        OLED_P6x8Str(0,7,txt);

        delayms(500);

    }

}


/*!
  * @brief    ����ȷ��ʱ
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      delayms_icm(100);
  *
  * @date     2019/4/22 ����һ
*/
void delayms_icm(uint16 ms)
{
	volatile uint32_t i = 0;
	while(ms--)
	{
		for (i = 0; i < 30000; ++i)
		{
			__asm("NOP"); /* delay */
		}
	}
}




/*!
  * @brief    ��ʼ��ICM20602 ���� ICM20602
  *
  * @param    ��
  *
  * @return   0����ʼ���ɹ�   1��ʧ��
  *
  * @note     ʹ��SPI���߷�ʽ�ĳ�ʼ��
  *
  * @see      ICM20602_Init();
  *
  * @date     2019/6/12 ������
  */
unsigned char ICM20602_Init(void)
{
    unsigned char res;
    res = ICM_Read_Byte(WHO_AM_I);           //��ȡICM20602��ID
    if(res == 0x12)                          //����ID��ȷ
    {
//        PRINTF("ICM20602 is OK!\n");
    }
    else
    {
//        PRINTF("\r\nThe correct IMU was not detected\r\nPlease check the wiring ID=%X\r\n",res);
        return 1;
    }

    res = 0;
    res += ICM_Write_Byte(ICM_PWR_MGMT1_REG,0X80);//��λICM20602
    delayms_icm(100);  //��ʱ100ms
    res += ICM_Write_Byte(ICM_PWR_MGMT1_REG,0X00);//����ICM20602
    res += ICM_Set_Gyro_Fsr(3);					  //�����Ǵ�����,��2000dps
    res += ICM_Set_Accel_Fsr(1);				  //���ٶȴ�����,��4g
    res += ICM_Set_Rate(1000);					  //���ò�����1000Hz
    res += ICM_Write_Byte(ICM_CFG_REG,0x02);      //�������ֵ�ͨ�˲���   98hz
    res += ICM_Write_Byte(ICM_INT_EN_REG,0X00);   //�ر������ж�
    res += ICM_Write_Byte(ICM_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
    res += ICM_Write_Byte(ICM_PWR_MGMT1_REG,0X01);//����CLKSEL,PLL X��Ϊ�ο�
    res += ICM_Write_Byte(ICM_PWR_MGMT2_REG,0X00);//���ٶ��������Ƕ�����

    if(res == 0)  //����Ĵ�����д��ɹ�
    {
//        PRINTF("MPU set is OK!\n");
    }
    else return 1;

    return 0;
}




/*!
  * @brief    ���������ǲ�����Χ
  *
  * @param    fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
  *
  * @return   0 �����óɹ�
  *
  * @note     ʹ��SPI���߷�ʽ�ĳ�ʼ��
  *
  * @see      ICM_Set_Gyro_Fsr(3);	//�����Ǵ�����,��2000dps
  *
  * @date     2019/6/12 ������
  */
unsigned char ICM_Set_Gyro_Fsr(unsigned char fsr)
{
	return ICM_Write_Byte(ICM_GYRO_CFG_REG,fsr<<3);
}



/*!
  * @brief    ���ü��ٶȼƲ�����Χ
  *
  * @param    fsr:0,��2g;  1,��4g;  2,��8g;   3,��16g
  *
  * @return   0�����óɹ�
  *
  * @note     ʹ��SPI���߷�ʽ�ĳ�ʼ��
  *
  * @see      ICM_Set_Accel_Fsr(1);				  //���ٶȴ�����,��4g
  *
  * @date     2019/6/12 ������
  */
unsigned char ICM_Set_Accel_Fsr(unsigned char fsr)
{
	return ICM_Write_Byte(ICM_ACCEL_CFG_REG,fsr<<3);
}



/*!
  * @brief    �������ֵ�ͨ�˲�
  *
  * @param    lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
  *
  * @return   0�����óɹ�
  *
  * @note     ʹ��SPI���߷�ʽ�ĳ�ʼ��
  *
  * @see      ICM_Set_LPF(100); //�������ֵ�ͨ�˲�
  *
  * @date     2019/6/12 ������
  */
unsigned char ICM_Set_LPF(uint16 lpf)
{
	unsigned char data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return ICM_Write_Byte(ICM_CFG_REG,data);//�������ֵ�ͨ�˲���
}




/*!
  * @brief    ���ò�����
  *
  * @param    rate:4~1000(Hz)
  *
  * @return   0�����óɹ�
  *
  * @note     ʹ��SPI���߷�ʽ�ĳ�ʼ��
  *
  * @see      ICM_Set_Rate(1000);		//���ò�����1000Hz
  *
  * @date     2019/6/12 ������
  */
unsigned char ICM_Set_Rate(uint16 rate)
{
	unsigned char data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	ICM_Write_Byte(ICM_SAMPLE_RATE_REG,data);	                //�������ֵ�ͨ�˲���
 	return ICM_Set_LPF(rate/2);	                                //�Զ�����LPFΪ�����ʵ�һ��
}



/*!
  * @brief    ��ȡ�¶�ֵ
  *
  * @param    ��
  *
  * @return   �¶�ֵ(������100��)
  *
  * @note     �¶�ֵ(������100��)
  *
  * @see      ICM_Get_Temperature();
  *
  * @date     2019/6/12 ������
  */
short ICM_Get_Temperature(void)
{
    unsigned char buf[3];
    short raw;
	float temp;
	ICM_Read_Len(ICM_TEMP_OUTH_REG,2,buf);
    raw=((uint16)buf[1]<<8)|buf[2];
    temp=21+((double)raw)/333.87;
    return (short)temp*100;
}





/*!
  * @brief    ��ȡ������ֵ
  *
  * @param    gx,gy,gz:������x,y,z���ԭʼ����(������)
  *
  * @return   0����ȡ�ɹ�
  *
  * @note     ��
  *
  * @see      int6_t data[3];
  * @see      ICM_Get_Gyroscope(&data[0], &data[1], &data[2]);
  *
  * @date     2019/6/12 ������
  */
unsigned char ICM_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    unsigned char buf[7],res;
	res=ICM_Read_Len(ICM_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16)buf[1]<<8)|buf[2];
		*gy=((uint16)buf[3]<<8)|buf[4];
		*gz=((uint16)buf[5]<<8)|buf[6];
	}
    return res;
}




/*!
  * @brief    ��ȡ���ٶ�ֵ
  *
  * @param    ax,ay,az:������x,y,z���ԭʼ����(������)
  *
  * @return   0����ȡ�ɹ�
  *
  * @note     ��
  *
  * @see      int6_t data[3];
  * @see      ICM_Get_Accelerometer(&data[0], &data[1], &data[2]);
  *
  * @date     2019/6/12 ������
  */
unsigned char ICM_Get_Accelerometer(short *ax,short *ay,short *az)
{
    unsigned char buf[7],res;
	res=ICM_Read_Len(ICM_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16)buf[1]<<8)|buf[2];
		*ay=((uint16)buf[3]<<8)|buf[4];
		*az=((uint16)buf[5]<<8)|buf[6];
	}
    return res;
}




/*!
  * @brief    ��ȡ ���ٶ�ֵ ���ٶ�ֵ
  *
  * @param    ax,ay,az:������x,y,z���ԭʼ����(������)
  * @param    gx,gy,gz:������x,y,z���ԭʼ����(������)
  *
  * @return   0����ȡ�ɹ�
  *
  * @note     ��
  *
  * @see      int6_t data[6];
  * @see      ICM_Get_Raw_data(&data[0], &data[1], &data[2],&data[3], &data[4], &data[5]);
  *
  * @date     2019/6/12 ������
  */
unsigned char ICM_Get_Raw_data(short *ax,short *ay,short *az,short *gx,short *gy,short *gz)
{
    unsigned char buf[15],res;
	res=ICM_Read_Len(ICM_ACCEL_XOUTH_REG,14,buf);
	if(res==0)
	{
		*ax=((uint16)buf[1]<<8)|buf[2];
		*ay=((uint16)buf[3]<<8)|buf[4];
		*az=((uint16)buf[5]<<8)|buf[6];
        *gx=((uint16)buf[9]<<8)|buf[10];
		*gy=((uint16)buf[11]<<8)|buf[12];
		*gz=((uint16)buf[13]<<8)|buf[14];
	}
    return res;
}




/*!
  * @brief    SPI ������
  *
  * @param    reg :Ҫ��ȡ�ļĴ�����ַ
  * @param    len :Ҫ��ȡ�ĳ���
  * @param    buf :��ȡ�������ݴ洢��
  *
  * @return   0 ����ȡ�ɹ�
  *
  * @note     �ײ����� ��ֲʱ��Ҫ�޸�
  *
  * @see      unsigned char buf[15],res;
  * @see      res=ICM_Read_Len(ICM_ACCEL_XOUTH_REG,14,buf);
  *
  * @date     2019/6/12 ������
  */
unsigned char ICM_Read_Len(unsigned char reg,unsigned char len,unsigned char *buf)
{
    buf[0] = reg | 0x80;

    /* д��Ҫ���ļĴ�����ַ */
    return SPI_ReadWriteNByte(SPI3, buf, buf, len + 1);
}



/*!
  * @brief    SPI дһ���Ĵ���
  *
  * @param    reg   :Ҫд�ļĴ�����ַ
  * @param    value :Ҫд���ֵ
  *
  * @return   0 ����ȡ�ɹ�
  *
  * @note     �ײ����� ��ֲʱ��Ҫ�޸�
  *
  * @see      ICM_Write_Byte(ICM_SAMPLE_RATE_REG,1);
  *
  * @date     2019/6/12 ������
  */
unsigned char ICM_Write_Byte(unsigned char reg,unsigned char value)
{
    unsigned char buff[2];

    buff[0] = reg;          //�ȷ��ͼĴ���
    buff[1] = value;        //�ٷ�������

    return SPI_ReadWriteNByte(SPI3, buff, buff, 2); //����buff�����ݣ����ɼ��� buff��
}

/*!
  * @brief    SPI ��һ���Ĵ���
  *
  * @param    reg   :Ҫд�ļĴ�����ַ
  *
  * @return   ��ȡ��ֵ
  *
  * @note     �ײ����� ��ֲʱ��Ҫ�޸�
  *
  * @see      ICM_Read_Byte(WHO_AM_I);
  *
  * @date     2019/6/12 ������
  */
unsigned char ICM_Read_Byte(unsigned char reg)
{
    unsigned char buff[2];
    buff[0] = reg | 0x80;          //�ȷ��ͼĴ���

    SPI_ReadWriteNByte(SPI3, buff, buff, 2);
    return buff[1];
}









