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
#ifndef __LQ_20602_H_
#define __LQ_20602_H_

#include "stdint.h"




//*!**************************************
// ����ICM20602�ڲ���ַ
//*!**************************************
//MPU6500���ڲ��Ĵ���
#define ICM_SELF_TESTX_REG		0X0D	//�Լ�Ĵ���X
#define ICM_SELF_TESTY_REG		0X0E	//�Լ�Ĵ���Y
#define ICM_SELF_TESTZ_REG		0X0F	//�Լ�Ĵ���Z
#define ICM_SELF_TESTA_REG		0X10	//�Լ�Ĵ���A
#define ICM_SAMPLE_RATE_REG		0X19	//����Ƶ�ʷ�Ƶ��
#define ICM_CFG_REG				0X1A	//���üĴ���
#define ICM_GYRO_CFG_REG		0X1B	//���������üĴ���
#define ICM_ACCEL_CFG_REG		0X1C	//���ٶȼ����üĴ���
#define ICM_MOTION_DET_REG		0X1F	//�˶���ֵⷧ���üĴ���
#define ICM_FIFO_EN_REG			0X23	//FIFOʹ�ܼĴ���

#define ICM_I2CMST_STA_REG		0X36	//IIC����״̬�Ĵ���
#define ICM_INTBP_CFG_REG		0X37	//�ж�/��·���üĴ���
#define ICM_INT_EN_REG			0X38	//�ж�ʹ�ܼĴ���
#define ICM_INT_STA_REG			0X3A	//�ж�״̬�Ĵ���

#define ICM_ACCEL_XOUTH_REG		0X3B	//���ٶ�ֵ,X���8λ�Ĵ���
#define ICM_ACCEL_XOUTL_REG		0X3C	//���ٶ�ֵ,X���8λ�Ĵ���
#define ICM_ACCEL_YOUTH_REG		0X3D	//���ٶ�ֵ,Y���8λ�Ĵ���
#define ICM_ACCEL_YOUTL_REG		0X3E	//���ٶ�ֵ,Y���8λ�Ĵ���
#define ICM_ACCEL_ZOUTH_REG		0X3F	//���ٶ�ֵ,Z���8λ�Ĵ���
#define ICM_ACCEL_ZOUTL_REG		0X40	//���ٶ�ֵ,Z���8λ�Ĵ���

#define ICM_TEMP_OUTH_REG		0X41	//�¶�ֵ�߰�λ�Ĵ���
#define ICM_TEMP_OUTL_REG		0X42	//�¶�ֵ��8λ�Ĵ���

#define ICM_GYRO_XOUTH_REG		0X43	//������ֵ,X���8λ�Ĵ���
#define ICM_GYRO_XOUTL_REG		0X44	//������ֵ,X���8λ�Ĵ���
#define ICM_GYRO_YOUTH_REG		0X45	//������ֵ,Y���8λ�Ĵ���
#define ICM_GYRO_YOUTL_REG		0X46	//������ֵ,Y���8λ�Ĵ���
#define ICM_GYRO_ZOUTH_REG		0X47	//������ֵ,Z���8λ�Ĵ���
#define ICM_GYRO_ZOUTL_REG		0X48	//������ֵ,Z���8λ�Ĵ���

#define ICM_I2CSLV0_DO_REG		0X63	//IIC�ӻ�0���ݼĴ���
#define ICM_I2CSLV1_DO_REG		0X64	//IIC�ӻ�1���ݼĴ���
#define ICM_I2CSLV2_DO_REG		0X65	//IIC�ӻ�2���ݼĴ���
#define ICM_I2CSLV3_DO_REG		0X66	//IIC�ӻ�3���ݼĴ���

#define ICM_I2CMST_DELAY_REG	0X67	//IIC������ʱ����Ĵ���
#define ICM_SIGPATH_RST_REG		0X68	//�ź�ͨ����λ�Ĵ���
#define ICM_MDETECT_CTRL_REG	0X69	//�˶������ƼĴ���
#define ICM_USER_CTRL_REG		0X6A	//�û����ƼĴ���
#define ICM_PWR_MGMT1_REG		0X6B	//��Դ����Ĵ���1
#define ICM_PWR_MGMT2_REG		0X6C	//��Դ����Ĵ���2
#define ICM_FIFO_CNTH_REG		0X72	//FIFO�����Ĵ����߰�λ
#define ICM_FIFO_CNTL_REG		0X73	//FIFO�����Ĵ����Ͱ�λ
#define ICM_FIFO_RW_REG			0X74	//FIFO��д�Ĵ���
#define WHO_AM_I		        0X75	//����ID�Ĵ���


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
void Test_ICM20602(void);


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
unsigned char ICM20602_Init(void);





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
unsigned char ICM_Set_Gyro_Fsr(unsigned char fsr);





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
unsigned char ICM_Set_Accel_Fsr(unsigned char fsr);





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
unsigned char ICM_Set_LPF(unsigned short lpf);



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
unsigned char ICM_Set_Rate(unsigned short rate);




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
short ICM_Get_Temperature(void);




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
unsigned char ICM_Get_Gyroscope(short *gx,short *gy,short *gz);



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
unsigned char ICM_Get_Accelerometer(short *ax,short *ay,short *az);




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
unsigned char ICM_Get_Raw_data(short *ax,short *ay,short *az,short *gx,short *gy,short *gz);



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
unsigned char ICM_Read_Len(unsigned char reg,unsigned char len,unsigned char *buf);




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
unsigned char ICM_Write_Byte(unsigned char reg,unsigned char value);



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
unsigned char ICM_Read_Byte(unsigned char reg);

#endif

