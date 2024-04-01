/**********************************************************************************
 * @note	Copyright 2019 SudekiMing. All Rights Reserved.
		
 * @file  	  	ANO_DT.c
 * @author    	SudekiMing  
 * @environment MDK-Standard Version: V5.15
 * @repository  ST3.5.0
 * @date		2019-11-07
 
 * @function   
				1.��������վͨ��Э��V6.00
				2.��������վV6.56�汾
						
 * @update
	@date	
	@details
	
**********************************************************************************/

#include "ANO_DT.h"
#include "usart.h"

u8 data_to_send[50];	// ���ݷ��ͻ���


/**
 * @func	ANO_DT_SendData
 * @brief	���ݷ��ͺ���
 * @param 	
	   @arg dataToSend : Ҫ���͵�����
 * @retval	��
 **/
static void ANO_DT_SendData(u8 dataToSend)
{

	USART_SendChar(dataToSend);

}


/**
 * @func	ANO_DT_SendStatus
 * @brief	��̬��Ϣ���
 * @param 	
	   @arg angle_rol : �����
	   @arg angle_pit : ������
	   @arg angle_yaw : �����
 * @retval	��
 **/
void ANO_DT_SendStatus(float angle_rol, float angle_pit, float angle_yaw)
{
	u8 _cnt=0;
	u8 sum = 0;
	vs16 _temp;
	vs32 _temp2 = 0;
	
	data_to_send[_cnt++]=0xAA;				// ֡ͷ0xAA
	data_to_send[_cnt++]=SDEV_ADDR;			// �����豸��ַ
	data_to_send[_cnt++]=DDEV_ADDR;			// Ŀ���豸��ַ
	data_to_send[_cnt++]=0x01;				// ֡ID�������֣�0x00~0xFA��
	data_to_send[_cnt++]=0;					// �������ݳ���
	
	_temp = (int)(angle_rol*100);			// ��̬��Ϣ���
	data_to_send[_cnt++]=BYTE1(_temp);		// ���ֽ���ǰ
	data_to_send[_cnt++]=BYTE0(_temp);		// ���ֽ��ں�
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0-(int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);		// ���и߶�,Ĭ��Ϊ0
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = 0;				// ����ģʽ��Ĭ��Ϊ0
	
	data_to_send[_cnt++] = 0;				// �ɿ��Ƿ������Ĭ��Ϊ0
	
	
	data_to_send[4] = _cnt-5;				// �����������ֽ���

	
	for(u8 i=0;i<_cnt;i++)					// ��У��
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	
	for(u8 i=0;i<_cnt;i++)					// ���ڷ�������
		ANO_DT_SendData(data_to_send[i]);
}


/**
 * @func	ANO_DT_SendSenser
 * @brief	������ԭʼ�������
 * @param 	
	   @arg Gyro  : ����������ԭʼ����
	   @arg Accel : ������ٶȼ�ԭʼ����
 * @retval	��
 **/
void ANO_DT_SendSenser(int16_t *Gyro,int16_t *Accel)
{
	u8 _cnt=0;
	u8 sum = 0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;				// ֡ͷ0xAA
	data_to_send[_cnt++]=SDEV_ADDR;			// �����豸��ַ
	data_to_send[_cnt++]=DDEV_ADDR;			// Ŀ���豸��ַ
	data_to_send[_cnt++]=0x02;				// ֡ID�������֣�0x00~0xFA��
	data_to_send[_cnt++]=0;					// �������ݳ���
	
	_temp = Accel[0];						// ������ٶȼ�ԭʼ����
	data_to_send[_cnt++]=BYTE1(_temp);		// ���ֽ���ǰ
	data_to_send[_cnt++]=BYTE0(_temp);		// ���ֽ��ں�
	_temp = Accel[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Accel[2];	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = Gyro[0];						// ����������ԭʼ����
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Gyro[1];	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Gyro[2];	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = 0;								// ���������ԭʼ���ݣ�Ĭ��Ϊ0
	data_to_send[_cnt++]=BYTE1(_temp);		
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[4] = _cnt-5;				// �����������ֽ���
	
	
	for(u8 i=0;i<_cnt;i++)					// ��У��
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	for(u8 i=0;i<_cnt;i++)					// ���ڷ�������
		ANO_DT_SendData(data_to_send[i]);
}
