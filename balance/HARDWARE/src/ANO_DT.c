/**********************************************************************************
 * @note	Copyright 2019 SudekiMing. All Rights Reserved.
		
 * @file  	  	ANO_DT.c
 * @author    	SudekiMing  
 * @environment MDK-Standard Version: V5.15
 * @repository  ST3.5.0
 * @date		2019-11-07
 
 * @function   
				1.匿名地面站通信协议V6.00
				2.匿名地面站V6.56版本
						
 * @update
	@date	
	@details
	
**********************************************************************************/

#include "ANO_DT.h"
#include "usart.h"

u8 data_to_send[50];	// 数据发送缓存


/**
 * @func	ANO_DT_SendData
 * @brief	数据发送函数
 * @param 	
	   @arg dataToSend : 要发送的数据
 * @retval	无
 **/
static void ANO_DT_SendData(u8 dataToSend)
{

	USART_SendChar(dataToSend);

}


/**
 * @func	ANO_DT_SendStatus
 * @brief	姿态信息输出
 * @param 	
	   @arg angle_rol : 横滚角
	   @arg angle_pit : 俯仰角
	   @arg angle_yaw : 航向角
 * @retval	无
 **/
void ANO_DT_SendStatus(float angle_rol, float angle_pit, float angle_yaw)
{
	u8 _cnt=0;
	u8 sum = 0;
	vs16 _temp;
	vs32 _temp2 = 0;
	
	data_to_send[_cnt++]=0xAA;				// 帧头0xAA
	data_to_send[_cnt++]=SDEV_ADDR;			// 发送设备地址
	data_to_send[_cnt++]=DDEV_ADDR;			// 目标设备地址
	data_to_send[_cnt++]=0x01;				// 帧ID（功能字：0x00~0xFA）
	data_to_send[_cnt++]=0;					// 报文数据长度
	
	_temp = (int)(angle_rol*100);			// 姿态信息输出
	data_to_send[_cnt++]=BYTE1(_temp);		// 高字节在前
	data_to_send[_cnt++]=BYTE0(_temp);		// 低字节在后
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0-(int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);		// 飞行高度,默认为0
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = 0;				// 飞行模式，默认为0
	
	data_to_send[_cnt++] = 0;				// 飞控是否解锁，默认为0
	
	
	data_to_send[4] = _cnt-5;				// 计算数据总字节数

	
	for(u8 i=0;i<_cnt;i++)					// 和校验
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	
	for(u8 i=0;i<_cnt;i++)					// 串口发送数据
		ANO_DT_SendData(data_to_send[i]);
}


/**
 * @func	ANO_DT_SendSenser
 * @brief	陀螺仪原始数据输出
 * @param 	
	   @arg Gyro  : 三轴陀螺仪原始数据
	   @arg Accel : 三轴加速度计原始数据
 * @retval	无
 **/
void ANO_DT_SendSenser(int16_t *Gyro,int16_t *Accel)
{
	u8 _cnt=0;
	u8 sum = 0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;				// 帧头0xAA
	data_to_send[_cnt++]=SDEV_ADDR;			// 发送设备地址
	data_to_send[_cnt++]=DDEV_ADDR;			// 目标设备地址
	data_to_send[_cnt++]=0x02;				// 帧ID（功能字：0x00~0xFA）
	data_to_send[_cnt++]=0;					// 报文数据长度
	
	_temp = Accel[0];						// 三轴加速度计原始数据
	data_to_send[_cnt++]=BYTE1(_temp);		// 高字节在前
	data_to_send[_cnt++]=BYTE0(_temp);		// 低字节在后
	_temp = Accel[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Accel[2];	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = Gyro[0];						// 三轴陀螺仪原始数据
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Gyro[1];	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Gyro[2];	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = 0;								// 三轴磁力计原始数据，默认为0
	data_to_send[_cnt++]=BYTE1(_temp);		
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[4] = _cnt-5;				// 计算数据总字节数
	
	
	for(u8 i=0;i<_cnt;i++)					// 和校验
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	for(u8 i=0;i<_cnt;i++)					// 串口发送数据
		ANO_DT_SendData(data_to_send[i]);
}
