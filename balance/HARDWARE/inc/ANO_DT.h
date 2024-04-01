#ifndef __ANO_DT_H
#define	__ANO_DT_H

#include "stm32f10x.h"

/* 小端系统，即高位值存于高地址，低位值存于低地址
   &dwTemp取得dwTemp的地址转为char*类型后，加一得到高位地址
   此时地址类型仍为char*型，用*去操作值后，即为高位值 */
/* 大端系统，与小端系统相反，高位值存于低位地址，低位值存于高位地址
   所以对于大端系统，这个表达式取到的是dwTemp的低8位 */
/* 由于大小端系统的区别，所以这个表达式并不是最好的选择，更通用的
   是取高八位值的方式是采用位运算，即：(dwTemp>>8)&0xff，代码更为简单，且不会出错*/
   
/* 数据拆分宏定义，在发送大于1字节的数据类型时，比如int16/float等，需要把数据拆分成单独字节进行发送 */ 
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))		// 低八位
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	// 高八位
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


#define SDEV_ADDR	0x05	// 发送设备代码
#define DDEV_ADDR	0xAF	// 目标设备代码


void ANO_DT_SendStatus(float angle_rol, float angle_pit, float angle_yaw);
void ANO_DT_SendSenser(int16_t *Gyro,int16_t *Accel);	

#endif /* __ANO_DT_H */
