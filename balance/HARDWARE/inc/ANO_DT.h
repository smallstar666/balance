#ifndef __ANO_DT_H
#define	__ANO_DT_H

#include "stm32f10x.h"

/* С��ϵͳ������λֵ���ڸߵ�ַ����λֵ���ڵ͵�ַ
   &dwTempȡ��dwTemp�ĵ�ַתΪchar*���ͺ󣬼�һ�õ���λ��ַ
   ��ʱ��ַ������Ϊchar*�ͣ���*ȥ����ֵ�󣬼�Ϊ��λֵ */
/* ���ϵͳ����С��ϵͳ�෴����λֵ���ڵ�λ��ַ����λֵ���ڸ�λ��ַ
   ���Զ��ڴ��ϵͳ��������ʽȡ������dwTemp�ĵ�8λ */
/* ���ڴ�С��ϵͳ����������������ʽ��������õ�ѡ�񣬸�ͨ�õ�
   ��ȡ�߰�λֵ�ķ�ʽ�ǲ���λ���㣬����(dwTemp>>8)&0xff�������Ϊ�򵥣��Ҳ������*/
   
/* ���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16/float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з��� */ 
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))		// �Ͱ�λ
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	// �߰�λ
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


#define SDEV_ADDR	0x05	// �����豸����
#define DDEV_ADDR	0xAF	// Ŀ���豸����


void ANO_DT_SendStatus(float angle_rol, float angle_pit, float angle_yaw);
void ANO_DT_SendSenser(int16_t *Gyro,int16_t *Accel);	

#endif /* __ANO_DT_H */
