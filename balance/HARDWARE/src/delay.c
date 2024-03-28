//�ο����ͣ�https://blog.csdn.net/qq_38800089/article/details/75195600
//�ο�����:  https://blog.csdn.net/qq_38405680/article/details/82055729

#include "stm32f10x.h"
#include "delay.h"

static uint8_t D_us = 0;    //΢��ϵ��
static uint16_t D_ms = 0;   //����ϵ��

void Delay_Init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//SysTick->CTRL &= (uint32_t)0xFFFFFFFB;
	D_us = SystemCoreClock/8000000; 
	//SystemCoreClock/8��72000000 Hz/8 = 9000000 Hz
	//SysTickһ������ʱ��Ϊ��1/(SystemCoreClock/8) s
	//1us��Ҫ��������Ϊ10^(-6)��[1/(SystemCoreClock/8)]=SystemCoreClock/8000000
	D_ms = (uint16_t)D_us * 1000; //1ms��Ҫ��������
}

void Delay_us(uint32_t nus)
{
	uint32_t temp = 0;
	SysTick->CTRL = 0x00;     //�ر�SysTick��ʱ��
	SysTick->LOAD = nus*D_us; //��ʱ��װ��ֵ
	SysTick->VAL  = 0x00;     //��ռ�����
	SysTick->CTRL |= 0x01;     //����SysTick��ʱ��
	do
	{
		temp = SysTick->CTRL;
	}while( (temp&0x01) &&!(temp&(1<<16))); //�ȴ���ʱ����  
	SysTick->CTRL = 0x00;     //�ر�SysTick��ʱ��
	SysTick->VAL  = 0x00;     //��ռ�����
}

void Delay_ms(uint32_t nms)
{
	uint32_t temp = 0;
	SysTick->CTRL = 0x00;     //�ر�SysTick��ʱ��
	SysTick->LOAD = nms*D_ms; //��ʱ��װ��ֵ
	SysTick->VAL  = 0x00;     //��ռ�����
	SysTick->CTRL |= 0x01;     //����SysTick��ʱ��
	do
	{
		temp = SysTick->CTRL;
	}while( (temp&0x01) &&!(temp&(1<<16)));
	SysTick->CTRL = 0x00;     //�ر�SysTick��ʱ��
	SysTick->VAL  = 0x00;     //��ռ�����
}
