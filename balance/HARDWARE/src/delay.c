//参考博客：https://blog.csdn.net/qq_38800089/article/details/75195600
//参考博客:  https://blog.csdn.net/qq_38405680/article/details/82055729

#include "stm32f10x.h"
#include "delay.h"

static uint8_t D_us = 0;    //微妙系数
static uint16_t D_ms = 0;   //毫秒系数

void Delay_Init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//SysTick->CTRL &= (uint32_t)0xFFFFFFFB;
	D_us = SystemCoreClock/8000000; 
	//SystemCoreClock/8：72000000 Hz/8 = 9000000 Hz
	//SysTick一个计数时长为：1/(SystemCoreClock/8) s
	//1us需要计数次数为10^(-6)÷[1/(SystemCoreClock/8)]=SystemCoreClock/8000000
	D_ms = (uint16_t)D_us * 1000; //1ms需要计数次数
}

void Delay_us(uint32_t nus)
{
	uint32_t temp = 0;
	SysTick->CTRL = 0x00;     //关闭SysTick定时器
	SysTick->LOAD = nus*D_us; //延时重装载值
	SysTick->VAL  = 0x00;     //清空计数器
	SysTick->CTRL |= 0x01;     //开启SysTick定时器
	do
	{
		temp = SysTick->CTRL;
	}while( (temp&0x01) &&!(temp&(1<<16))); //等待延时结束  
	SysTick->CTRL = 0x00;     //关闭SysTick定时器
	SysTick->VAL  = 0x00;     //清空计数器
}

void Delay_ms(uint32_t nms)
{
	uint32_t temp = 0;
	SysTick->CTRL = 0x00;     //关闭SysTick定时器
	SysTick->LOAD = nms*D_ms; //延时重装载值
	SysTick->VAL  = 0x00;     //清空计数器
	SysTick->CTRL |= 0x01;     //开启SysTick定时器
	do
	{
		temp = SysTick->CTRL;
	}while( (temp&0x01) &&!(temp&(1<<16)));
	SysTick->CTRL = 0x00;     //关闭SysTick定时器
	SysTick->VAL  = 0x00;     //清空计数器
}
