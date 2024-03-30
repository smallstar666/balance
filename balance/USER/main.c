#include "stm32f10x.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "stdio.h"

int main(void)
{
	LED_Init();        //小灯初始化
	Delay_Init();      //延时初始化
	Usart_Init(115200);//串口初始化
	printf("hello world!\r\n");
	while(1)
	{
		LED_ON();
		Delay_ms(1800);
		LED_OFF();
		Delay_ms(1800);
	}
}
