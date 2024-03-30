#include "stm32f10x.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "stdio.h"

int main(void)
{
	LED_Init();        //С�Ƴ�ʼ��
	Delay_Init();      //��ʱ��ʼ��
	Usart_Init(115200);//���ڳ�ʼ��
	printf("hello world!\r\n");
	while(1)
	{
		LED_ON();
		Delay_ms(1800);
		LED_OFF();
		Delay_ms(1800);
	}
}
