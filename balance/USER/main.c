#include "stm32f10x.h"
#include "led.h"
#include "delay.h"

int main(void)
{
	LED_Init();
	Delay_Init();
	while(1)
	{
		LED_ON();
		Delay_ms(1800);
		LED_OFF();
		Delay_ms(1800);
	}
}
