#include "stm32f10x.h"
#include "led.h"

//´ÖÂÔÑÓÊ±
void delay(uint32_t timers)
{
	uint16_t i=0,j=0;
	for(i = 0;i<timers;i++)
	{
		for(j =0;j<65535;j++);
	}
}

int main(void)
{
	LED_Init();
	while(1)
	{
		LED_ON();
		delay(100);
		LED_OFF();
		delay(100);
	}
}
