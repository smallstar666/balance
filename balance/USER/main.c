#include "stm32f10x.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "adc.h"
#include "OLED.h"
#include "pwm.h"
#include "stdio.h"

int main(void)
{
	LED_Init();        //小灯初始化
	Delay_Init();      //延时初始化
	Usart_Init(115200);//串口初始化
	ADC_init();        //ADC初始化
	OLED_Init();       //OLED初始化
	//PWM_Init(899,0);  //(899+1)*(0+1)/72000000hz =  0.0000125s  换成频率为80khz
	//TIM_SetCompare3(TIM3,450);//设置PWM占空比
	//TIM_SetCompare4(TIM3,450);//设置PWM占空比
	printf("Balance Car\r\n");
	OLED_ShowString(1, 1, "Balance Car");
	OLED_ShowString(2, 1, "---------------");
	OLED_ShowString(3, 3, "Designed By");
	OLED_ShowString(4, 4, "Xiao Xing");
	while(1)
	{
		LED_ON();
		Delay_ms(1800);
		LED_OFF();
		Delay_ms(1800);
		while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));  //等待转换完成
		printf("Voltage Measurement is %f V\r\n",ADC_GetConversionValue(ADC1)*3.3/4096);
	}
}
