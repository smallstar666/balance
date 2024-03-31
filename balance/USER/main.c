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
	LED_Init();        //С�Ƴ�ʼ��
	Delay_Init();      //��ʱ��ʼ��
	Usart_Init(115200);//���ڳ�ʼ��
	ADC_init();        //ADC��ʼ��
	OLED_Init();       //OLED��ʼ��
	//PWM_Init(899,0);  //(899+1)*(0+1)/72000000hz =  0.0000125s  ����Ƶ��Ϊ80khz
	//TIM_SetCompare3(TIM3,450);//����PWMռ�ձ�
	//TIM_SetCompare4(TIM3,450);//����PWMռ�ձ�
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
		while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));  //�ȴ�ת�����
		printf("Voltage Measurement is %f V\r\n",ADC_GetConversionValue(ADC1)*3.3/4096);
	}
}
