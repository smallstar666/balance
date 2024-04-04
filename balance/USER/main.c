#include "stm32f10x.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "adc.h"
#include "OLED.h"
#include "pwm.h"
#include "mpu6050.h"
#include "SysTick.h"
#include "EXTI.h"
#include "bsp_i2c.h"
#include "motor.h"
#include "stdio.h"
#include "sys.h"

int main(void)
{
	SysTick_Init();    //�δ�����ʼ��
	EXTIx_Init();      //�ⲿ�жϳ�ʼ��
	LED_Init();        //С�Ƴ�ʼ��
	Delay_Init();      //��ʱ��ʼ��
	Usart_Init(115200);//���ڳ�ʼ��
	//ADC_init();      //ADC��ʼ��
	OLED_Init();       //OLED��ʼ��
	PWM_Init(7199,0);  //(7199+1)*(0+1)/72000000hz =  0.0001s  ����Ƶ��Ϊ10khz
	Motor_Init();      //�����ʼ��
	//-----------------------------------//
	PAout(4) = 1;
	PAout(5) = 0;
	PBout(14) = 0;
	PBout(15) = 1;
	//-----------------------------------//
	TIM_SetCompare3(TIM3,900);//����PWMռ�ձ�
	TIM_SetCompare4(TIM3,900);//����PWMռ�ձ�
	Software_I2C_GPIO_Config();//ģ��I2C��ʼ��
	printf("Balance Car\r\n");
	OLED_ShowString(1, 1, "Balance Car");
	OLED_ShowString(2, 1, "---------------");
	OLED_ShowString(3, 3, "Designed By");
	OLED_ShowString(4, 4, "Xiao Xing");
	Delay_ms(500);
	OLED_Clear();
	MPU6050_mpu_init();
	MPU6050_mpl_init();
	MPU6050_config();
	while(1)
	{
//		LED_ON();
//		Delay_ms(1800);
//		LED_OFF();
//		Delay_ms(1800);
//		while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));  //�ȴ�ת�����
//		printf("Voltage Measurement is %f V\r\n",ADC_GetConversionValue(ADC1)*3.3/4096);
		MPU6050_FUNC2();//mpu6050���ݲɼ�
	}
}
