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
	SysTick_Init();    //滴答器初始化
	EXTIx_Init();      //外部中断初始化
	LED_Init();        //小灯初始化
	Delay_Init();      //延时初始化
	Usart_Init(115200);//串口初始化
	//ADC_init();      //ADC初始化
	OLED_Init();       //OLED初始化
	PWM_Init(7199,0);  //(7199+1)*(0+1)/72000000hz =  0.0001s  换成频率为10khz
	Motor_Init();      //电机初始化
	//-----------------------------------//
	PAout(4) = 1;
	PAout(5) = 0;
	PBout(14) = 0;
	PBout(15) = 1;
	//-----------------------------------//
	TIM_SetCompare3(TIM3,900);//设置PWM占空比
	TIM_SetCompare4(TIM3,900);//设置PWM占空比
	Software_I2C_GPIO_Config();//模拟I2C初始化
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
//		while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));  //等待转换完成
//		printf("Voltage Measurement is %f V\r\n",ADC_GetConversionValue(ADC1)*3.3/4096);
		MPU6050_FUNC2();//mpu6050数据采集
	}
}
