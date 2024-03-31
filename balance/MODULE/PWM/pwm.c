#include "stm32f10x.h"
#include "pwm.h"

void PWM_Init(uint16_t per,uint16_t psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;//定义定时器的结构体
	GPIO_InitTypeDef GPIO_InitStruct;    //定义GPIO的结构体
	TIM_OCInitTypeDef TIM_OCInitStruct;  //定义输出比较结构体
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);//打开GPIOA的时钟、复用时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//打开定时器3时钟
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; //PB0和PB1
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_50MHz;//速度为50mhz
	GPIO_Init(GPIOB, &GPIO_InitStruct);//对PB0引脚进行初始化

	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;//1分频
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_Period = per;  //自动装载值
	TIM_TimeBaseInitStruct.TIM_Prescaler =	psc;//预分频值
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);//对TIM3进行初始化
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;   //PWM1模式
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCNPolarity_Low;//输出比较极性低
	TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Enable;//输出状态使能
	TIM_OCInitStruct.TIM_Pulse = 0;//初始化占空比为0
	TIM_OC3Init(TIM3, &TIM_OCInitStruct);//输出比较初始化
	TIM_OC4Init(TIM3, &TIM_OCInitStruct);//输出比较初始化
	
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);//输出比较3预装载寄存器使能
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);//输出比较4预装载寄存器使能
	TIM_Cmd(TIM3,ENABLE);//TIM3使能
}
