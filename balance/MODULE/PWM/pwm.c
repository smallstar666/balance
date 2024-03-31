#include "stm32f10x.h"
#include "pwm.h"

void PWM_Init(uint16_t per,uint16_t psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;//���嶨ʱ���Ľṹ��
	GPIO_InitTypeDef GPIO_InitStruct;    //����GPIO�Ľṹ��
	TIM_OCInitTypeDef TIM_OCInitStruct;  //��������ȽϽṹ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);//��GPIOA��ʱ�ӡ�����ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//�򿪶�ʱ��3ʱ��
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;//�����������
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; //PB0��PB1
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_50MHz;//�ٶ�Ϊ50mhz
	GPIO_Init(GPIOB, &GPIO_InitStruct);//��PB0���Ž��г�ʼ��

	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;//1��Ƶ
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInitStruct.TIM_Period = per;  //�Զ�װ��ֵ
	TIM_TimeBaseInitStruct.TIM_Prescaler =	psc;//Ԥ��Ƶֵ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);//��TIM3���г�ʼ��
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;   //PWM1ģʽ
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCNPolarity_Low;//����Ƚϼ��Ե�
	TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Enable;//���״̬ʹ��
	TIM_OCInitStruct.TIM_Pulse = 0;//��ʼ��ռ�ձ�Ϊ0
	TIM_OC3Init(TIM3, &TIM_OCInitStruct);//����Ƚϳ�ʼ��
	TIM_OC4Init(TIM3, &TIM_OCInitStruct);//����Ƚϳ�ʼ��
	
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);//����Ƚ�3Ԥװ�ؼĴ���ʹ��
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);//����Ƚ�4Ԥװ�ؼĴ���ʹ��
	TIM_Cmd(TIM3,ENABLE);//TIM3ʹ��
}
