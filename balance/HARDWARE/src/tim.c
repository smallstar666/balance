#include "stm32f10x.h"
#include "tim.h"
#include "encoder.h"

void TIM1_Init(uint16_t per,uint16_t psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = per;
	TIM_TimeBaseInitStruct.TIM_Prescaler = psc;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);
	
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);               //��������ж�
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM1_UP_IRQn;        //TIM1�ж�
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�1��
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;        //�����ȼ�3��
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_Cmd(TIM1,ENABLE);     //ʹ�ܶ�ʱ��
} 


void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) //���TIM1�����жϷ������
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);    //���TIM1�����жϱ�־
		calc_motor_rotate_speed();
	}
}
