#include "stm32f10x.h"
#include "encoder.h"
//PA0(TIM2_CH1) PA1(TIM2_CH2) PA6(TIM3_CH1) PA7(TIM3_CH2)

void Encoder_TIM2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;                    
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef  TIM_ICInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//ʹ�ܶ�ʱ��2��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��GPIOA��ʱ��
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; //����PA0��PA1
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;    //50MHZ
	GPIO_Init(GPIOA, &GPIO_InitStruct);               //PA0��PA1���ų�ʼ��
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 65535;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);////ʹ�ñ�����ģʽ3
	
	TIM_ICStructInit(&TIM_ICInitStruct);     //��ʼ�����벶��
	TIM_ICInitStruct.TIM_ICFilter = 10;      //�����˲������� 
	TIM_ICInit(TIM2, &TIM_ICInitStruct);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);//���TIM2�ĸ��±�־λ
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);//ʹ�ܶ�ʱ���ж�
	
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE); //ʹ�ܶ�ʱ��2
}


void Encoder_TIM3_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;                    
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef  TIM_ICInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//ʹ�ܶ�ʱ��2��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��GPIOA��ʱ��
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //����PA6��PA7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;    //50MHZ
	GPIO_Init(GPIOA, &GPIO_InitStruct);               //PA0��PA1���ų�ʼ��
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 65535;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);////ʹ�ñ�����ģʽ3
	
	TIM_ICStructInit(&TIM_ICInitStruct);     //��ʼ�����벶��
	TIM_ICInitStruct.TIM_ICFilter = 10; 
	TIM_ICInit(TIM3, &TIM_ICInitStruct);
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);//���TIM3�ĸ��±�־λ
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//ʹ�ܶ�ʱ���ж�
	
	TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM3, ENABLE); //ʹ�ܶ�ʱ��3
}

int Read_Speed(int TIMx)   //���ﴫ��Ĳ�����2��3������2�ǵõ�TIM2����ֵ������3�ǵõ�TIM3����ֵ
{
	int value_1 = 0;
	switch(TIMx)
	{
		case 2:value_1=(short)TIM_GetCounter(TIM2);TIM_SetCounter(TIM2,0);break;//IF�Ƕ�ʱ��2��1.�ɼ��������ļ���ֵ�����档2.����ʱ���ļ���ֵ���㡣
		case 3:value_1=(short)TIM_GetCounter(TIM3);TIM_SetCounter(TIM3,0);break;
		default:value_1=0;
	}
	return value_1;
}

void TIM2_IRQHandler(void)  //TIM2�жϷ�����
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=0)
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}
	
void TIM3_IRQHandler(void)  //TIM3�жϷ�����
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=0)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	}
}

