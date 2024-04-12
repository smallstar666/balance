#include "stm32f10x.h"
#include "encoder.h"
#include "OLED.h"
#include "stdio.h"
//PA0(TIM2_CH1) PA1(TIM2_CH2) PB6(TIM4_CH1) PB7(TIM4_CH2)

#define ENCODER_RESOLUTION 11    /*������һȦ������������*/
#define ENCODER_MULTIPLE 4       /*��������Ƶ��ͨ����ʱ���ı�����ģʽ����*/
#define MOTOR_REDUCTION_RATIO 30 /*����ļ��ٱ� 1��30*/
/*���תһȦ�ܵ�������(��ʱ���ܶ�����������) = ����������������*��������Ƶ*������ٱ� */
#define TOTAL_RESOLUTION ( ENCODER_RESOLUTION*ENCODER_MULTIPLE*MOTOR_REDUCTION_RATIO ) 

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
	
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE); //ʹ�ܶ�ʱ��2
}


void Encoder_TIM4_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;                    
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef  TIM_ICInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//ʹ�ܶ�ʱ��2��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��GPIOA��ʱ��
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //����PB6��PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;    //50MHZ
	GPIO_Init(GPIOB, &GPIO_InitStruct);               //PA0��PA1���ų�ʼ��
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 65535;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);////ʹ�ñ�����ģʽ3
	
	TIM_ICStructInit(&TIM_ICInitStruct);     //��ʼ�����벶��
	TIM_ICInitStruct.TIM_ICFilter = 10; 
	TIM_ICInit(TIM4, &TIM_ICInitStruct);
		
	TIM_SetCounter(TIM4,0);
	TIM_Cmd(TIM4, ENABLE); //ʹ�ܶ�ʱ��4
}

int Read_Speed(int TIMx)   //���ﴫ��Ĳ�����2��4������2�ǵõ�TIM2����ֵ������4�ǵõ�TIM4����ֵ
{
	int value_1 = 0;
	switch(TIMx)
	{
		case 2:value_1=(short)TIM_GetCounter(TIM2);TIM_SetCounter(TIM2,0);break;//IF�Ƕ�ʱ��2��1.�ɼ��������ļ���ֵ�����档2.����ʱ���ļ���ֵ���㡣
		case 4:value_1=(short)TIM_GetCounter(TIM4);TIM_SetCounter(TIM4,0);break;
		default:value_1=0;
	}
	return value_1;
}

//������ת�٣�����һ����ʱ��100ms����1�Σ�
void calc_motor_rotate_speed()
{
	int encoderNum_L = 0;
	float rotateSpeed_L = 0;
	int encoderNum_R = 0;
	float rotateSpeed_R = 0;	
	
	/*��ȡ��������ֵ������������ת����*/
	encoderNum_L = Read_Speed(2);
	/* ת��(1����ת����Ȧ)=��λʱ���ڵļ���ֵ/�ֱܷ���*ʱ��ϵ�� */
	rotateSpeed_L = (float)encoderNum_L/TOTAL_RESOLUTION*10;
	
	//printf("encoder_L: %d\t speed_L:%.2f rps\r\n",encoderNum_L,rotateSpeed_L);
	OLED_ShowNum(2, 1, encoderNum_L, 5);
	//OLED_ShowNum(2, 1, rotateSpeed_L, 5);
	
	/*��ȡ��������ֵ������������ת����*/
	encoderNum_R = -Read_Speed(4);
	/* ת��(1����ת����Ȧ)=��λʱ���ڵļ���ֵ/�ֱܷ���*ʱ��ϵ�� */
	rotateSpeed_R = (float)encoderNum_L/TOTAL_RESOLUTION*10;
	
	//printf("encoder_R: %d\t speed_R:%.2f rps\r\n",encoderNum_R,rotateSpeed_R);
	OLED_ShowNum(2, 8, encoderNum_R, 5);
	//OLED_ShowNum(2, 8, rotateSpeed_R, 5);	
}
