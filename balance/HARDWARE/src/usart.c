#include "stm32f10x.h"
#include "usart.h"	
#include "stdio.h"

int fputc(int ch,FILE *f)
{
	USART_SendData(USART1,(uint8_t)ch);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);//�ȴ��������
	return ch;
}

void Usart_Init(uint32_t bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA,ENABLE);//��USART1��GPIOA��ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;        //PA9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHZ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_Init(GPIOA,&GPIO_InitStructure);  //��ʼ��PA9����
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;        //PA10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHZ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //��������
	GPIO_Init(GPIOA,&GPIO_InitStructure);  //��ʼ��PA10����
	
	USART_InitStructure.USART_BaudRate = bound;  //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //ֹͣλΪ1λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Parity = USART_Parity_No; //����żУ��λ
	USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;//���ͽ���ģʽ
	USART_Init(USART1,&USART_InitStructure); //��ʼ��USART1
	
	USART_Cmd(USART1,ENABLE);  //ʹ��USART1
}

