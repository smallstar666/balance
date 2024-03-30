#include "stm32f10x.h"
#include "usart.h"	
#include "stdio.h"

int fputc(int ch,FILE *f)
{
	USART_SendData(USART1,(uint8_t)ch);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);//等待发送完成
	return ch;
}

void Usart_Init(uint32_t bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA,ENABLE);//打开USART1和GPIOA的时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;        //PA9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHZ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_Init(GPIOA,&GPIO_InitStructure);  //初始化PA9引脚
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;        //PA10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHZ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
	GPIO_Init(GPIOA,&GPIO_InitStructure);  //初始化PA10引脚
	
	USART_InitStructure.USART_BaudRate = bound;  //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //停止位为1位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Parity = USART_Parity_No; //无奇偶校验位
	USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;//发送接收模式
	USART_Init(USART1,&USART_InitStructure); //初始化USART1
	
	USART_Cmd(USART1,ENABLE);  //使能USART1
}

