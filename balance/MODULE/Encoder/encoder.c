#include "stm32f10x.h"
#include "encoder.h"
#include "OLED.h"
#include "stdio.h"
//PA0(TIM2_CH1) PA1(TIM2_CH2) PB6(TIM4_CH1) PB7(TIM4_CH2)

#define ENCODER_RESOLUTION 11    /*编码器一圈的物理脉冲数*/
#define ENCODER_MULTIPLE 4       /*编码器倍频，通过定时器的编码器模式设置*/
#define MOTOR_REDUCTION_RATIO 30 /*电机的减速比 1：30*/
/*电机转一圈总的脉冲数(定时器能读到的脉冲数) = 编码器物理脉冲数*编码器倍频*电机减速比 */
#define TOTAL_RESOLUTION ( ENCODER_RESOLUTION*ENCODER_MULTIPLE*MOTOR_REDUCTION_RATIO ) 

void Encoder_TIM2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;                    
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef  TIM_ICInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能定时器2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能GPIOA的时钟
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; //配置PA0和PA1
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;    //50MHZ
	GPIO_Init(GPIOA, &GPIO_InitStruct);               //PA0和PA1引脚初始化
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 65535;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);////使用编码器模式3
	
	TIM_ICStructInit(&TIM_ICInitStruct);     //初始化输入捕获
	TIM_ICInitStruct.TIM_ICFilter = 10;      //设置滤波器长度 
	TIM_ICInit(TIM2, &TIM_ICInitStruct);
	
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE); //使能定时器2
}


void Encoder_TIM4_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;                    
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef  TIM_ICInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能GPIOA的时钟
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //配置PB6和PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;    //50MHZ
	GPIO_Init(GPIOB, &GPIO_InitStruct);               //PA0和PA1引脚初始化
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 65535;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);////使用编码器模式3
	
	TIM_ICStructInit(&TIM_ICInitStruct);     //初始化输入捕获
	TIM_ICInitStruct.TIM_ICFilter = 10; 
	TIM_ICInit(TIM4, &TIM_ICInitStruct);
		
	TIM_SetCounter(TIM4,0);
	TIM_Cmd(TIM4, ENABLE); //使能定时器4
}

int Read_Speed(int TIMx)   //这里传入的参数是2和4，输入2是得到TIM2的数值，输入4是得到TIM4的数值
{
	int value_1 = 0;
	switch(TIMx)
	{
		case 2:value_1=(short)TIM_GetCounter(TIM2);TIM_SetCounter(TIM2,0);break;//IF是定时器2，1.采集编码器的计数值并保存。2.将定时器的计数值清零。
		case 4:value_1=(short)TIM_GetCounter(TIM4);TIM_SetCounter(TIM4,0);break;
		default:value_1=0;
	}
	return value_1;
}

//计算电机转速（被另一个定时器100ms调用1次）
void calc_motor_rotate_speed()
{
	int encoderNum_L = 0;
	float rotateSpeed_L = 0;
	int encoderNum_R = 0;
	float rotateSpeed_R = 0;	
	
	/*读取编码器的值，正负代表旋转方向*/
	encoderNum_L = Read_Speed(2);
	/* 转速(1秒钟转多少圈)=单位时间内的计数值/总分辨率*时间系数 */
	rotateSpeed_L = (float)encoderNum_L/TOTAL_RESOLUTION*10;
	
	//printf("encoder_L: %d\t speed_L:%.2f rps\r\n",encoderNum_L,rotateSpeed_L);
	OLED_ShowNum(2, 1, encoderNum_L, 5);
	//OLED_ShowNum(2, 1, rotateSpeed_L, 5);
	
	/*读取编码器的值，正负代表旋转方向*/
	encoderNum_R = -Read_Speed(4);
	/* 转速(1秒钟转多少圈)=单位时间内的计数值/总分辨率*时间系数 */
	rotateSpeed_R = (float)encoderNum_L/TOTAL_RESOLUTION*10;
	
	//printf("encoder_R: %d\t speed_R:%.2f rps\r\n",encoderNum_R,rotateSpeed_R);
	OLED_ShowNum(2, 8, encoderNum_R, 5);
	//OLED_ShowNum(2, 8, rotateSpeed_R, 5);	
}
