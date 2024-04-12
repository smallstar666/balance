#include "stm32f10x.h"
#include "motor.h"

void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}


/* 限幅函数 */
void Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX) *motoA = PWM_MAX;
	if(*motoA<PWM_MIN) *motoA = PWM_MIN;

	if(*motoB>PWM_MAX) *motoB = PWM_MAX;
	if(*motoB<PWM_MIN) *motoB = PWM_MIN;	
}

/* 限幅函数 */
int GFP_abs(int p)
{
	int q = 0;
	if(p>0)
		q= p;
	else
		q = -p;
	return q;
}

/* 限幅函数 */
void Load(int moto1,int moto2)
{
	if(moto1>0) Ain1 =1,Ain2 = 0;
	else Ain1 =0,Ain2 = 1;
	TIM_SetCompare3(TIM3,GFP_abs(moto1));
	//TIM_SetCompare3(TIM3,500);

	if(moto2>0) Bin1 =0,Bin2 = 1;
	else Bin1 =1,Bin2 = 0;
	TIM_SetCompare4(TIM3,GFP_abs(moto2));	
	//TIM_SetCompare4(TIM3,500);
}
