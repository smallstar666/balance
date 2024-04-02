#ifndef __ENCODER_H
#define __ENCODER_H

void Encoder_TIM2_Init(void);
void Encoder_TIM3_Init(void);
int Read_Speed(int TIMx);
void TIM2_IRQHandler(void);  //TIM2中断服务函数
void TIM3_IRQHandler(void);  //TIM3中断服务函数
#endif
