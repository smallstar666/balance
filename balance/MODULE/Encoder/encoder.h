#ifndef __ENCODER_H
#define __ENCODER_H

void Encoder_TIM2_Init(void);
void Encoder_TIM4_Init(void);
int Read_Speed(int TIMx);
void calc_motor_rotate_speed(void);
#endif
