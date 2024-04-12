#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h" 
void Motor_Init(void);

#define Ain1  PAout(4)
#define Ain2  PAout(5)

#define Bin1  PBout(14)
#define Bin2  PBout(15)

void Limit(int *motoA,int *motoB);
int GFP_abs(int p);
void Load(int moto1,int moto2);

extern int PWM_MAX,PWM_MIN;
#endif
