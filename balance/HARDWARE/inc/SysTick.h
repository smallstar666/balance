#ifndef __SYSTICK_H
#define	__SYSTICK_H

#include "stm32f10x.h"

void SysTick_Init(void);
void mdelay(unsigned long nTime);
int get_tick_count(unsigned long *count);

#endif /* __SYSTICK_H */
