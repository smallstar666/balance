#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"
#include <stdio.h>

void Usart_Init(uint32_t bound);
int fputc(int ch,FILE *f);
int fputcc(int ch);
void USART_SendChar(uint8_t chr);

#endif
