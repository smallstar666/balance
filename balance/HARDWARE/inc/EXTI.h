#ifndef __EXTI_H
#define	__EXTI_H

#include "stm32f10x.h"


/** 
  * EXITx�궨�壺�ⲿ�жϣ���ֲʱ��Ҫ�޸��⼸����
  */ 
#define		EXTI_GPIO_CLK  			RCC_APB2Periph_GPIOA     
#define		EXTI_GPIO_PORT     		GPIOA   
#define    	EXTI_GPIO_PIN           GPIO_Pin_8
#define    	EXTI_SOURCE_PORT    	GPIO_PortSourceGPIOA
#define    	EXTI_SOURCE_PIN  		GPIO_PinSource8
#define    	EXTI_LINE           	EXTI_Line8
#define    	EXTI_IRQn             	EXTI9_5_IRQn
#define     EXTI_IRQHandler			EXTI9_5_IRQHandler


void EXTIx_Init(void);

#endif /* __EXTI_H */
