/**********************************************************************************
 * @note	Copyright 2019 SudekiMing. All Rights Reserved.
		
 * @file  	  	SysTick.c
 * @author    	SudekiMing  
 * @environment MDK-Standard Version: V5.15
 * @repository  ST3.5.0
 * @date		2019-11-02
 
 * @function   
				1.SysTick ϵͳ�δ�ʱ��10us�жϺ����⣬�ж�ʱ�����������
				2.���õ��� 1us 10us 1ms �ж�
						
 * @update
	@date	
	@details
	
**********************************************************************************/

#include "SysTick.h"
#include "delay.h"

static volatile uint32_t TimingDelay=0;	// ���뼶��ʱ�����ۼ���
volatile uint32_t g_ul_ms_ticks=0;		// ���뼶ʱ��������ۼ���


/**
 * @func	SysTick_Init
 * @brief	ϵͳ�δ�ʱ����ʼ��
			SystemFrequency / 1000    	1ms�ж�һ��
			SystemFrequency / 100000	10us�ж�һ��
			SystemFrequency / 1000000 	1us�ж�һ��	
			ע��ST3.5.0��汾SystemCoreClock/10���ܳ���16777216
	
 * @param 	��
 * @retval	��
 **/
void SysTick_Init(void)
{

	if(SysTick_Config(SystemCoreClock/1000))	
	{ 
		/* Capture error */ 
		while (1);
	}
	/* �ر�ϵͳ�δ�ʱ�� */
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
	
	/* ����ϵͳ�δ�ʱ�� */
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;		
}


/**
 * @func	TimingDelay_Decrement
 * @brief	���뼶��ʱ�����ۼ���
 * @param 	��
 * @retval	��
 **/
static void TimingDelay_Decrement(void)
{
	if(TimingDelay != 0x00)
		TimingDelay--;
	
}


/**
 * @func	TimeStamp_Increment
 * @brief	���뼶ʱ��������ۼ���
 * @param 	��
 * @retval	��
 **/
static void TimeStamp_Increment(void)
{
	g_ul_ms_ticks++;
}


/**
 * @func	mdelay
 * @brief	�ṩ���뼶��ʱ����
 * @param 	
	   @arg nTime : ��ʱ n ms
 * @retval	��
 **/
void mdelay(unsigned long nTime)
{
	TimingDelay = nTime;
	while(TimingDelay == 0);
}


/**
 * @func	get_tick_count
 * @brief	��ȡ��ǰ����ֵ���ṩ���뼶ʱ����ӿ�
 * @param 	
	   @arg count ���洢���º���ֵ�ı���
 * @retval	
 **/
int get_tick_count(unsigned long *count)
{
	count[0] = g_ul_ms_ticks;
	
	return 0;
}


/**
  * @brief  ϵͳ�δ�ʱ���жϷ�����
  * @param  ��
  * @retval ��
  */
void SysTick_Handler(void)
{
	TimingDelay_Decrement();
	TimeStamp_Increment();
	
}

