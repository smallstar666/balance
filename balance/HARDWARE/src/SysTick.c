/**********************************************************************************
 * @note	Copyright 2019 SudekiMing. All Rights Reserved.
		
 * @file  	  	SysTick.c
 * @author    	SudekiMing  
 * @environment MDK-Standard Version: V5.15
 * @repository  ST3.5.0
 * @date		2019-11-02
 
 * @function   
				1.SysTick 系统滴答定时器10us中断函数库，中断时间可自由配置
				2.常用的有 1us 10us 1ms 中断
						
 * @update
	@date	
	@details
	
**********************************************************************************/

#include "SysTick.h"
#include "delay.h"

static volatile uint32_t TimingDelay=0;	// 毫秒级延时计数累加器
volatile uint32_t g_ul_ms_ticks=0;		// 毫秒级时间戳计数累加器


/**
 * @func	SysTick_Init
 * @brief	系统滴答定时器初始化
			SystemFrequency / 1000    	1ms中断一次
			SystemFrequency / 100000	10us中断一次
			SystemFrequency / 1000000 	1us中断一次	
			注：ST3.5.0库版本SystemCoreClock/10不能超过16777216
	
 * @param 	无
 * @retval	无
 **/
void SysTick_Init(void)
{

	if(SysTick_Config(SystemCoreClock/1000))	
	{ 
		/* Capture error */ 
		while (1);
	}
	/* 关闭系统滴答定时器 */
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
	
	/* 开启系统滴答定时器 */
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;		
}


/**
 * @func	TimingDelay_Decrement
 * @brief	毫秒级延时计数累加器
 * @param 	无
 * @retval	无
 **/
static void TimingDelay_Decrement(void)
{
	if(TimingDelay != 0x00)
		TimingDelay--;
	
}


/**
 * @func	TimeStamp_Increment
 * @brief	毫秒级时间戳计数累加器
 * @param 	无
 * @retval	无
 **/
static void TimeStamp_Increment(void)
{
	g_ul_ms_ticks++;
}


/**
 * @func	mdelay
 * @brief	提供毫秒级延时服务
 * @param 	
	   @arg nTime : 延时 n ms
 * @retval	无
 **/
void mdelay(unsigned long nTime)
{
	TimingDelay = nTime;
	while(TimingDelay == 0);
}


/**
 * @func	get_tick_count
 * @brief	获取当前毫秒值，提供毫秒级时间戳接口
 * @param 	
	   @arg count ：存储最新毫秒值的变量
 * @retval	
 **/
int get_tick_count(unsigned long *count)
{
	count[0] = g_ul_ms_ticks;
	
	return 0;
}


/**
  * @brief  系统滴答定时器中断服务函数
  * @param  无
  * @retval 无
  */
void SysTick_Handler(void)
{
	TimingDelay_Decrement();
	TimeStamp_Increment();
	
}

