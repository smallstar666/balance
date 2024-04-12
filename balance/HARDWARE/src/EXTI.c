/**********************************************************************************
 * @note	Copyright 2019 SudekiMing. All Rights Reserved.
 
 * @file  	  	EXTI.c
 * @author    	SudekiMing  
 * @environment MDK-Standard Version: V5.15
 * @repository 	ST3.5.0
 * @date		2019-11-02
 
 * @function   
				1.EXTIx 外部中断配置初始化
				2.为Motion_Driver提供中断处理接口
				3.I2C的PA8-I2C_INT引脚配置
			
 * @hardware	 -------------------
 *				|                   |
 *          	|   INT  -  PA8  	|
 *				|                   |
 *          	 ------------------- 

 * @update
	@date	
	@details
	
**********************************************************************************/

#include "EXTI.h"
#include "mpu6050.h"

/**
 * @func	EXTIx_NVIC_Config
 * @brief	配置EXTIx嵌套向量中断控制器NVIC
 * @param 	无
 * @retval	无
 **/
static void EXTIx_NVIC_Config(void)
{
	
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 嵌套向量中断控制器组选择 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	
	/* 配置中断源 */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQn;	
	/* 抢占优先级为1 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	/* 子优先级为0 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	/* 使能中断通道 */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* 初始化配置NVIC */
	NVIC_Init(&NVIC_InitStructure);
	
}


/**
 * @func	EXTIx_GPIO_Config
 * @brief	EXTIx GPIO配置
 * @param 	无
 * @retval	无
 **/
static void EXTIx_GPIO_Config(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	
	/* 使能GPIOx时钟 */
	RCC_APB2PeriphClockCmd(EXTI_GPIO_CLK,ENABLE);
	
	/* 配置I2C的GPIO引脚：I2C_INT */
	GPIO_InitStructure.GPIO_Pin = EXTI_GPIO_PIN;
	/* 设置引脚模式为上拉输入 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	       
	/* 初始化GPIOx */
	GPIO_Init(EXTI_GPIO_PORT, &GPIO_InitStructure);
	
}


/**
 * @func	EXTIx_Mode_Config
 * @brief	配置EXTIx 工作模式
 * @param 	无
 * @retval	无
 **/
static void EXTIx_Mode_Config(void)
{
	
	EXTI_InitTypeDef EXTI_InitStructure;
	
	/* 配置EXTI_line信号源 */
	GPIO_EXTILineConfig(EXTI_SOURCE_PORT, EXTI_SOURCE_PIN); 
	/* 选择外部中断线 */
	EXTI_InitStructure.EXTI_Line = EXTI_LINE;
	/* EXTI为中断模式 */
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* 下降沿中断 */
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
	/* 使能外部中断 */
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	/* 初始化EXTI外设 */
	EXTI_Init(&EXTI_InitStructure); 
}


/*
 * 函数名：EXTIx_Init
 * 描述  ：EXTIx配置初始化
 * 输入  ：无
 * 输出  ：无
 */

void EXTIx_Init(void)
{
	/* 嵌套向量中断控制器NVIC配置 */
	EXTIx_NVIC_Config();	
	/* EXTIx GPIO配置 */
	EXTIx_GPIO_Config();	
	/* EXTIx 工作模式配置 */
	EXTIx_Mode_Config();	
	
}
