/**********************************************************************************
 * @note	Copyright 2019 SudekiMing. All Rights Reserved.
 
 * @file  	  	EXTI.c
 * @author    	SudekiMing  
 * @environment MDK-Standard Version: V5.15
 * @repository 	ST3.5.0
 * @date		2019-11-02
 
 * @function   
				1.EXTIx �ⲿ�ж����ó�ʼ��
				2.ΪMotion_Driver�ṩ�жϴ���ӿ�
				3.I2C��PA8-I2C_INT��������
			
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
 * @brief	����EXTIxǶ�������жϿ�����NVIC
 * @param 	��
 * @retval	��
 **/
static void EXTIx_NVIC_Config(void)
{
	
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Ƕ�������жϿ�������ѡ�� */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
	
	/* �����ж�Դ */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQn;	
	/* ��ռ���ȼ�Ϊ1 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	/* �����ȼ�Ϊ0 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	/* ʹ���ж�ͨ�� */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* ��ʼ������NVIC */
	NVIC_Init(&NVIC_InitStructure);
	
}


/**
 * @func	EXTIx_GPIO_Config
 * @brief	EXTIx GPIO����
 * @param 	��
 * @retval	��
 **/
static void EXTIx_GPIO_Config(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	
	/* ʹ��GPIOxʱ�� */
	RCC_APB2PeriphClockCmd(EXTI_GPIO_CLK,ENABLE);
	
	/* ����I2C��GPIO���ţ�I2C_INT */
	GPIO_InitStructure.GPIO_Pin = EXTI_GPIO_PIN;
	/* ��������ģʽΪ�������� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	       
	/* ��ʼ��GPIOx */
	GPIO_Init(EXTI_GPIO_PORT, &GPIO_InitStructure);
	
}


/**
 * @func	EXTIx_Mode_Config
 * @brief	����EXTIx ����ģʽ
 * @param 	��
 * @retval	��
 **/
static void EXTIx_Mode_Config(void)
{
	
	EXTI_InitTypeDef EXTI_InitStructure;
	
	/* ����EXTI_line�ź�Դ */
	GPIO_EXTILineConfig(EXTI_SOURCE_PORT, EXTI_SOURCE_PIN); 
	/* ѡ���ⲿ�ж��� */
	EXTI_InitStructure.EXTI_Line = EXTI_LINE;
	/* EXTIΪ�ж�ģʽ */
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* �������ж� */
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
	/* ʹ���ⲿ�ж� */
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	/* ��ʼ��EXTI���� */
	EXTI_Init(&EXTI_InitStructure); 
}


/*
 * ��������EXTIx_Init
 * ����  ��EXTIx���ó�ʼ��
 * ����  ����
 * ���  ����
 */

void EXTIx_Init(void)
{
	/* Ƕ�������жϿ�����NVIC���� */
	EXTIx_NVIC_Config();	
	/* EXTIx GPIO���� */
	EXTIx_GPIO_Config();	
	/* EXTIx ����ģʽ���� */
	EXTIx_Mode_Config();	
	
}


/**
 * @func	EXTIx_ENABLE
 * @brief	ʹ���ⲿ�ж�
 * @param 	��
 * @retval	��
 **/
void EXTIx_ENABLE(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	/* ѡ���ⲿ�ж��� */
	EXTI_InitStructure.EXTI_Line = EXTI_LINE;
	/* EXTIΪ�ж�ģʽ */
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* �������ж� */
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	/* ʹ���ⲿ�ж� */
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	/* ��ʼ��EXTI���� */
	EXTI_Init(&EXTI_InitStructure);
}


/**
 * @func	EXTIx_DISABLE
 * @brief	ʧ���ⲿ�ж�
 * @param 	��
 * @retval	��
 **/
void EXTIx_DISABLE(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	/* ѡ���ⲿ�ж��� */
	EXTI_InitStructure.EXTI_Line = EXTI_LINE;
	/* EXTIΪ�ж�ģʽ */
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* �������ж� */
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	/* ʧ�����ⲿ�ж� */
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	/* ��ʼ��EXTI���� */
	EXTI_Init(&EXTI_InitStructure);
	/* ����ж����ϵ��жϱ�־λ*/
	EXTI_ClearITPendingBit(EXTI_LINE);
}


/*
 *��������EXTI_IRQHandler
 *����  ��EXTIx �жϴ�����
 *����  ����
 *���  ����
 */
void EXTI_IRQHandler(void)
{
	/* �ж��Ƿ������EXTI_Line�ж� */
	if(EXTI_GetITStatus(EXTI_LINE) != RESET) 	
	{
		gyro_data_ready_cb();	// ��������������
		
		/* ����жϱ�־λ */
		EXTI_ClearITPendingBit(EXTI_LINE); 
	}  
}
