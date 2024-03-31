#include "stm32f10x.h"
#include "adc.h"

void ADC_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;             //定义GPIO结构体变量
	ADC_InitTypeDef  ADC_InitStruct;              //定义ADC结构体变量
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1,ENABLE);//打开GPIOA和ADC1的时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);  //PCLK2 6分频后作为ADC时钟。ADC时钟为72mhz/6= 12mhz
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;    //模拟输入
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;        //PA0
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//50MHZ
	GPIO_Init(GPIOA, &GPIO_InitStruct);           //PA0引脚初始化
	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;    //ADC独立模式
	ADC_InitStruct.ADC_ScanConvMode  = DISABLE;        //不打开扫描转换模式
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;    //打开连续转换模式
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;//数据右对齐
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//不开启外部触发转换
	ADC_InitStruct.ADC_NbrOfChannel = 1;               //通道数为1
	ADC_Init(ADC1, &ADC_InitStruct);                   //ADC初始化
 
	ADC_Cmd(ADC1, ENABLE);//使能ADC1
	
	//配置ADC1通道0，规则采样顺序1，采样时间239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
	
	ADC_ResetCalibration(ADC1);//开始复位校准
	while(ADC_GetResetCalibrationStatus(ADC1));//等待复位校准完成
	ADC_StartCalibration(ADC1);//开始ADC校准
	while(ADC_GetCalibrationStatus(ADC1));//等待ADC校准完成
	
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);//使能软件触发转换
}
