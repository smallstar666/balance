#include "stm32f10x.h"
#include "adc.h"

void ADC_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;             //����GPIO�ṹ�����
	ADC_InitTypeDef  ADC_InitStruct;              //����ADC�ṹ�����
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1,ENABLE);//��GPIOA��ADC1��ʱ��
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);  //PCLK2 6��Ƶ����ΪADCʱ�ӡ�ADCʱ��Ϊ72mhz/6= 12mhz
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;    //ģ������
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;        //PA0
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//50MHZ
	GPIO_Init(GPIOA, &GPIO_InitStruct);           //PA0���ų�ʼ��
	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;    //ADC����ģʽ
	ADC_InitStruct.ADC_ScanConvMode  = DISABLE;        //����ɨ��ת��ģʽ
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;    //������ת��ģʽ
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;//�����Ҷ���
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//�������ⲿ����ת��
	ADC_InitStruct.ADC_NbrOfChannel = 1;               //ͨ����Ϊ1
	ADC_Init(ADC1, &ADC_InitStruct);                   //ADC��ʼ��
 
	ADC_Cmd(ADC1, ENABLE);//ʹ��ADC1
	
	//����ADC1ͨ��0���������˳��1������ʱ��239.5����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
	
	ADC_ResetCalibration(ADC1);//��ʼ��λУ׼
	while(ADC_GetResetCalibrationStatus(ADC1));//�ȴ���λУ׼���
	ADC_StartCalibration(ADC1);//��ʼADCУ׼
	while(ADC_GetCalibrationStatus(ADC1));//�ȴ�ADCУ׼���
	
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);//ʹ���������ת��
}
