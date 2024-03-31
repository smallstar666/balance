#include "stm32f10x.h"
#include "bsp_i2c.h"
#include "delay.h"

//采用模拟I2C
//PB6 SCL
//PB7 SDA
#define SCL_H         GPIOB->BSRR = GPIO_Pin_6
#define SCL_L         GPIOB->BRR  = GPIO_Pin_6 
   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_7
#define SDA_L         GPIOB->BRR  = GPIO_Pin_7

#define SCL_read      GPIOB->IDR  & GPIO_Pin_6
#define SDA_read      GPIOB->IDR  & GPIO_Pin_7

void I2C_delay(void)
{
		
   u8 i=5; //这里可以优化速度，经测试最低到5还能写入
   while(i) 
   { 
     i--; 
   }  
}

void Software_I2C_GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; //开漏输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; //开漏输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void I2C_Start(void)  //I2C开始波形
{
	SDA_H;
	SCL_H;
	I2C_delay();
	I2C_delay();
	
	SDA_L;
	I2C_delay();
	
	SCL_L;
	I2C_delay();
}

void I2C_Stop(void)  //I2C结束波形
{
	SDA_L;
	SCL_L;
	I2C_delay();
	
	SCL_H;
	I2C_delay();
	
	SDA_H;
	I2C_delay();
} 

void I2C_Ack(void)   //主机(单片机)发送ACK波形
{	
	SCL_L;
	SDA_L;
	I2C_delay();
	
	SDA_L;
	I2C_delay();
	
	SCL_H;
	I2C_delay();
	I2C_delay();
	
	SCL_L;
	I2C_delay();
	
	SCL_L;
	SDA_L;
	I2C_delay();
}   

//void I2C_NoAck(void) //主机(单片机)发送NACK波形
//{	
//	SCL_L;
//	SDA_L;
//	I2C_delay();
//	
//	SDA_H;
//	I2C_delay();
//	
//	SCL_H;
//	I2C_delay();
//	I2C_delay();
//	
//	SCL_L;
//	I2C_delay();
//	
//	SCL_L;
//	SDA_L;
//	I2C_delay();
//} 


void I2C_WaitAck(void)
{
	SCL_L;
	SDA_L;
	I2C_delay();
	
	SDA_H;
	I2C_delay();
	
	SCL_H;
	I2C_delay();
	if((SDA_read) == 1)
	{
		I2C_delay();
		SCL_L;
		I2C_delay();
		SCL_L;
		SDA_L;
		I2C_delay();
	}
	else
	{
		I2C_delay();
		SCL_L;
		I2C_delay();
		SCL_L;
		SDA_L;
		I2C_delay();
	}
}


void I2C_SendByte(uint8_t SendByte)
{
    uint8_t i=8;
    while(i--)
    {
		SCL_L;
		SDA_L;
		I2C_delay();
		
		if(SendByte&0x80)
			SDA_H;
		else
			SDA_L;
		I2C_delay();
		
		SCL_H;
		I2C_delay();
		I2C_delay();
		
		SCL_L;
		I2C_delay();
		
		SCL_L;
		SDA_L;
		I2C_delay();
		SendByte <<= 1;
    }
} 

unsigned char I2C_ReadByte(void)
{ 
    uint8_t i=8;
    uint8_t ReceiveByte=0;
			
    while(i--)
    {
		ReceiveByte<<=1;
		SCL_L;
		SDA_L;
		I2C_delay();
		
		SDA_H;
		I2C_delay();
		
		SCL_H;
		I2C_delay();
		if(SDA_read)
		{
			ReceiveByte|=0x01;
		}
		I2C_delay();
		
		SCL_L;
		I2C_delay();
		
		SCL_L;
		SDA_L;
		I2C_delay();
    }
    return ReceiveByte;
}

int Sensors_I2C_WriteRegister(uint8_t slave_address,uint8_t register_address,uint8_t length,const uint8_t *data)
{
	int i = 0;
	uint8_t ret = 0;
    I2C_Start();                     //起始信号
    I2C_SendByte(slave_address<<1);     //发送设备地址+写信号
	I2C_WaitAck();
    I2C_SendByte(register_address);  //内部寄存器地址
	I2C_WaitAck();
	for (i=0;i<length;i++) 
	{
		I2C_SendByte(data[i]);             //内部寄存器数据
		I2C_WaitAck();
	}
    I2C_Stop();                      //发送停止信号
	return  ret;
}


int Sensors_I2C_ReadRegister(uint8_t slave_address,uint8_t register_address,uint8_t length, uint8_t *data)
{
	int i = 0;
	uint8_t ret = 0;
	I2C_Start();                      //起始信号
	I2C_SendByte(slave_address<<1);      //发送设备地址+写信号
	I2C_WaitAck();
	I2C_SendByte(register_address);   //发送存储单元地址，从0开始
	I2C_WaitAck();
	I2C_Stop();  				      //停止信号
	I2C_Start();                      //起始信号
	I2C_SendByte((slave_address<<1)+1);    //发送设备地址+读信号
	I2C_WaitAck();
	for(i = 0;i<length;i++)
	{
		*(data+i)=I2C_ReadByte();             //读出寄存器数据
		if(i!=length-1)
			I2C_Ack();                	  //主机发送ACK信号
	}
	I2C_WaitAck();
	I2C_Stop();                       //停止信号
	return  ret;	
}
