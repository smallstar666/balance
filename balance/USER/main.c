#include "stm32f10x.h"
#include "user_led.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "adc.h"
#include "OLED.h"
#include "pwm.h"
#include "mpu6050.h"
#include "SysTick.h"
#include "EXTI.h"
#include "bsp_i2c.h"
#include "motor.h"
#include "tim.h"
#include "testpoint.h"
#include "encoder.h"
#include "beep.h"
#include "ANO_DT.h"
#include "stdio.h"
#include "sys.h"

int Encoder_Left = 0;
int Encoder_Right = 0;

float Med_Angle=0;	//机械中值。---在这里修改你的机械中值即可。
float Target_Speed=0;	//期望速度。---二次开发接口，用于控制小车前进后退及其速度。

//PI控制 速度环
float Velocity_kp = 0;
float Velocity_ki= 0;

//PD控制 直立环
float Vertical_kp =250;//
float Vertical_kd = 1;

int Vertical_out,Velocity_out,Turn_out;//直立环&速度环&转向环的输出变量

int PWM_MAX=7200,PWM_MIN=-7200;	//PWM限幅
int MOTO1,MOTO2;	

int Vertical(float Med,float Angle,float gyro);
int Velocity(int Target,int encoder_left,int encoder_right);
int Turn(int gyro_Z);

void EXTI9_5_IRQHandler(void)
{
	int PWM_out=0;
	/* 判断是否产生了EXTI_Line中断 */
	if(EXTI_GetITStatus(EXTI_LINE) != RESET) 	
	{
		if(PAin(8)==0)  //再次确认是否为低电平
		{
			gyro_data_ready_cb();	// 更新陀螺仪数据
			UpdateAngle();
			Encoder_Left = Read_Speed(2);
			Encoder_Right = -Read_Speed(4);
			Velocity_out = Velocity(Target_Speed,Encoder_Left,Encoder_Right);//速度环
			Vertical_out = Vertical(Med_Angle+Velocity_out,Pitch,gyro[0]);
			Turn_out=Turn(gyro[2]);
			
			PWM_out = Vertical_out;
			MOTO1 = PWM_out-Turn_out;
			MOTO2 = PWM_out+Turn_out;
			Limit(&MOTO1,&MOTO2);
			Load(MOTO1,MOTO2);
			/* 向匿名上位机发送姿态数据 */
			//ANO_DT_SendStatus(0,Pitch,0);
			//GPIOA->ODR ^= GPIO_Pin_12;
		}
		EXTI_ClearITPendingBit(EXTI_LINE); 
	}  
}

int main(void)
{
	//LED_Init();        //小灯初始化
	//USER_LED_Init();
	Delay_Init();      //延时初始化
	Usart_Init(115200);//串口初始化
	//ADC_init();      //ADC初始化
	OLED_Init();       //OLED初始化
	//Beep_Init();
	//Beep_OFF();
	PWM_Init(7199,0);  //(7199+1)*(0+1)/72000000hz =  0.0001s  换成频率为10khz
	Motor_Init();      //电机初始化
	//PAout(4) = 1;
	//PAout(5) = 0;
	//PBout(14) = 0;
	//PBout(15) = 1;
	//TIM_SetCompare3(TIM3,500);//设置PWM占空比
	//TIM_SetCompare4(TIM3,500);//设置PWM占空比
	Encoder_TIM2_Init();
	Encoder_TIM4_Init();
	//TIM1_Init(7200-1,1000-1); //(7200-1+1)*(1000-1+1)/72000000hz =  0.1s = 100ms
	//100ms溢出一次，实现每100ms计算一次转速
	//printf("Balance Car\r\n");
	//OLED_ShowString(1, 1, "Balance Car");
	//OLED_ShowString(2, 1, "---------------");
	//OLED_ShowString(3, 3, "Designed By");
	//OLED_ShowString(4, 4, "Xiao Xing");
	//Delay_ms(500);
	//OLED_Clear();
	SysTick_Init();    //滴答器初始化
	EXTIx_Init();      //外部中断初始化
	Software_I2C_GPIO_Config();//模拟I2C初始化
	MPU6050_mpu_init();
	MPU6050_mpl_init();
	MPU6050_config();	
	//TestPoint_Init();
	while(1)
	{
			OLED_ShowString(1, 1, "Pitch:");
			OLED_ShowSignedNum(1, 7, Pitch, 5);
			OLED_ShowSignedNum(2, 1, Encoder_Left, 5);
			OLED_ShowSignedNum(2, 8, Encoder_Right, 5);		
			OLED_ShowString(3, 1, "Gyro0:");
			OLED_ShowSignedNum(3, 7, gyro[0], 5);
//		LED_ON();
//		USER_LED_ON();
//		Delay_ms(1800);
//		LED_OFF();
//		USER_LED_OFF();
//		Delay_ms(1800);
		//while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));  //等待转换完成
		//printf("Voltage Measurement is %f V\r\n",ADC_GetConversionValue(ADC1)*3.3/4096);
	}
}

//速度环PI控制
int Velocity(int Target,int encoder_left,int encoder_right)
{
	static int Encoder_S;       //staic
	int Encoder_Err = 0;
	int Err_Lowout = 0;
	static int Err_Lowout_last; //staic
	int PWM_out = 0;
	float a = 0.7;
	//1. 计算速度偏差
	Encoder_Err = ((encoder_left+encoder_right)-Target_Speed);
	//2. 对速度偏差进行低通滤波
	//low_out=(1-a)*Ek+a*low_out_last;
	Err_Lowout = (1-a)*Encoder_Err + a* Err_Lowout_last;
	Err_Lowout_last = Err_Lowout;
	//3. 对速度偏差积分，积分出位移
	Encoder_S += Err_Lowout;
	//4. 积分限幅
	if(Encoder_S>10000)       //最大值限幅
		Encoder_S = 10000;
	else if(Encoder_S<-10000) //最小值限幅
		Encoder_S = -10000;
	else 
		Encoder_S = Encoder_S;
	//5. 速度环控制输出计算
	PWM_out = Velocity_kp*Err_Lowout+Velocity_ki*Encoder_S;
	return PWM_out;
}

//角度环PD控制
/*********************
直立环PD控制器：Kp*Ek+Kd*Ek_D

入口：期望角度、真实角度、真实角速度
出口：直立环输出
*********************/
int Vertical(float Med,float Angle,float gyro)
{
	int PWM_out = 0;
	PWM_out= Vertical_kp*(Angle-Med)+Vertical_kd*(gyro-0);
	return PWM_out;
}

/*********************
转向环：系数*Z轴角速度
*********************/
int Turn(int gyro_Z)
{
	int PWM_out = 0;
	
	PWM_out = (-0.6)*gyro_Z;
	return PWM_out;
}
