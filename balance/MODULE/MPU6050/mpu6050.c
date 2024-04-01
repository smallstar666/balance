#include "stm32f10x.h"
#include "bsp_i2c.h"
#include "mpu6050.h"

#include "usart.h"
#include "ANO_DT.h"
#include "oled.h"
#include "SysTick.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"
#include "data_builder.h"


//**************************************
//MPU6050写寄存器
//**************************************
void MPU6050_WriteReg(uint8_t reg_address,uint8_t reg_data)
{
	Sensors_I2C_WriteRegister(SlaveAddress,reg_address,1,&reg_data);
}

//**************************************
//MPU6050读寄存器
//**************************************
void MPU6050_ReadReg(uint8_t reg_address,uint8_t *reg_data,uint8_t length)
{
	Sensors_I2C_ReadRegister(SlaveAddress,reg_address,length,reg_data);
}

//**************************************
//初始化MPU6050
//**************************************
void InitMPU6050(void)
{
	//----------------------------------------------------------------//
    int i=0,j=0;
    //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
    for (i=0; i<1000; i++) {
        for (j=0; j<1000; j++) {
            ;
        }
    }
	//----------------------------------------------------------------//
	MPU6050_WriteReg(PWR_MGMT_1, 0x00);	  //0x6B   解除睡眠并且选择内部8M时钟
	MPU6050_WriteReg(PWR_MGMT_2, 0x00);	  //0x6C
	MPU6050_WriteReg(SMPLRT_DIV, 0x07);   //0x19   陀螺仪采样率
	MPU6050_WriteReg(CONFIG, 0x06);       //0x1A   低通滤波频率
	MPU6050_WriteReg(GYRO_CONFIG, 0x18);  //0x1B   陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
	MPU6050_WriteReg(ACCEL_CONFIG, 0x18); //0x1C   加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
}

//**************************************
//读取ID
//**************************************
uint8_t MPU6050_ReadID(uint8_t reg_address)
{
	uint8_t id = 0;
	MPU6050_ReadReg(reg_address,&id,1);
	return id;
}


//**************************************
//读取陀螺仪的原始数据
//**************************************
void MPU6050_ReadGyro(int16_t *gyro_data)
{
	uint8_t buf[6] = {0};
	MPU6050_ReadReg(GYRO_XOUT_H,buf,6);
	gyro_data[0] = (buf[0]<<8)| buf[1];
	gyro_data[1] = (buf[2]<<8)| buf[3];
	gyro_data[2] = (buf[4]<<8)| buf[5];
}

//**************************************
//读取加速度计的原始数据
//**************************************
void MPU6050_ReadAccel(int16_t *accel_data)
{
	uint8_t buf[6] = {0};
	MPU6050_ReadReg(ACCEL_XOUT_H,buf,6);
	accel_data[0] = (buf[0]<<8)| buf[1];
	accel_data[1] = (buf[2]<<8)| buf[3];
	accel_data[2] = (buf[4]<<8)| buf[5];
}

//**************************************
//读取温度计的原始数据
//**************************************
void MPU6050_ReadTemp(int16_t *temp_data)
{
	uint8_t buf[2] = {0};
	MPU6050_ReadReg(TEMP_OUT_H,buf,2);
	*temp_data = (buf[0]<<8)| buf[1];
}

//**************************************
//读取温度计的实际数据
//**************************************
void MPU6050_ReadRealTemp(int16_t *temp_data)
{
	uint8_t buf[2] = {0};
	MPU6050_ReadReg(TEMP_OUT_H,buf,2);
	*temp_data = (buf[0]<<8)| buf[1];
	*temp_data = (double)((*temp_data)/340.0)+36.63;
}


//-------------------------------------暂时保留-------------------------------------//
//**************************************
//合成数据
//**************************************
int16_t MPU6050_GetData(uint8_t REG_Address)
{
	uint8_t H,L;
	H=Single_ReadI2C(REG_Address);
	L=Single_ReadI2C(REG_Address+1);
	return (H<<8)+L;   //合成数据
}
//-------------------------------------暂时保留-------------------------------------//


extern struct inv_sensor_cal_t sensors;
volatile uint32_t hal_timestamp = 0;
volatile unsigned char rx_new;
unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

inv_error_t result;
unsigned char accel_fsr,  new_temp = 0;
unsigned short gyro_rate, gyro_fsr;
unsigned long timestamp;
struct int_param_s int_param;


/**
 * @func	read_from_mpl
 * @brief	从MPL库获取数据，并将获取的数据添加到
			inv_get_sensor_type_xxx APIs，进行新数据与旧数据的区分
 * @param 	无
 * @retval	无
 **/
static void read_from_mpl(void)
{
    long msg, data[9];
    int8_t accuracy;
    unsigned long timestamp;
    float float_data[3] = {0};

    MPU_DEBUG_FUNC();
    if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)) 
	{
		
		/* 将四元数数据包发送到PC上位机，使用其来直观的表示3D四元数 
		   因此每次MPL有新数据时都会执行该函数*/
        eMPL_send_quat(data);

        /* 可以使用USB串口命令发送或禁止特定的数据包 */
        if (hal.report & PRINT_QUAT)
            eMPL_send_data(PACKET_DATA_QUAT, data);
    }
    if (hal.report & PRINT_ACCEL) 
	{
        if (inv_get_sensor_type_accel(data, &accuracy,(inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_ACCEL, data);
    }
    if (hal.report & PRINT_GYRO) 
	{
        if (inv_get_sensor_type_gyro(data, &accuracy,(inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_GYRO, data);

    }

    if (hal.report & PRINT_EULER) 
	{
        if (inv_get_sensor_type_euler(data, &accuracy,(inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_EULER, data);
			
    }
	
	/**************************发送数据到匿名四轴上位机********************************/
    if (1)
    {
				
		unsigned long timestamp,step_count,walk_time;
			
		/* 获取欧拉角 */
		if (inv_get_sensor_type_euler(data, &accuracy,(inv_time_t*)&timestamp))
		{
			/* 读出的数据类型是Q16格式，所以左移16位 */
			float Pitch,Roll,Yaw;
			Pitch =data[0]*1.0/(1<<16) ;
			Roll = data[1]*1.0/(1<<16);
			Yaw = data[2]*1.0/(1<<16);
			
			//-----------------------------------//
			OLED_ShowString(1, 1, "Pitch:");
			OLED_ShowString(2, 1, "Roll :");
			OLED_ShowString(3, 1, "Yaw  :");
			OLED_ShowSignedNum(1, 7, Pitch, 3);
			OLED_ShowSignedNum(2, 7, Roll, 3);
			OLED_ShowSignedNum(3, 7, Yaw, 3);			
			
			
			//-----------------------------------//
							
			/* 向匿名上位机发送姿态数据 */
			ANO_DT_SendStatus(Roll,Pitch,Yaw);
			/* 向匿名上位机发送传感器原始数据 */
			ANO_DT_SendSenser((int16_t *)&sensors.gyro.raw,(int16_t *)&sensors.accel.raw);
						
		}
						
		/* 获取计步数 */        
		get_tick_count(&timestamp);
		if (timestamp > hal.next_pedo_ms) 
		{
			hal.next_pedo_ms = timestamp + PEDO_READ_MS;
			dmp_get_pedometer_step_count(&step_count);
			dmp_get_pedometer_walk_time(&walk_time);		
		}
	}


    if (hal.report & PRINT_ROT_MAT) 
	{
        if (inv_get_sensor_type_rot_mat(data, &accuracy,(inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_ROT, data);
    }
    if (hal.report & PRINT_HEADING) 
	{
        if (inv_get_sensor_type_heading(data, &accuracy,(inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_HEADING, data);
    }
    if (hal.report & PRINT_LINEAR_ACCEL) 
	{
        if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp)) 
		{
        	MPL_LOGI("Linear Accel: %7.5f %7.5f %7.5f\r\n",
        	float_data[0], float_data[1], float_data[2]);                                        
         }
    }
    if (hal.report & PRINT_GRAVITY_VECTOR) 
	{
		if (inv_get_sensor_type_gravity(float_data, &accuracy,(inv_time_t*)&timestamp))
		{
            MPL_LOGI("Gravity Vector: %7.5f %7.5f %7.5f\r\n",
			float_data[0], float_data[1], float_data[2]);
		}
    }
    if (hal.report & PRINT_PEDO) 
	{
        unsigned long timestamp;
        get_tick_count(&timestamp);
        if (timestamp > hal.next_pedo_ms) 
		{
            hal.next_pedo_ms = timestamp + PEDO_READ_MS;
            unsigned long step_count, walk_time;
            dmp_get_pedometer_step_count(&step_count);
            dmp_get_pedometer_walk_time(&walk_time);
            MPL_LOGI("Walked %ld steps over %ld milliseconds..\n", step_count,
            walk_time);
        }
    }
	
	/* 只要MPL检测到运动状态发生变化，就可以通知该应用程序 
	   在本示例中，我们使用串口输出调试信息表示当前的运动状态 */
    msg = inv_get_message_level_0(INV_MSG_MOTION_EVENT | INV_MSG_NO_MOTION_EVENT);
    if (msg) 
	{
		if (msg & INV_MSG_MOTION_EVENT) 
		{
            MPL_LOGI("Motion!\n");
        } 
		else if (msg & INV_MSG_NO_MOTION_EVENT) 
		{
            MPL_LOGI("No motion!\n");
        }
    }
}


/**
 * @func	setup_gyro
 * @brief	陀螺仪工作配置函数
 * @param 	无
 * @retval	无
 **/
static void setup_gyro(void)
{
    unsigned char mask = 0, lp_accel_was_on = 0;

    MPU_DEBUG_FUNC();
    if (hal.sensors & ACCEL_ON)
        mask |= INV_XYZ_ACCEL;
    if (hal.sensors & GYRO_ON) 
	{
        mask |= INV_XYZ_GYRO;
        lp_accel_was_on |= hal.lp_accel_mode;
    }
#ifdef COMPASS_ENABLED
    if (hal.sensors & COMPASS_ON) 
	{
        mask |= INV_XYZ_COMPASS;
        lp_accel_was_on |= hal.lp_accel_mode;
    }
#endif
	
	/* 如果需要电源转换，则应在仍启用传感器掩码的情况下调用此功能 
	   驱动程序将关闭此掩码中未包含的所有传感器 */
    mpu_set_sensors(mask);
    mpu_configure_fifo(mask);
    if (lp_accel_was_on) 
	{
        unsigned short rate;
        hal.lp_accel_mode = 0;
        /* 退出LP加速度，将新的加速度采样率通知MPL */
        mpu_get_sample_rate(&rate);
        inv_set_accel_sample_rate(1000000L / rate);
    }
}


/**
 * @func	tap_cb
 * @brief	
 * @param 	无
 * @retval	无
 **/
static void tap_cb(unsigned char direction, unsigned char count)
{
	MPU_DEBUG_FUNC();
    switch (direction) 
	{
		case TAP_X_UP:
			MPL_LOGI("Tap X+ ");
			break;
		case TAP_X_DOWN:
			MPL_LOGI("Tap X- ");
			break;
		case TAP_Y_UP:
			MPL_LOGI("Tap Y+ ");
			break;
		case TAP_Y_DOWN:
			MPL_LOGI("Tap Y- ");
			break;
		case TAP_Z_UP:
			MPL_LOGI("Tap Z+ ");
			break;
		case TAP_Z_DOWN:
			MPL_LOGI("Tap Z- ");
			break;
		default:
			return;
    }
    MPL_LOGI("x%d\n", count);
	
    return;
}


/**
 * @func	android_orient_cb
 * @brief	
 * @param 	无
 * @retval	无
 **/
static void android_orient_cb(unsigned char orientation)
{
	MPU_DEBUG_FUNC();
	switch (orientation) 
	{
		case ANDROID_ORIENT_PORTRAIT:
			MPL_LOGI("Portrait\n");
			break;
		case ANDROID_ORIENT_LANDSCAPE:
			MPL_LOGI("Landscape\n");
			break;
		case ANDROID_ORIENT_REVERSE_PORTRAIT:
			MPL_LOGI("Reverse Portrait\n");
			break;
		case ANDROID_ORIENT_REVERSE_LANDSCAPE:
			MPL_LOGI("Reverse Landscape\n");
			break;
		default:
			return;
	}
}


/**
 * @func	run_self_test
 * @brief	自检函数，用于坐标原标定
 * @param 	无
 * @retval	无
 **/
static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];
	
    MPU_DEBUG_FUNC();
#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) 
	{
		 /* 自检通过， 我们能够在这里获取真实的陀螺仪数据，因此我们现在需要更新校准数据 */
		MPL_LOGI("Passed!\n");
        MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",accel[0]/65536.f,accel[1]/65536.f,accel[2]/65536.f);
        MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",gyro[0]/65536.f,gyro[1]/65536.f,gyro[2]/65536.f);
       
#ifdef USE_CAL_HW_REGISTERS
		
		/* 这部分代码使用的是MPUxxxx器件中的偏移寄存器，而不是将校准数据推动到MPL软件库 */
        unsigned char i = 0;

        for(i = 0; i<3; i++) 
		{
			gyro[i] = (long)(gyro[i] * 32.8f); 	// 转换为 正负1000dps
        	accel[i] *= 2048.f; 				// 转换为 正负16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#else
		
		/* 将校准后的数据推送到MPL库 */
		/* MPL期望硬件单元<<16的数据，但是自检返回g's << 16的数据 */
    	unsigned short accel_sens;
    	float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
		
#endif
    }
    else 
	{
		if (!(result & 0x1))
			MPL_LOGE("Gyro failed.\n");
        if (!(result & 0x2))
            MPL_LOGE("Accel failed.\n");
		if (!(result & 0x4))
            MPL_LOGE("Compass failed.\n");
     }

}


/**
 * @func	handle_input
 * @brief	
 * @param 	无
 * @retval	无
 **/
static void handle_input(void)
{
  
    char c = USART_ReceiveData(USART1);
    MPU_DEBUG_FUNC();
	
    switch (c) 
	{
		/* 这些命令用来关闭单个传感器 */
		case '8':
			hal.sensors ^= ACCEL_ON;
			setup_gyro();
			if (!(hal.sensors & ACCEL_ON))
				inv_accel_was_turned_off();
			break;
		case '9':
			hal.sensors ^= GYRO_ON;
			setup_gyro();
			if (!(hal.sensors & GYRO_ON))
				inv_gyro_was_turned_off();
			break;
#ifdef COMPASS_ENABLED
		case '0':
			hal.sensors ^= COMPASS_ON;
			setup_gyro();
			if (!(hal.sensors & COMPASS_ON))
				inv_compass_was_turned_off();
			break;
#endif
		/* 这些命令将单个传感器数据或融合数据发送到PC上位机 */
		case 'a':
			hal.report ^= PRINT_ACCEL;
			break;
		case 'g':
			hal.report ^= PRINT_GYRO;
			break;
#ifdef COMPASS_ENABLED
		case 'c':
			hal.report ^= PRINT_COMPASS;
			break;
#endif
		case 'e':
			hal.report ^= PRINT_EULER;
			break;
		case 'r':
			hal.report ^= PRINT_ROT_MAT;
			break;
		case 'q':
			hal.report ^= PRINT_QUAT;
			break;
		case 'h':
			hal.report ^= PRINT_HEADING;
			break;
		case 'i':
			hal.report ^= PRINT_LINEAR_ACCEL;
			break;
		case 'o':
			hal.report ^= PRINT_GRAVITY_VECTOR;
			break;
#ifdef COMPASS_ENABLED
		case 'w':
			send_status_compass();
			break;
#endif
		/* 此命令打印出每个陀螺仪寄存器的值以进行调试。如果禁用日志记录，则此功能无效 */
		case 'd':
			mpu_reg_dump();
			break;
		/* 测试低功耗加速模式 */
		case 'p':
        if (hal.dmp_on)
            /* LP加速模式与DMP不兼容 */
            break;
        mpu_lp_accel_mode(20);
		/* 启用LP加速度模式时，驱动程序将自动为锁存的中断配置硬件，
		   但是，MCU有时会错过上升/下降沿，并且永远不会设置hal.new_gyro标志。
		   为避免锁定在这种状态下，我们覆盖了驱动程序的配置并坚持使用未锁定的中断模式 */
		/* 要求：MCU支持电平触发的中断 */
        mpu_set_int_latched(0);
        hal.sensors &= ~(GYRO_ON|COMPASS_ON);
        hal.sensors |= ACCEL_ON;
        hal.lp_accel_mode = 1;
        inv_gyro_was_turned_off();
        inv_compass_was_turned_off();
        break;
		/* 硬件自检可以在不完全与MPL交互的情况下运行，应为它完全位于陀螺仪驱动程序中
		   假定已启用日志记录，否则，可以在此处使用几个LED来显示测试结果 */
		case 't':
			run_self_test();
			/* 告诉MPL传感器已关闭 */
			inv_accel_was_turned_off();
			inv_gyro_was_turned_off();
			inv_compass_was_turned_off();
        break;
		/* 根据你的程序，可能需要更改或更慢速率的传感器数据，这些命令可以加快或减慢将传感器数据推送到MPL的速度， */
		/* 在本示例中，指南针速率始终不变 */
		case '1':
			if (hal.dmp_on) 
			{
				dmp_set_fifo_rate(10);
				inv_set_quat_sample_rate(100000L);
			} 
			else
				mpu_set_sample_rate(10);
				inv_set_gyro_sample_rate(100000L);
				inv_set_accel_sample_rate(100000L);
			break;
		case '2':
			if (hal.dmp_on) 
			{
				dmp_set_fifo_rate(20);
				inv_set_quat_sample_rate(50000L);
			} 
			else
				mpu_set_sample_rate(20);
				inv_set_gyro_sample_rate(50000L);
				inv_set_accel_sample_rate(50000L);
			break;
		case '3':
			if (hal.dmp_on) 
			{
				dmp_set_fifo_rate(40);
				inv_set_quat_sample_rate(25000L);
			} 
			else
				mpu_set_sample_rate(40);
				inv_set_gyro_sample_rate(25000L);
				inv_set_accel_sample_rate(25000L);
			break;
		case '4':
			if (hal.dmp_on) 
			{
				dmp_set_fifo_rate(50);
				inv_set_quat_sample_rate(20000L);
			} 
			else
				mpu_set_sample_rate(50);
				inv_set_gyro_sample_rate(20000L);
				inv_set_accel_sample_rate(20000L);
			break;
		case '5':
			if (hal.dmp_on) 
			{
				dmp_set_fifo_rate(100);
				inv_set_quat_sample_rate(10000L);
			} 
			else
				mpu_set_sample_rate(100);
				inv_set_gyro_sample_rate(10000L);
				inv_set_accel_sample_rate(10000L);
			break;
		case ',':
		/* 将硬件设置为仅在手势事件时中断，此功能对于保持MCU处于休眠状态，直到DMP检测到发生轻敲或定向事件为止 */	
			dmp_set_interrupt_mode(DMP_INT_GESTURE);
			break;
		case '.':
			/* 将硬件设置为周期性中断 */
			dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
			break;
		case '6':
			/* 切换计步器先显示 */
			hal.report ^= PRINT_PEDO;
			break;
		case '7':
			/* 重置计步器 */
			dmp_set_pedometer_step_count(0);
			dmp_set_pedometer_walk_time(0);
			break;
		case 'f':
			if (hal.lp_accel_mode)
			/* LP加速模式与DMP不兼容 */
			return;
			/* 切换DMP */
			if (hal.dmp_on) 
			{
				unsigned short dmp_rate;
				unsigned char mask = 0;
				hal.dmp_on = 0;
				mpu_set_dmp_state(0);
				/* 恢复FIFO设置 */
				if (hal.sensors & ACCEL_ON)
					mask |= INV_XYZ_ACCEL;
				if (hal.sensors & GYRO_ON)
					mask |= INV_XYZ_GYRO;
				if (hal.sensors & COMPASS_ON)
					mask |= INV_XYZ_COMPASS;
				mpu_configure_fifo(mask);
				/* 使用DMP时，硬件采样率固定位200Hz，并且DMP配置为使用函数dmp_set_fifo_rate对FIFO输出进行降低采样率
				   但是，当DMP关闭时，采样率保持在200Hz，可以在inv_mpu.c文件中进行处理，但需要引用inv_mpu_dmp_motion_driver.c 
				   为了避免这种情况，我们将额外的逻辑代码放在应用程序层中 */
				dmp_get_fifo_rate(&dmp_rate);
				mpu_set_sample_rate(dmp_rate);
				inv_quaternion_sensor_was_turned_off();
				MPL_LOGI("DMP disabled.\n");
			} 
			else 
			{
				unsigned short sample_rate;
				hal.dmp_on = 1;
				/* 保持当前FIFO速率 */
				mpu_get_sample_rate(&sample_rate);
				dmp_set_fifo_rate(sample_rate);
				inv_set_quat_sample_rate(1000000L / sample_rate);
				mpu_set_dmp_state(1);
				MPL_LOGI("DMP enabled.\n");
			}
			break;
		case 'm':
			/* 测试运动中断硬件功能 */
			#ifndef MPU6050 	// 不适用MPU6050产品
				hal.motion_int_mode = 1;
			#endif 
			break;
		case 'v':
			/* 切换LP四元数，可以在运行时启用/禁用DMP功能，对其它功能可以使用相同的方法 */
			hal.dmp_features ^= DMP_FEATURE_6X_LP_QUAT;
			dmp_enable_feature(hal.dmp_features);
			if (!(hal.dmp_features & DMP_FEATURE_6X_LP_QUAT)) 
			{
				inv_quaternion_sensor_was_turned_off();
				MPL_LOGI("LP quaternion disabled.\n");
			} 
			else
				MPL_LOGI("LP quaternion enabled.\n");
			break;
		default:
			break;
    }
    hal.rx.cmd = 0;
}


/**
 * @func	gyro_data_ready_cb
 * @brief	设置标志变量hal.new_gyro，以通知MPL库有了新的数据
			每当有新的数据产生时，本函数会被外部中断服务函数调用
			在本工程中，此函数设置的标志位用于指示及保护FIFO缓冲区
 * @param 	无
 * @retval	无
 **/
void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}


/**
 * @func	MPU6050_FUNC1
 * @brief	函数1
 * @param 	无
 * @retval	无
 **/
void MPU6050_FUNC1(void)
{
	printf("mpu 6050 test start\r\n");
	result = mpu_init(&int_param);
	if (result) 	
		MPL_LOGE("Could not initialize gyro.result =  %d\n",result);
	else
		printf("mpu initialize Done\r\n");
		
	/* 如果你未使用MPU9150，也没有使用DMP功能，这个函数会将所有的从机置于总线上 */
    /*  mpu_set_bypass(1); */
     
	result = inv_init_mpl();
	if (result) 
	{
		MPL_LOGE("Could not initialize MPL.\n");
	}
	else
		printf("mpl initialize Done\r\n");
	  
	/* 计算6轴和9轴传感器的四元数 */
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
	
    /* 无运动状态时更新陀螺仪 */
    /* 注：下面这三个函数的功能是各不相同的 */ 
    inv_enable_fast_nomot();
    // inv_enable_motion_no_motion(); 
    // inv_set_no_motion_time(1000); 


    /* 当温度变化时更新陀螺仪数据 */
    inv_enable_gyro_tc();

	/* 允许 read_from_mpl 使用 MPL APIs */
    inv_enable_eMPL_outputs();

	result = inv_start_mpl();
	if (result == INV_ERROR_NOT_AUTHORIZED) 
	{
		while (1) 
		{
			MPL_LOGE("Not authorized.\n");
		}
	}
	if (result) 
	{
		MPL_LOGE("Could not start the MPL.\n");
	}

    /* 设置寄存器，开启陀螺仪 */
    /* 唤醒所有传感器*/
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* 将陀螺仪和加速度数据都送入FIFO */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
	
    /* 重新读取配置，确认前面的设置成功 */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
	
    /* 与MPL同步程序驱动程序配置 */
    /* 设置每毫秒的采样率*/
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);

	/* 设置芯片到主体的方向矩阵 */
    /* 将硬件单位设置为 dps/g's/度 的比例因子 */   
    inv_set_gyro_orientation_and_scale(inv_orientation_matrix_to_scalar(gyro_pdata.orientation),(long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(inv_orientation_matrix_to_scalar(gyro_pdata.orientation),(long)accel_fsr<<15);

    /* 初始化硬件状态相关变量 */
    hal.sensors = ACCEL_ON | GYRO_ON;

    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

	/* 指南针读取获取时间戳 */
	get_tick_count(&timestamp);

    /* 初始化DMP步骤：
     * 1. 调用dmp_load_motion_driver_firmware()
		  它会把inv_mpu_dmp_motion_driver.h文件中的DMP固件写入到MPU的存储空间
     * 2. 把陀螺仪和加速度计的原始数据矩阵送入DMP
     * 3. 注册姿态回调函数，除非相应的特性使能了，否则该回调函数不会被执行
     * 4. 调用 dmp_enable_feature(mask) 使能不同的特性
     * 5. 调用 dmp_set_fifo_rate(freq) 设置DMP输出频率
     * 6. 调用特定的特性控制相关的函数
     *
     * 调用 mpu_set_dmp_state(1)使能DMP，该函数可在DMP运行时被重复调用设置使能或关闭
     *
     * 以下是inv_mpu_dmp_motion_driver.c文件中DMP固件提供的特性的简介：
  
	 * DMP_FEATURE_LP_QUAT: 使用DMP以200Hz的频率产生一个只包含陀螺仪的四元数数据
							以高速的状态解算陀螺仪数据，减少错误（相对于使用MCu以一个低采样率的方式采样）
	 * DMP_FEATURE_6X_LP_QUAT: 使用DMP以200Hz的频率产生 陀螺仪/加速度计 四元数，它不能与前面的DMP_FEATURE_LP_QUAT同时使用
	 * DMP_FEATURE_TAP: 检测 X，Y，和 Z 轴
	 * DMP_FEATURE_ANDROID_ORIENT: 谷歌屏幕翻转算法，挡屏幕翻转时，在四个方向产生一个事件
	 * DMP_FEATURE_GYRO_CAL: 若8s内都没有运动，计算陀螺仪数据
	 * DMP_FEATURE_SEND_RAW_ACCEL: 添加原始加速度计数据到FIFO
	 * DMP_FEATURE_SEND_RAW_GYRO: 添加原始陀螺仪数据到FIFO
	 * DMP_FEATURE_SEND_CAL_GYRO: 添加校准后的陀螺仪数据到FIFO，不能与DMP_FEATURE_SEND_RAW_GYRO同时使用  
     */
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
    dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
	
    
     /* 已知错误 -
	    DMP启用后将以200Hz采样传感器数据，并以dmp_set_fifo_rate API中指定的速率传输到FIFO
	    一旦将采样数据放入FIFO，DMP将发送一个中断，因此，如果dmp_set_fifo_rate为25Hz，则是来自MPU设备的25Hz中断。
	    一个已知的问题：如果不启用DMP_FEATURE_TAP，则即使将FIFO速率设置为其他速率，中断也会以200Hz的频率运行
	    为避免此问题，请包括DMP_FEATURE_TAP  */  
	 /* DMP传感器融合仅适用于gyro at +-2000dps and accel +-2G */
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
	hal.dmp_on = 1;
}


/**
 * @func	MPU6050_FUNC2
 * @brief	函数2
 * @param 	无
 * @retval	无
 **/
void MPU6050_FUNC2(void)
{
	//while(1)
	//{
		unsigned long sensor_timestamp;
		int new_data = 0;
		
		if (USART_GetFlagStatus (USART1, USART_FLAG_RXNE)) 
		{
			
			/* 已通过串口接收到数据，使用handle_input来处理串口接收到的命令 */
		    /* 这部分是处理Python官方上位机串口发送指令的 */
			
			USART_ClearFlag(USART1, USART_FLAG_RXNE);

			handle_input();
					
		}
		 /* 获取时间戳 */
		get_tick_count(&timestamp);

		/* 温度数据不需要像陀螺仪数据那样每次都采样，这里设置隔一段时间采样 */
        if (timestamp > hal.next_temp_ms) 
		{
            hal.next_temp_ms = timestamp + TEMP_READ_MS;
            new_temp = 1;
        }

		if (hal.motion_int_mode) 
		{
			/* 启动运动中断 */
			mpu_lp_motion_interrupt(500, 1, 5);
			/* 告诉MPL传感器已关闭 */
			inv_accel_was_turned_off();
			inv_gyro_was_turned_off();
			inv_compass_was_turned_off();
			inv_quaternion_sensor_was_turned_off();
			/* 等待MPU中断 */
			while (!hal.new_gyro) {}
			/* 恢复之前的传感器配置 */
			mpu_lp_motion_interrupt(0, 0, 0);
			hal.motion_int_mode = 0;
		}
		
		/* 如果满足条件则不执行循环体的剩余内容，跳出循环，执行下一次循环 */
		if (!hal.sensors || !hal.new_gyro) 
		{
			//continue;
			goto next;
		}    
        if (hal.new_gyro && hal.lp_accel_mode) 
		{
            short accel_short[3];
            long accel[3];
            mpu_get_accel_reg(accel_short, &sensor_timestamp);
            accel[0] = (long)accel_short[0];
            accel[1] = (long)accel_short[1];
            accel[2] = (long)accel_short[2];
            inv_build_accel(accel, 0, sensor_timestamp);
            new_data = 1;
            hal.new_gyro = 0;
        } 
		if (hal.new_gyro && hal.dmp_on) 
		{
            short gyro[3], accel_short[3], sensors;
            unsigned char more;
            long accel[3], quat[4], temperature;
			
			/* 当使用DMP时，本函数从FIFO读取新数据，FIFO中存储了陀螺仪、加速度、四元数及手势数据，
			   传感器参数可告知调用者哪个有了新数据 
			   例如：if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT)，那么FIFO中就不包含加速度数据
     		   手势数据的解算由是否产生手势运动事件来触发，若产生了事件，应用函数会使用回调函数来通知
			   如果FIFO中有剩余数据包，则more参数为非零 */
            dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
            if (!more)
                hal.new_gyro = 0;
            if (sensors & INV_XYZ_GYRO) 
			{
                /* 将新数据推送到MPL */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp) 
				{
                    new_temp = 0;
                    /* 温度仅用于陀螺仪温度补偿 */
                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            if (sensors & INV_XYZ_ACCEL) 
			{
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
            if (sensors & INV_WXYZ_QUAT) 
			{
                inv_build_quat(quat, 0, sensor_timestamp);
                new_data = 1;
            }
        } 
		else if (hal.new_gyro) 
		{
            short gyro[3], accel_short[3];
            unsigned char sensors, more;
            long accel[3], temperature;
			 /* 此函数从FIFO获取新数据，FIFO可以包含陀螺仪，加速度或两者都包含，或者都不包含
			    sensors参数告诉调用者哪些数据字段实际填充了了新数据
			    例如：if sensors == INV_XYZ_GYRO，则FIFO不会填充加速度数据，如果FIFO中有剩余数据包，
			    则more参数为非零，硬件可以使用此信息来增加调用此函数的频率 */
            hal.new_gyro = 0;
            mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
                &sensors, &more);
            if (more)
                hal.new_gyro = 1;
            if (sensors & INV_XYZ_GYRO) 
			{
                /* 将新数据推送到MPL */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp) 
				{
                    new_temp = 0;
                    /* 温度仅用于陀螺仪温度补偿 */
                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            if (sensors & INV_XYZ_ACCEL) 
			{
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
        }

        if (new_data) 
		{
            inv_execute_on_data();
			/* 本函数读取补偿后的传感器数据和经过MPL传感器融合后输出的数据
			输出的格式见 eMPL_outputs.c 文件，这个函数在主机需要数据的时候调用即可，对频率无要求*/
            read_from_mpl();
        }
		next: __NOP();//空指令
		
	//}
}




