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
//MPU6050д�Ĵ���
//**************************************
void MPU6050_WriteReg(uint8_t reg_address,uint8_t reg_data)
{
	Sensors_I2C_WriteRegister(SlaveAddress,reg_address,1,&reg_data);
}

//**************************************
//MPU6050���Ĵ���
//**************************************
void MPU6050_ReadReg(uint8_t reg_address,uint8_t *reg_data,uint8_t length)
{
	Sensors_I2C_ReadRegister(SlaveAddress,reg_address,length,reg_data);
}

//**************************************
//��ʼ��MPU6050
//**************************************
void InitMPU6050(void)
{
	//----------------------------------------------------------------//
    int i=0,j=0;
    //�ڳ�ʼ��֮ǰҪ��ʱһ��ʱ�䣬��û����ʱ����ϵ�����ϵ����ݿ��ܻ����
    for (i=0; i<1000; i++) {
        for (j=0; j<1000; j++) {
            ;
        }
    }
	//----------------------------------------------------------------//
	MPU6050_WriteReg(PWR_MGMT_1, 0x00);	  //0x6B   ���˯�߲���ѡ���ڲ�8Mʱ��
	MPU6050_WriteReg(PWR_MGMT_2, 0x00);	  //0x6C
	MPU6050_WriteReg(SMPLRT_DIV, 0x07);   //0x19   �����ǲ�����
	MPU6050_WriteReg(CONFIG, 0x06);       //0x1A   ��ͨ�˲�Ƶ��
	MPU6050_WriteReg(GYRO_CONFIG, 0x18);  //0x1B   �������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
	MPU6050_WriteReg(ACCEL_CONFIG, 0x18); //0x1C   ���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
}

//**************************************
//��ȡID
//**************************************
uint8_t MPU6050_ReadID(uint8_t reg_address)
{
	uint8_t id = 0;
	MPU6050_ReadReg(reg_address,&id,1);
	return id;
}


//**************************************
//��ȡ�����ǵ�ԭʼ����
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
//��ȡ���ٶȼƵ�ԭʼ����
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
//��ȡ�¶ȼƵ�ԭʼ����
//**************************************
void MPU6050_ReadTemp(int16_t *temp_data)
{
	uint8_t buf[2] = {0};
	MPU6050_ReadReg(TEMP_OUT_H,buf,2);
	*temp_data = (buf[0]<<8)| buf[1];
}

//**************************************
//��ȡ�¶ȼƵ�ʵ������
//**************************************
void MPU6050_ReadRealTemp(int16_t *temp_data)
{
	uint8_t buf[2] = {0};
	MPU6050_ReadReg(TEMP_OUT_H,buf,2);
	*temp_data = (buf[0]<<8)| buf[1];
	*temp_data = (double)((*temp_data)/340.0)+36.63;
}


//-------------------------------------��ʱ����-------------------------------------//
//**************************************
//�ϳ�����
//**************************************
int16_t MPU6050_GetData(uint8_t REG_Address)
{
	uint8_t H,L;
	H=Single_ReadI2C(REG_Address);
	L=Single_ReadI2C(REG_Address+1);
	return (H<<8)+L;   //�ϳ�����
}
//-------------------------------------��ʱ����-------------------------------------//


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
 * @brief	��MPL���ȡ���ݣ�������ȡ��������ӵ�
			inv_get_sensor_type_xxx APIs������������������ݵ�����
 * @param 	��
 * @retval	��
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
		
		/* ����Ԫ�����ݰ����͵�PC��λ����ʹ������ֱ�۵ı�ʾ3D��Ԫ�� 
		   ���ÿ��MPL��������ʱ����ִ�иú���*/
        eMPL_send_quat(data);

        /* ����ʹ��USB��������ͻ��ֹ�ض������ݰ� */
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
	
	/**************************�������ݵ�����������λ��********************************/
    if (1)
    {
				
		unsigned long timestamp,step_count,walk_time;
			
		/* ��ȡŷ���� */
		if (inv_get_sensor_type_euler(data, &accuracy,(inv_time_t*)&timestamp))
		{
			/* ����������������Q16��ʽ����������16λ */
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
							
			/* ��������λ��������̬���� */
			ANO_DT_SendStatus(Roll,Pitch,Yaw);
			/* ��������λ�����ʹ�����ԭʼ���� */
			ANO_DT_SendSenser((int16_t *)&sensors.gyro.raw,(int16_t *)&sensors.accel.raw);
						
		}
						
		/* ��ȡ�Ʋ��� */        
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
	
	/* ֻҪMPL��⵽�˶�״̬�����仯���Ϳ���֪ͨ��Ӧ�ó��� 
	   �ڱ�ʾ���У�����ʹ�ô������������Ϣ��ʾ��ǰ���˶�״̬ */
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
 * @brief	�����ǹ������ú���
 * @param 	��
 * @retval	��
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
	
	/* �����Ҫ��Դת������Ӧ�������ô��������������µ��ô˹��� 
	   �������򽫹رմ�������δ���������д����� */
    mpu_set_sensors(mask);
    mpu_configure_fifo(mask);
    if (lp_accel_was_on) 
	{
        unsigned short rate;
        hal.lp_accel_mode = 0;
        /* �˳�LP���ٶȣ����µļ��ٶȲ�����֪ͨMPL */
        mpu_get_sample_rate(&rate);
        inv_set_accel_sample_rate(1000000L / rate);
    }
}


/**
 * @func	tap_cb
 * @brief	
 * @param 	��
 * @retval	��
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
 * @param 	��
 * @retval	��
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
 * @brief	�Լ캯������������ԭ�궨
 * @param 	��
 * @retval	��
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
		 /* �Լ�ͨ���� �����ܹ��������ȡ��ʵ�����������ݣ��������������Ҫ����У׼���� */
		MPL_LOGI("Passed!\n");
        MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",accel[0]/65536.f,accel[1]/65536.f,accel[2]/65536.f);
        MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",gyro[0]/65536.f,gyro[1]/65536.f,gyro[2]/65536.f);
       
#ifdef USE_CAL_HW_REGISTERS
		
		/* �ⲿ�ִ���ʹ�õ���MPUxxxx�����е�ƫ�ƼĴ����������ǽ�У׼�����ƶ���MPL����� */
        unsigned char i = 0;

        for(i = 0; i<3; i++) 
		{
			gyro[i] = (long)(gyro[i] * 32.8f); 	// ת��Ϊ ����1000dps
        	accel[i] *= 2048.f; 				// ת��Ϊ ����16G
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
		
		/* ��У׼����������͵�MPL�� */
		/* MPL����Ӳ����Ԫ<<16�����ݣ������Լ췵��g's << 16������ */
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
 * @param 	��
 * @retval	��
 **/
static void handle_input(void)
{
  
    char c = USART_ReceiveData(USART1);
    MPU_DEBUG_FUNC();
	
    switch (c) 
	{
		/* ��Щ���������رյ��������� */
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
		/* ��Щ����������������ݻ��ں����ݷ��͵�PC��λ�� */
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
		/* �������ӡ��ÿ�������ǼĴ�����ֵ�Խ��е��ԡ����������־��¼����˹�����Ч */
		case 'd':
			mpu_reg_dump();
			break;
		/* ���Ե͹��ļ���ģʽ */
		case 'p':
        if (hal.dmp_on)
            /* LP����ģʽ��DMP������ */
            break;
        mpu_lp_accel_mode(20);
		/* ����LP���ٶ�ģʽʱ�����������Զ�Ϊ������ж�����Ӳ����
		   ���ǣ�MCU��ʱ��������/�½��أ�������Զ��������hal.new_gyro��־��
		   Ϊ��������������״̬�£����Ǹ�����������������ò����ʹ��δ�������ж�ģʽ */
		/* Ҫ��MCU֧�ֵ�ƽ�������ж� */
        mpu_set_int_latched(0);
        hal.sensors &= ~(GYRO_ON|COMPASS_ON);
        hal.sensors |= ACCEL_ON;
        hal.lp_accel_mode = 1;
        inv_gyro_was_turned_off();
        inv_compass_was_turned_off();
        break;
		/* Ӳ���Լ�����ڲ���ȫ��MPL��������������У�ӦΪ����ȫλ������������������
		   �ٶ���������־��¼�����򣬿����ڴ˴�ʹ�ü���LED����ʾ���Խ�� */
		case 't':
			run_self_test();
			/* ����MPL�������ѹر� */
			inv_accel_was_turned_off();
			inv_gyro_was_turned_off();
			inv_compass_was_turned_off();
        break;
		/* ������ĳ��򣬿�����Ҫ���Ļ�������ʵĴ��������ݣ���Щ������Լӿ��������������������͵�MPL���ٶȣ� */
		/* �ڱ�ʾ���У�ָ��������ʼ�ղ��� */
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
		/* ��Ӳ������Ϊ���������¼�ʱ�жϣ��˹��ܶ��ڱ���MCU��������״̬��ֱ��DMP��⵽�������û����¼�Ϊֹ */	
			dmp_set_interrupt_mode(DMP_INT_GESTURE);
			break;
		case '.':
			/* ��Ӳ������Ϊ�������ж� */
			dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
			break;
		case '6':
			/* �л��Ʋ�������ʾ */
			hal.report ^= PRINT_PEDO;
			break;
		case '7':
			/* ���üƲ��� */
			dmp_set_pedometer_step_count(0);
			dmp_set_pedometer_walk_time(0);
			break;
		case 'f':
			if (hal.lp_accel_mode)
			/* LP����ģʽ��DMP������ */
			return;
			/* �л�DMP */
			if (hal.dmp_on) 
			{
				unsigned short dmp_rate;
				unsigned char mask = 0;
				hal.dmp_on = 0;
				mpu_set_dmp_state(0);
				/* �ָ�FIFO���� */
				if (hal.sensors & ACCEL_ON)
					mask |= INV_XYZ_ACCEL;
				if (hal.sensors & GYRO_ON)
					mask |= INV_XYZ_GYRO;
				if (hal.sensors & COMPASS_ON)
					mask |= INV_XYZ_COMPASS;
				mpu_configure_fifo(mask);
				/* ʹ��DMPʱ��Ӳ�������ʹ̶�λ200Hz������DMP����Ϊʹ�ú���dmp_set_fifo_rate��FIFO������н��Ͳ�����
				   ���ǣ���DMP�ر�ʱ�������ʱ�����200Hz��������inv_mpu.c�ļ��н��д�������Ҫ����inv_mpu_dmp_motion_driver.c 
				   Ϊ�˱���������������ǽ�������߼��������Ӧ�ó������ */
				dmp_get_fifo_rate(&dmp_rate);
				mpu_set_sample_rate(dmp_rate);
				inv_quaternion_sensor_was_turned_off();
				MPL_LOGI("DMP disabled.\n");
			} 
			else 
			{
				unsigned short sample_rate;
				hal.dmp_on = 1;
				/* ���ֵ�ǰFIFO���� */
				mpu_get_sample_rate(&sample_rate);
				dmp_set_fifo_rate(sample_rate);
				inv_set_quat_sample_rate(1000000L / sample_rate);
				mpu_set_dmp_state(1);
				MPL_LOGI("DMP enabled.\n");
			}
			break;
		case 'm':
			/* �����˶��ж�Ӳ������ */
			#ifndef MPU6050 	// ������MPU6050��Ʒ
				hal.motion_int_mode = 1;
			#endif 
			break;
		case 'v':
			/* �л�LP��Ԫ��������������ʱ����/����DMP���ܣ����������ܿ���ʹ����ͬ�ķ��� */
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
 * @brief	���ñ�־����hal.new_gyro����֪ͨMPL�������µ�����
			ÿ�����µ����ݲ���ʱ���������ᱻ�ⲿ�жϷ���������
			�ڱ������У��˺������õı�־λ����ָʾ������FIFO������
 * @param 	��
 * @retval	��
 **/
void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}


/**
 * @func	MPU6050_FUNC1
 * @brief	����1
 * @param 	��
 * @retval	��
 **/
void MPU6050_FUNC1(void)
{
	printf("mpu 6050 test start\r\n");
	result = mpu_init(&int_param);
	if (result) 	
		MPL_LOGE("Could not initialize gyro.result =  %d\n",result);
	else
		printf("mpu initialize Done\r\n");
		
	/* �����δʹ��MPU9150��Ҳû��ʹ��DMP���ܣ���������Ὣ���еĴӻ����������� */
    /*  mpu_set_bypass(1); */
     
	result = inv_init_mpl();
	if (result) 
	{
		MPL_LOGE("Could not initialize MPL.\n");
	}
	else
		printf("mpl initialize Done\r\n");
	  
	/* ����6���9�ᴫ��������Ԫ�� */
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
	
    /* ���˶�״̬ʱ���������� */
    /* ע�����������������Ĺ����Ǹ�����ͬ�� */ 
    inv_enable_fast_nomot();
    // inv_enable_motion_no_motion(); 
    // inv_set_no_motion_time(1000); 


    /* ���¶ȱ仯ʱ�������������� */
    inv_enable_gyro_tc();

	/* ���� read_from_mpl ʹ�� MPL APIs */
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

    /* ���üĴ��������������� */
    /* �������д�����*/
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* �������Ǻͼ��ٶ����ݶ�����FIFO */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
	
    /* ���¶�ȡ���ã�ȷ��ǰ������óɹ� */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
	
    /* ��MPLͬ������������������ */
    /* ����ÿ����Ĳ�����*/
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);

	/* ����оƬ������ķ������ */
    /* ��Ӳ����λ����Ϊ dps/g's/�� �ı������� */   
    inv_set_gyro_orientation_and_scale(inv_orientation_matrix_to_scalar(gyro_pdata.orientation),(long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(inv_orientation_matrix_to_scalar(gyro_pdata.orientation),(long)accel_fsr<<15);

    /* ��ʼ��Ӳ��״̬��ر��� */
    hal.sensors = ACCEL_ON | GYRO_ON;

    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

	/* ָ�����ȡ��ȡʱ��� */
	get_tick_count(&timestamp);

    /* ��ʼ��DMP���裺
     * 1. ����dmp_load_motion_driver_firmware()
		  �����inv_mpu_dmp_motion_driver.h�ļ��е�DMP�̼�д�뵽MPU�Ĵ洢�ռ�
     * 2. �������Ǻͼ��ٶȼƵ�ԭʼ���ݾ�������DMP
     * 3. ע����̬�ص�������������Ӧ������ʹ���ˣ�����ûص��������ᱻִ��
     * 4. ���� dmp_enable_feature(mask) ʹ�ܲ�ͬ������
     * 5. ���� dmp_set_fifo_rate(freq) ����DMP���Ƶ��
     * 6. �����ض������Կ�����صĺ���
     *
     * ���� mpu_set_dmp_state(1)ʹ��DMP���ú�������DMP����ʱ���ظ���������ʹ�ܻ�ر�
     *
     * ������inv_mpu_dmp_motion_driver.c�ļ���DMP�̼��ṩ�����Եļ�飺
  
	 * DMP_FEATURE_LP_QUAT: ʹ��DMP��200Hz��Ƶ�ʲ���һ��ֻ���������ǵ���Ԫ������
							�Ը��ٵ�״̬�������������ݣ����ٴ��������ʹ��MCu��һ���Ͳ����ʵķ�ʽ������
	 * DMP_FEATURE_6X_LP_QUAT: ʹ��DMP��200Hz��Ƶ�ʲ��� ������/���ٶȼ� ��Ԫ������������ǰ���DMP_FEATURE_LP_QUATͬʱʹ��
	 * DMP_FEATURE_TAP: ��� X��Y���� Z ��
	 * DMP_FEATURE_ANDROID_ORIENT: �ȸ���Ļ��ת�㷨������Ļ��תʱ�����ĸ��������һ���¼�
	 * DMP_FEATURE_GYRO_CAL: ��8s�ڶ�û���˶�����������������
	 * DMP_FEATURE_SEND_RAW_ACCEL: ���ԭʼ���ٶȼ����ݵ�FIFO
	 * DMP_FEATURE_SEND_RAW_GYRO: ���ԭʼ���������ݵ�FIFO
	 * DMP_FEATURE_SEND_CAL_GYRO: ���У׼������������ݵ�FIFO��������DMP_FEATURE_SEND_RAW_GYROͬʱʹ��  
     */
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
    dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
	
    
     /* ��֪���� -
	    DMP���ú���200Hz�������������ݣ�����dmp_set_fifo_rate API��ָ�������ʴ��䵽FIFO
	    һ�����������ݷ���FIFO��DMP������һ���жϣ���ˣ����dmp_set_fifo_rateΪ25Hz����������MPU�豸��25Hz�жϡ�
	    һ����֪�����⣺���������DMP_FEATURE_TAP����ʹ��FIFO��������Ϊ�������ʣ��ж�Ҳ����200Hz��Ƶ������
	    Ϊ��������⣬�����DMP_FEATURE_TAP  */  
	 /* DMP�������ںϽ�������gyro at +-2000dps and accel +-2G */
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
 * @brief	����2
 * @param 	��
 * @retval	��
 **/
void MPU6050_FUNC2(void)
{
	//while(1)
	//{
		unsigned long sensor_timestamp;
		int new_data = 0;
		
		if (USART_GetFlagStatus (USART1, USART_FLAG_RXNE)) 
		{
			
			/* ��ͨ�����ڽ��յ����ݣ�ʹ��handle_input�������ڽ��յ������� */
		    /* �ⲿ���Ǵ���Python�ٷ���λ�����ڷ���ָ��� */
			
			USART_ClearFlag(USART1, USART_FLAG_RXNE);

			handle_input();
					
		}
		 /* ��ȡʱ��� */
		get_tick_count(&timestamp);

		/* �¶����ݲ���Ҫ����������������ÿ�ζ��������������ø�һ��ʱ����� */
        if (timestamp > hal.next_temp_ms) 
		{
            hal.next_temp_ms = timestamp + TEMP_READ_MS;
            new_temp = 1;
        }

		if (hal.motion_int_mode) 
		{
			/* �����˶��ж� */
			mpu_lp_motion_interrupt(500, 1, 5);
			/* ����MPL�������ѹر� */
			inv_accel_was_turned_off();
			inv_gyro_was_turned_off();
			inv_compass_was_turned_off();
			inv_quaternion_sensor_was_turned_off();
			/* �ȴ�MPU�ж� */
			while (!hal.new_gyro) {}
			/* �ָ�֮ǰ�Ĵ��������� */
			mpu_lp_motion_interrupt(0, 0, 0);
			hal.motion_int_mode = 0;
		}
		
		/* �������������ִ��ѭ�����ʣ�����ݣ�����ѭ����ִ����һ��ѭ�� */
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
			
			/* ��ʹ��DMPʱ����������FIFO��ȡ�����ݣ�FIFO�д洢�������ǡ����ٶȡ���Ԫ�����������ݣ�
			   �����������ɸ�֪�������ĸ����������� 
			   ���磺if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT)����ôFIFO�оͲ��������ٶ�����
     		   �������ݵĽ������Ƿ���������˶��¼������������������¼���Ӧ�ú�����ʹ�ûص�������֪ͨ
			   ���FIFO����ʣ�����ݰ�����more����Ϊ���� */
            dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
            if (!more)
                hal.new_gyro = 0;
            if (sensors & INV_XYZ_GYRO) 
			{
                /* �����������͵�MPL */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp) 
				{
                    new_temp = 0;
                    /* �¶Ƚ������������¶Ȳ��� */
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
			 /* �˺�����FIFO��ȡ�����ݣ�FIFO���԰��������ǣ����ٶȻ����߶����������߶�������
			    sensors�������ߵ�������Щ�����ֶ�ʵ���������������
			    ���磺if sensors == INV_XYZ_GYRO����FIFO���������ٶ����ݣ����FIFO����ʣ�����ݰ���
			    ��more����Ϊ���㣬Ӳ������ʹ�ô���Ϣ�����ӵ��ô˺�����Ƶ�� */
            hal.new_gyro = 0;
            mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
                &sensors, &more);
            if (more)
                hal.new_gyro = 1;
            if (sensors & INV_XYZ_GYRO) 
			{
                /* �����������͵�MPL */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp) 
				{
                    new_temp = 0;
                    /* �¶Ƚ������������¶Ȳ��� */
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
			/* ��������ȡ������Ĵ��������ݺ;���MPL�������ںϺ����������
			����ĸ�ʽ�� eMPL_outputs.c �ļ������������������Ҫ���ݵ�ʱ����ü��ɣ���Ƶ����Ҫ��*/
            read_from_mpl();
        }
		next: __NOP();//��ָ��
		
	//}
}




