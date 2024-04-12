#ifndef ___MPU6050_H
#define ___MPU6050_H

#include "stm32f10x.h"

#define uchar unsigned char
#define uint unsigned char	
	
//****************************************
// 定义MPU6050内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	PWR_MGMT_2		0x6C
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	SlaveAddress	0xD0	//IIC写入时的地址字节数据，+1为读取


/* 从MPL中读取的数据 */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)


#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* 默认采样率 */
#define DEFAULT_MPU_HZ  (100)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)


struct rx_s 
{
    unsigned char header[3];
    unsigned char cmd;
};


/* 每当有新的陀螺仪数据可用时，就在ISR上下文中调用此函数
   在本例中，设置了一个标志来保护FIFO读取功能 */
struct hal_s 
{
	unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};

static struct hal_s hal = {0};


/* 特定平台信息，有点像 boardfile */
struct platform_data_s 
{
    signed char orientation[9];
};


/* 传感器可以任何方向安装到板上
   下面显示的安装矩阵告诉MPL如何旋转来自驱动器的原始数据， */
/* 注：以下矩阵涉及Invensense内部测试板上的配置，如果需要，
   请修改矩阵以适配你的特定设置的芯片到主体的矩阵*/
static struct platform_data_s gyro_pdata = {
		
	.orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};


#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY


static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY

static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif


/* 调试信息输出 */
#define MPU_DEBUG_ON         0
#define MPU_DEBUG_FUNC_ON    0

#define MPU_INFO(fmt,arg...)           printf("<<-MPU-INFO->> "fmt"\n",##arg)
#define MPU_ERROR(fmt,arg...)          printf("<<-MPU-ERROR->> "fmt"\n",##arg)
#define MPU_DEBUG(fmt,arg...)          do{\
                                          if(MPU_DEBUG_ON)\
                                          printf("<<-MPU-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                         }while(0)

#define MPU_DEBUG_FUNC()               do{\
										  if(MPU_DEBUG_FUNC_ON)\
                                          printf("<<-MPU-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
										 }while(0)

void MPU6050_FUNC1(void);
void MPU6050_FUNC2(void);	
extern void gyro_data_ready_cb(void);
void UpdateAngle(void);

void InitMPU6050(void);

uint8_t MPU6050_ReadID(uint8_t reg_address);
void MPU6050_ReadGyro(int16_t *gyro_data);
void MPU6050_ReadAccel(int16_t *accel_data);
void MPU6050_ReadTemp(int16_t *temp_data);
void MPU6050_ReadRealTemp(int16_t *temp_data);
		
void MPU6050_WriteReg(uint8_t reg_address,uint8_t reg_data);
void MPU6050_ReadReg(uint8_t reg_address,uint8_t *reg_data,uint8_t length);


//-------------------------------------暂时保留-------------------------------------//
int16_t MPU6050_GetData(uint8_t REG_Address);
//-------------------------------------暂时保留-------------------------------------//
										 
uint8_t MPU6050_mpu_init(void);
uint8_t MPU6050_mpl_init(void);
uint8_t MPU6050_config(void);

//得到角度和加速度进行PID调参
extern float Pitch;
extern short gyro[3];
#endif
