#pragma once
#include <stm32f1xx_hal.h>
#include <arm_math.h>

typedef struct
{
	float x, y, z;
}IMU_Accel, IMU_Gyro, IMU_Mag;
typedef struct
{
	float pitch, roll, yaw;
}IMU_Angle;

extern IMU_Accel accel;
extern IMU_Gyro  gyro;
extern IMU_Mag   mag;
extern IMU_Angle angle;

#define	PWR_MGMT_1		0x6B	//µçÔ´¹ÜÀí£¬µäÐÍÖµ£º0x00(Õý³£ÆôÓÃ)
#define	SMPLRT_DIV		0x19	//ÍÓÂÝÒÇ²ÉÑùÂÊ£¬µäÐÍÖµ£º0x07(125Hz)
#define	CONFIG			0x1A	//0x05
#define	GYRO_CONFIG		0x1B	//0x08 Set the gyro scale to 500 °/s and FCHOICE_B
#define	ACCEL_CONFIG_1	0x1C    //0x08 Set the accel scale to 4g
#define ACCEL_CONFIG_2  0x1D    //0x05 Turn on the internal low-pass filter for accelerometer with 10.2Hz bandwidth
#define ASAX            0x10
#define ASAY            0x11
#define ASAZ            0x12

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define ACCEL_YOUT_H	0x3D
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


#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08

//****************************
#define	GYRO_ADDRESS   0xD0
#define MAG_ADDRESS    0x18
#define ACCEL_ADDRESS  0xD0

#define GPIO     GPIOB
#define SCL      GPIO_PIN_6
#define SDA      GPIO_PIN_7

#define SCL_H    GPIO->BSRR = SCL
#define SCL_L    GPIO->BRR  = SCL

#define SDA_H    GPIO->BSRR = SDA
#define SDA_L    GPIO->BRR  = SDA

#define SCL_read GPIO->IDR  & SCL
#define SDA_read GPIO->IDR  & SDA

void IMU_Init(void);
bool IMU_Measure(void);
void IMU_Calibrate(void);