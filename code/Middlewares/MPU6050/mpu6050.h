#ifndef __MPU6050_H
#define __MPU6050_H
#include "inv_mpu.h"

typedef struct
{
	short acc_x;
	short acc_y;
	short acc_z;
	
	short gyro_x;
	short gyro_y;
	short gyro_z;
	
	float pitch;
	float roll;
	float yaw;
}MPU6050_Data_Struct;

uint8_t MPU_Init(void);
uint8_t MPU_GetData(MPU6050_Data_Struct* MPU_Data);

uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr);
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr);
uint8_t MPU_Set_LPF(uint16_t lpf);
uint8_t MPU_Set_Rate(uint16_t rate);

short MPU_Get_Temperature(void);
uint8_t MPU_Get_Gyroscope(MPU6050_Data_Struct* MPU_Data);
uint8_t MPU_Get_Accelerometer(MPU6050_Data_Struct* MPU_Data);

#endif
