/**
*********************************************************************************
* @file    control.h
* @author  Alex
* @version V1.0
* @date    2024-0414
* @brief   电机控制头文件
*********************************************************************************
*/

#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "motor.h"
#include "main.h"

/*PID参数结构体*/
typedef struct
{
	/*直立环*/
	float Balance_Kp;
	float Balance_Ki;
	float Balance_Kd;

	/*速度环*/
	float Velocity_Kp;
	float Velocity_Ki;
	float Velocity_Kd;

	/*转向环*/
	float  Turn_Kp;
	float  Turn_Ki;
	float  Turn_Kd;
}pid_arg;
extern pid_arg PID;

void Motor_init(void);
void Encoder_Init(void);
void Motor_setSpeed(uint8_t motorX, int16_t speed);
int16_t Motor_getSpeed(uint8_t EncoderX);
uint32_t Motor_verticalRingPD(float Angle, float Gyro);
float Motor_verticalSpeedPI(int encoder_left, int encoder_right, float Angle, float Movement);
float Motor_verticalTurnPD(float taget_yaw, float yaw, float gyro);
void Motor_PWMLimiting(int* motor1, int* motor2);
uint8_t Motor_turnOff(const float Angle);

#endif /* __MOTOR_CONTROL_H */
