/**
*********************************************************************************
* @file    motor_config.h
* @author  Alex
* @version V1.0
* @date    2024-04-09
* @brief   电机驱动配置文件
*********************************************************************************
*/
#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H

#define 	MOTOR_A 		1 //是否使用电机A
#define 	MOTOR_B 		2 //是否使用电机B

#if MOTOR_A

/*电机A端口*/
#define MOTOR_APWM_Port 			GPIOA
#define MOTOR_APWM_Pin 				GPIO_PIN_2
#define MOTOR_AIN1_Port				GPIOA
#define MOTOR_AIN1_Pin 				GPIO_PIN_4
#define MOTOR_AIN2_Port 			GPIOA
#define MOTOR_AIN2_Pin 				GPIO_PIN_5
/*控制电机A的PWM对应时钟资源*/
#define MOTOR_A_PWM_TIM 			TIM2
#define MOTOR_A_PWM_TIM_CCR_CHANNEL TIM_CHANNEL_3
/*电机A编码器端口*/
#define MOTOR_AENCODER_Port 		GPIOA
#define MOTOR_AENCODER_Pin1 		GPIO_PIN_0
#define MOTOR_AENCODER_Pin2 		GPIO_PIN_1
/*控制电机A编码器对应时钟资源*/
#define MOTOR_A_ENCODER_TIM 		TIM1

#endif /* MOTOR_A */

#if MOTOR_B

/*电机B端口*/
#define MOTOR_BPWM_Port 			GPIOB
#define MOTOR_BPWM_Pin 				GPIO_PIN_2
#define MOTOR_BIN1_Port 			GPIOB
#define MOTOR_BIN1_Pin 				GPIO_PIN_4
#define MOTOR_BIN2_Port 			GPIOB
#define MOTOR_BIN2_Pin 				GPIO_PIN_5
/*控制电机B的PWM对应时钟资源*/
#define MOTOR_B_PWM_TIM 			TIM3
#define MOTOR_B_PWM_TIM_CCR_CHANNEL TIM_CHANNEL_4
/*电机B编码器端口*/
#define MOTOR_BENCODER_Port 		GPIOB
#define MOTOR_BENCODER_Pin1 		GPIO_PIN_6
#define MOTOR_BENCODER_Pin2 		GPIO_PIN_7
/*控制电机B编码器对应时钟资源*/
#define MOTOR_B_ENCODER_TIM 		TIM4

#endif /* MOTOR_B */

#endif /* __MOTOR_CONFIG_H */
