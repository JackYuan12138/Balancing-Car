/**
*********************************************************************************
* @file    motor.h
* @author  Alex
* @version V1.0
* @date    2024-04-09
* @brief   电机驱动头文件
*********************************************************************************
*/

#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "motor_config.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_tim_ex.h"

uint32_t getCCR(TIM_TypeDef* TIMx, unsigned int channel);
void setCCR(TIM_TypeDef* TIMx, unsigned int channel, uint32_t CCR);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim);

#endif /* __MOTOR_H */
