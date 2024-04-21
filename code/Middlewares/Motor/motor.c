#include "motor.h"

/**
 * @brief 获取CCR
 * @param TIMx: TIM(1~4)
 * @param channel: 通道(1~4)
 * @return CCR的值
 */
uint32_t getCCR(TIM_TypeDef* TIMx, unsigned int channel)
{
	assert_param(IS_TIM_INSTANCE(TIMx));
	assert_param(IS_TIM_CHANNELS(channel));

	if (channel == TIM_CHANNEL_1) {
		return TIMx->CCR1;
	} else if (channel == TIM_CHANNEL_2) {
		return TIMx->CCR2;
	} else if (channel == TIM_CHANNEL_3) {
		return TIMx->CCR3;
	} else if (channel == TIM_CHANNEL_4) {
		return TIMx->CCR4;
	}

	return 0;
}

/**
 * @brief 设置CCR
 * @param TIMx: TIM(1~4)
 * @param channel: 通道(1~4)
 * @param CCR: CCR的值
 */
void setCCR(TIM_TypeDef* TIMx, unsigned int channel, uint32_t CCR)
{
	assert_param(IS_TIM_INSTANCE(TIMx));
	assert_param(IS_TIM_CHANNELS(channel));

	if (channel == TIM_CHANNEL_1) {
		TIMx->CCR1 = CCR;
	} else if (channel == TIM_CHANNEL_2) {
		TIMx->CCR2 = CCR;
	} else if (channel == TIM_CHANNEL_3) {
		TIMx->CCR3 = CCR;
	} else if (channel == TIM_CHANNEL_4) {
		TIMx->CCR4 = CCR;
	}
}

/**
 * @brief 电机PWM输出反初始化回调函数，用于反初始化GPIO，TIM时钟和NVIC
 * @param tim_pwmHandle TIM句柄
 */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{
	if (tim_pwmHandle->Instance == MOTOR_A_PWM_TIM) {
		/* MOTOR_A_START */
		#if MOTOR_A
		if (MOTOR_A_PWM_TIM == TIM1) {
			__HAL_RCC_TIM1_CLK_DISABLE();
		} else if (MOTOR_A_PWM_TIM == TIM2) {
			__HAL_RCC_TIM2_CLK_DISABLE();
		} else if (MOTOR_A_PWM_TIM == TIM3) {
			__HAL_RCC_TIM3_CLK_DISABLE();
		}
		#endif
		/* MOTOR_A_END */
	} else if (tim_pwmHandle->Instance == MOTOR_B_PWM_TIM) {
		/* MOTOR_B_START */
		#if MOTOR_B
		if (MOTOR_B_PWM_TIM == TIM1) {
			__HAL_RCC_TIM1_CLK_DISABLE();
			HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
		} else if (MOTOR_B_PWM_TIM == TIM2) {
			__HAL_RCC_TIM2_CLK_DISABLE();
			HAL_NVIC_DisableIRQ(TIM2_IRQn);
		} else if (MOTOR_B_PWM_TIM == TIM3) {
			__HAL_RCC_TIM3_CLK_DISABLE();
			HAL_NVIC_DisableIRQ(TIM3_IRQn);
		}
		#endif
	}
}

/**
 * @brief 电机PWM输出初始化回调函数，用于初始化GPIO，TIM时钟和NVIC
 * @param htim TIM句柄
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP; //复用推挽输出
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	/*针对PWM引脚对应A2的复用配置*/
	__HAL_RCC_AFIO_CLK_ENABLE();
	__HAL_AFIO_REMAP_TIM2_PARTIAL_1();
	/* MOTOR_A_START */
	#if MOTOR_A
	/*GPIO配置*/
	if (MOTOR_AIN1_Port == GPIOA || MOTOR_AIN2_Port == GPIOA) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
	} else if (MOTOR_AIN1_Port == GPIOB || MOTOR_AIN2_Port == GPIOB) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}
	GPIO_InitStructure.Pin = MOTOR_AIN1_Pin;
	HAL_GPIO_Init(MOTOR_AIN1_Port, &GPIO_InitStructure);
	GPIO_InitStructure.Pin = MOTOR_AIN2_Pin;
	HAL_GPIO_Init(MOTOR_AIN2_Port, &GPIO_InitStructure);
	/*时钟和NVIC配置*/
	if (htim->Instance == MOTOR_A_PWM_TIM) {
		if (MOTOR_A_PWM_TIM == TIM1) {
			__HAL_RCC_TIM1_CLK_ENABLE();
		} else if (MOTOR_A_PWM_TIM == TIM2) {
			__HAL_RCC_TIM2_CLK_ENABLE();
		} else if (MOTOR_A_PWM_TIM == TIM3) {
			__HAL_RCC_TIM3_CLK_ENABLE();
		}
	}
	#endif
	/* MOTOR_A_END */
	/* MOTOR_B_START */
	#if MOTOR_B
	/*GPIO配置*/
	if (MOTOR_BIN1_Port == GPIOA || MOTOR_BIN2_Port == GPIOA || MOTOR_BPWM_Port == GPIOA) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
	} else if (MOTOR_BIN1_Port == GPIOB || MOTOR_BIN2_Port == GPIOB || MOTOR_BPWM_Port == GPIOB) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}
	GPIO_InitStructure.Pin = MOTOR_BPWM_Pin;
	HAL_GPIO_Init(MOTOR_BPWM_Port, &GPIO_InitStructure);
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Pin = MOTOR_BIN1_Pin;
	HAL_GPIO_Init(MOTOR_BIN1_Port, &GPIO_InitStructure);
	GPIO_InitStructure.Pin = MOTOR_BIN2_Pin;
	HAL_GPIO_Init(MOTOR_BIN2_Port, &GPIO_InitStructure);
	/*时钟和NVIC配置*/
	if (htim->Instance == MOTOR_B_PWM_TIM) {
		if (MOTOR_B_PWM_TIM == TIM1) {
			__HAL_RCC_TIM1_CLK_ENABLE();
		} else if (MOTOR_B_PWM_TIM == TIM2) {
			__HAL_RCC_TIM2_CLK_ENABLE();
		} else if (MOTOR_B_PWM_TIM == TIM3) {
			__HAL_RCC_TIM3_CLK_ENABLE();
		}
	}
	#endif
	/* MOTOR_B_END */
}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* tim_encoderHandle)
{
	if (tim_encoderHandle->Instance == TIM1)
	{
		__HAL_RCC_TIM1_CLK_DISABLE();
	}else if (tim_encoderHandle->Instance == TIM2)
	{
		__HAL_RCC_TIM2_CLK_DISABLE();
	}else if (tim_encoderHandle->Instance == TIM3)
	{
		__HAL_RCC_TIM3_CLK_DISABLE();
	} else if (tim_encoderHandle->Instance == TIM4)
	{
		__HAL_RCC_TIM4_CLK_DISABLE();
	}
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* tim_encoderHandle)
{
	if (tim_encoderHandle->Instance == TIM1)
	{
		__HAL_RCC_TIM1_CLK_ENABLE();
	}else if (tim_encoderHandle->Instance == TIM2)
	{
		__HAL_RCC_TIM2_CLK_ENABLE();
	}else if (tim_encoderHandle->Instance == TIM3)
	{
		__HAL_RCC_TIM3_CLK_ENABLE();
	} else if (tim_encoderHandle->Instance == TIM4)
	{
		__HAL_RCC_TIM4_CLK_ENABLE();
	}
}