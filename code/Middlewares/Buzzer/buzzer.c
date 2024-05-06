#include "buzzer.h"

/**
 * @brief 蜂鸣器初始化
 */
void Buzzer_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/*开启时钟*/
	if (BUZZER_GPIO_PIN == GPIOA)
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
	}else if (BUZZER_GPIO_PIN==GPIOB)
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}

	GPIO_InitStruct.Pin = BUZZER_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief 蜂鸣器响
 */
void Buzzer_on(void)
{
	HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_RESET);
}

/**
 * @brief 蜂鸣器停
 */
void Buzzer_off(void)
{
	HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_SET);
}
