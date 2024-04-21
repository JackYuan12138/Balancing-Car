#include "usart.h"

DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
UART_HandleTypeDef huart;

void USART_init()
{
	huart.Instance = USARTx;
	huart.Init.BaudRate = USART_BAUDRATE;
	huart.Init.WordLength = UART_WORDLENGTH_8B;
	huart.Init.StopBits = UART_STOPBITS_1;
	huart.Init.Parity = UART_PARITY_NONE;
	huart.Init.Mode = UART_MODE_TX_RX;
	huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart) != HAL_OK) {
		Error_Handler("USART Init Error");
	}
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/*开启时钟*/
	if (uartHandle->Instance == USART1) {
		__HAL_RCC_USART1_CLK_ENABLE();
	} else if (uartHandle->Instance == USART2) {
		__HAL_RCC_USART2_CLK_ENABLE();
	} else if (uartHandle->Instance == USART3) {
		__HAL_RCC_USART3_CLK_ENABLE();
	}

	if (USART_RX_PORT == GPIOA || USART_TX_PORT == GPIOA) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
	} else if (USART_RX_PORT == GPIOB || USART_TX_PORT == GPIOB) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}

	/*配置GPIO*/
	GPIO_InitStruct.Pin = USART_RX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(USART_RX_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = USART_TX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USART_TX_PORT, &GPIO_InitStruct);

	/*RX口配置DMA*/
	hdma_usart3_rx.Instance = DMA1_Channel3;
	hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart3_rx.Init.Mode = DMA_NORMAL;
	hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
	if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) {
		Error_Handler("DMA Init Error");
	}
	__HAL_LINKDMA(uartHandle, hdmarx, hdma_usart3_rx);

	/*TX口配置DMA*/
	hdma_usart3_tx.Instance = DMA1_Channel2;
	hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart3_tx.Init.Mode = DMA_NORMAL;
	hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
	if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) {
		Error_Handler("DMA Init Error");
	}
	__HAL_LINKDMA(uartHandle, hdmatx, hdma_usart3_tx);

	if (USARTx == USART1) {
		HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
	} else if (USARTx == USART2) {
		HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);
	} else if (USARTx == USART3) {
		HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
	}
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
	if (uartHandle->Instance == USART1) {
		__HAL_RCC_USART1_CLK_DISABLE();
	} else if (uartHandle->Instance == USART2) {
		__HAL_RCC_USART2_CLK_DISABLE();
	} else if (uartHandle->Instance == USART3) {
		__HAL_RCC_USART3_CLK_DISABLE();
	}

	HAL_GPIO_DeInit(USART_RX_PORT, USART_RX_PIN);
	HAL_GPIO_DeInit(USART_TX_PORT, USART_TX_PIN);

	HAL_DMA_DeInit(uartHandle->hdmarx);
	HAL_DMA_DeInit(uartHandle->hdmatx);

	if (USARTx == USART1) {
		HAL_NVIC_DisableIRQ(USART1_IRQn);
	} else if (USARTx == USART2) {
		HAL_NVIC_DisableIRQ(USART2_IRQn);
	} else if (USARTx == USART3) {
		HAL_NVIC_DisableIRQ(USART3_IRQn);
	}
}
