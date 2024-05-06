#include "my_usart.h"
#include <stdlib.h>
#include <stdio.h>

User_USART BT_Data;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/**
 * @brief USART3初始化
 */
void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 9600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler("HC05 Error");
    }
}

/**
 * @brief DMA初始化
 */
void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
 * @brief USART MSP初始化
 * @param uartHandle: 串口句柄
 */
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    if (uartHandle->Instance == USART3) {
        __HAL_RCC_USART3_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USART3 DMA Init */
        /* USART3_RX Init */
        hdma_usart3_rx.Instance = DMA1_Channel3;
        hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_rx.Init.Mode = DMA_NORMAL;
        hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) {
            Error_Handler("HC05 Error");
        }

        __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart3_rx);

        /* USART3_TX Init */
        hdma_usart3_tx.Instance = DMA1_Channel2;
        hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_tx.Init.Mode = DMA_NORMAL;
        hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) {
            Error_Handler("HC05 Error");
        }

        __HAL_LINKDMA(uartHandle, hdmatx, hdma_usart3_tx);

        HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

    if (uartHandle->Instance == USART3) {
        __HAL_RCC_USART3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);

        HAL_DMA_DeInit(uartHandle->hdmarx);
        HAL_DMA_DeInit(uartHandle->hdmatx);

        HAL_NVIC_DisableIRQ(USART3_IRQn);
    }
}

/**
 * @brief USART初始化
 * @param Data: 串口结构体
 */
void User_USART_Init(User_USART* Data)
{
    for (uint8_t i = 0; i < RXBUFFER_LEN; i++)	Data->RxBuffer[i] = 0;
    Data->frame_head = 0xA5;
    Data->frame_tail = 0x5A;
    Data->Rx_flag = 0;
    Data->Rx_len = 0;
    Data->x = 0;
    Data->y = 0;
    Data->mode = 0;
    Data->rxflag = 0;
}

/**
 * @brief HC05蓝牙通信模块初始化
 */
void HC05_Init(void)
{
    MX_DMA_Init();
    MX_USART3_UART_Init();
    User_USART_Init(&BT_Data);
    HAL_UART_Receive_DMA(&huart3, BT_Data.RxBuffer, RXBUFFER_LEN);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}

/**
 * @brief 串口数据处理
 * @param RxBuffer: 串口接收数据
 */
void BTData_Process(uint8_t* RxBuffer)
{
    /*数据检验*/
    if (RxBuffer[0] != BT_Data.frame_head) return;
    if (RxBuffer[6] != BT_Data.frame_tail) return;

    BT_Data.x = RxBuffer[1];
    BT_Data.y = RxBuffer[2];
    BT_Data.mode = RxBuffer[3];
    BT_Data.rxflag = RxBuffer[4];

    if (BT_Data.x > 127) BT_Data.x -= 255;
    if (abs(BT_Data.x) < 10) BT_Data.x = 0;
    if (BT_Data.y > 127) BT_Data.y -= 255;
    if (abs(BT_Data.y) < 10) BT_Data.y = 0;
}

/**
 * @brief USART3中断处理函数
 */
void USART3_IRQHandler(void)
{
    /* USER CODE BEGIN USART3_IRQn 0 */
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET) {
        uint32_t temp_flag = 0;
        uint32_t temp;
        temp_flag = __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE);
        if ((temp_flag != RESET)) {
            __HAL_UART_CLEAR_IDLEFLAG(&huart3);
            temp = huart3.Instance->SR;
            temp = huart3.Instance->DR;
            HAL_UART_DMAStop(&huart3);
            temp = hdma_usart3_rx.Instance->CNDTR;
            BT_Data.Rx_len = RXBUFFER_LEN - temp;
            BTData_Process(BT_Data.RxBuffer);
            BT_Data.Rx_flag = 1;
            HAL_UART_Receive_DMA(&huart3, BT_Data.RxBuffer, RXBUFFER_LEN);
        }
    }
    /* USER CODE END USART3_IRQn 0 */
    HAL_UART_IRQHandler(&huart3);
    /* USER CODE BEGIN USART3_IRQn 1 */

    /* USER CODE END USART3_IRQn 1 */
}

/**
 * @brief 重置fpuc函数
 */
int fputc(int ch, FILE* fp)
{
    while (!(USART3->SR & (1 << 7)));

    USART3->DR = ch;
    return ch;
}
