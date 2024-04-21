/**
*********************************************************************************
* @file    usart.h
* @author  Alex
* @version V1.0
* @date    2024-04-17
* @brief   USART通信头文件
*********************************************************************************
*/

#ifndef __USART_H
#define __USART_H

#include "main.h"
#include "usart_config.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_i2s.h"

/**
 * @brief USART串口结构体
 */
typedef struct USART_Strust_t
{
	uint8_t Rx_flag;//接收完成标志
	uint8_t Rx_len;//接收长度
	uint8_t frame_head;//帧头
	uint8_t frame_tail;//帧尾
	uint8_t RxBuffer[USART_RXBUFFER_LEN];//数据存储
}USART_Strust;

extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart;

void USART_init(void);
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle);
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle);

#endif /* __USART_H */
