#ifndef _MY_USART_H
#define _MY_USART_H

#include "main.h"
#include "stdio.h"

#define RXBUFFER_LEN 20

typedef struct User_USART
{
	uint8_t Rx_flag;//接收完成标志
	uint8_t Rx_len;//接收数据长度
	uint8_t frame_head;//帧头
	uint8_t frame_tail;	//帧尾
	int x, y, mode, rxflag;
	uint8_t RxBuffer[RXBUFFER_LEN];//数据缓存
}User_USART;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern User_USART BT_Data;
void HC05_Init(void);

#endif
