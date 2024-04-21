#include "bluetooth.h"
#include "stdlib.h"
User_USART BT_Data;

/**
 * @brief 蓝牙初始化
 * @param Data: 蓝牙结构体
 */
void Bluetooth_USART_Init()
{
	USART_init();
	for (uint8_t i = 0; i < USART_RXBUFFER_LEN; i++) {
		BT_Data.USART_Base_Struct.RxBuffer[i] = 0;
	}
	BT_Data.USART_Base_Struct.frame_head = 0xA5;
	BT_Data.USART_Base_Struct.frame_tail = 0x5A;
	BT_Data.USART_Base_Struct.Rx_flag = 0;
	BT_Data.USART_Base_Struct.Rx_len = 0;
	BT_Data.x = 0;
	BT_Data.y = 0;
	BT_Data.mode = 0;
	BT_Data.rxflag = 0;
}

/**
 * @brief USART3中断处理函数
 */
void USART3_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart, UART_FLAG_IDLE) != RESET) {
		/*空闲中断*/
		uint32_t temp_flag = 0;
		uint32_t temp;
		temp_flag = __HAL_UART_GET_FLAG(&huart, UART_FLAG_IDLE);
		if ((temp_flag != RESET)) {
			__HAL_UART_CLEAR_IDLEFLAG(&huart);
			temp = huart.Instance->SR;
			temp = huart.Instance->DR;
			HAL_UART_DMAStop(&huart);
			temp = hdma_usart3_rx.Instance->CNDTR;
			BT_Data.USART_Base_Struct.Rx_len = USART_RXBUFFER_LEN - temp;
			BTData_Process();
			BT_Data.USART_Base_Struct.Rx_flag = 1;
			HAL_UART_Receive_DMA(&huart, BT_Data.USART_Base_Struct.RxBuffer, USART_RXBUFFER_LEN);//重新开启dma
		}
	}
	HAL_UART_IRQHandler(&huart);
}

/**
 * @brief 蓝牙数据处理
 */
void BTData_Process()
{
	/*检查帧头帧尾*/
	if (BT_Data.USART_Base_Struct.RxBuffer[0] != BT_Data.USART_Base_Struct.frame_head) return;
	if (BT_Data.USART_Base_Struct.RxBuffer[6] != BT_Data.USART_Base_Struct.frame_tail) return;

	BT_Data.x = BT_Data.USART_Base_Struct.RxBuffer[1];
	BT_Data.y = BT_Data.USART_Base_Struct.RxBuffer[2];
	BT_Data.mode = BT_Data.USART_Base_Struct.RxBuffer[3];
	BT_Data.rxflag = BT_Data.USART_Base_Struct.RxBuffer[4];

	if (BT_Data.x > 127) BT_Data.x -= 255;
	if (abs(BT_Data.x) < 10) BT_Data.x = 0;//消除转向抖动
	if (BT_Data.y > 127) BT_Data.y -= 255;
	if (abs(BT_Data.y) < 10) BT_Data.y = 0;//消除前进抖动
}
