/**
*********************************************************************************
* @file    bluetooth.h
* @author  Alex
* @version V1.0
* @date    2024-04-17
* @brief   USART应用头文件
*********************************************************************************
*/

#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "usart.h"

/**
 * @brief 用户USART串口结构体
 */
typedef struct User_USART
{
	USART_Strust USART_Base_Struct;
	int x, y, mode, rxflag;
}User_USART;

extern User_USART BT_Data;//蓝牙数据结构体

void Bluetooth_USART_Init(void);
void USART3_IRQHandler(void);
void BTData_Process(void);

#endif /* __BLUETOOTH_H */
