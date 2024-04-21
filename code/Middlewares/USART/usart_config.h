/**
*********************************************************************************
* @file    usart_config.h
* @author  Alex
* @version V1.0
* @date    2024-0417
* @brief   USART配置文件
*********************************************************************************
*/

#ifndef __USART_CONFIG_H
#define __USART_CONFIG_H

/*串口定义*/
#define USARTx USART3

/*串口配置*/
#define USART_RXBUFFER_LEN 20 //接收缓存长度
#define USART_BAUDRATE 9600 //波特率

/*串口引脚定义*/
// 此处RX指的是外设的RX引脚，TX同理
#define USART_RX_PORT GPIOB
#define USART_RX_PIN GPIO_PIN_10
#define USART_TX_PORT GPIOB
#define USART_TX_PIN GPIO_PIN_11

#endif /* __USART_CONFIG_H */
