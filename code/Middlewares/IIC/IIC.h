/**
*********************************************************************************
* @file    IIC.h
* @author  Alex
* @version V1.0
* @date    2024-04-02
* @brief   I2C通信配置及驱动
*********************************************************************************
*/

#ifndef __IIC_H
#define __IIC_H

#include <inttypes.h>

#define IIC_WR	0
#define IIC_RD	1

void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(uint8_t _ucByte);
uint8_t IIC_Read_Byte(uint8_t ack);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
uint8_t IIC_CheckDevice(uint8_t _Address);
void IIC_GPIO_Init(void);

#endif /* __IIC_H */
