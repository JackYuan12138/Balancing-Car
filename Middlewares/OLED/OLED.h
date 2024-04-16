/**
*********************************************************************************
* @file    OLED.h
* @author  Alex
* @version V1.0
* @date    2024-03-30
* @brief   OLED驱动头文件
*********************************************************************************
*/

#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C"
{
#endif

/* -------------------------------------------------------------------
 * Includes
 * ------------------------------------------------------------------- */
#include "stm32f1xx_hal.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "OLED_Config.h"
#include "OLED_Libraries.h"

/* ------------------------------------------------------------------- */
/* Functions
 * ------------------------------------------------------------------- */
    // 底层函数
    void OLED_I2C_Init(void);
    void OLED_I2C_Start(void);
    void OLED_I2C_Stop(void);
    void OLED_I2C_SendByte(uint8_t Byte);
    void OLED_writeCommand(uint8_t Command);
    void OLED_writeData(uint8_t Data);
    void OLED_setCursor(uint8_t Y, uint8_t X);
    // 应用层函数
    void OLED_Init(void);
    void OLED_clear(void);
    void OLED_showChar(uint8_t Line, uint8_t Column, char Char);
    void OLED_showString(uint8_t Line, uint8_t Column, char* String);
    void OLED_showNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
    void OLED_showSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
    void OLED_showHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
    void OLED_showBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
