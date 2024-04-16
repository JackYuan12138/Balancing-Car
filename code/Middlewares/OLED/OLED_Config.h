/**
*********************************************************************************
* @file    OLED_config.h
* @author  Alex
* @version V1.0
* @date    2024-03-30
* @brief   OLED配置文件
*********************************************************************************
*/

#ifndef __OLED_CONFIG_H
#define __OLED_CONFIG_H

/* 定义数据线(SDA)和时钟线(SCL)引脚 */
#define OLED_SDA_GPIO_Port      GPIOB
#define OLED_SDA_Pin            GPIO_PIN_9

#define OLED_SCL_GPIO_Port      GPIOB
#define OLED_SCL_Pin            GPIO_PIN_8

#endif /* __OLED_CONFIG_H */
