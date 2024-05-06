/**
*********************************************************************************
* @file    buzzer.h
* @author  Alex
* @version V1.0
* @date    2024-05-06
* @brief   蜂鸣器驱动
*********************************************************************************
*/

#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"
#include "buzzer_config.h"

void Buzzer_init(void);
void Buzzer_on(void);
void Buzzer_off(void);

#endif /* __BUZZER_H */
