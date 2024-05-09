#include "stm32f1xx_hal.h"
#include "stm32f103xb.h"
#include "stdint.h"
#include "OLED.h"
#include "MPU6050.h"
#include "motor_control.h"
#define SPI I2S SUPPORT
void Error_Handler(char *ErroMessage);
