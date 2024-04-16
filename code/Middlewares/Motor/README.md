---
time: 2024-04-16
author: Alex
---

## 应用层文件结构

| 文件 | 功能 |
| --- | --- |
| motor_control.h | 头文件 |
| motor_Config.h | 配置文件 |

## 使用方式
1. 在`motor_config.h`配置好引脚
2. 引入`motor_control.h`

## 常用函数

| 函数 | 功能 |
| --- | --- |
| Motor_init() | 电机初始化 |
| Motor_setSpeed() | 设置电机速度(-1000~1000) |
| Motor_getSpeed() | 获得电机速度 |
| Motor_turnOff() | 关闭电机 |
| Motor_verticalRingPD() | 直立环PD控制 |
| Motor_verticalSpeedPI() | 垂直速度PI控制 |
| Motor_verticalTurnPD() | 垂直转向PD控制 |
| Motor_PWMLimiting() | PWM限幅 |
