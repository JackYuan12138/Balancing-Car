---
time: 2024-03-30
author: Alex
---

## 应用层文件结构

| 文件 | 功能 |
| --- | --- |
| OLED.h | 头文件 |
| OLED_Config.h | 配置文件 |

## 使用方法

### IIC通信配置

在`OLED_Config.h`中配置好数据线(SDA)和时钟线(SCL)的引脚。

### 常用功能

| 函数 | 功能 |
| --- | --- |
| OLED_Init() | OLED初始化 |
| OLED_clear() | 清屏 |
| OLED_showString() | 显示字符串 |
| OLED_showNum() | 显示数字 |
