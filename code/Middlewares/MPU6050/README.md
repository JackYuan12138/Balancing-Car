---
author: Alex
date: 2024-04-06
---
## 使用之前

1. 在`IIC/IIC_config.h`中配置有关引脚
2. 引入`mpu6050.h`
3. 使用结构体`MPU6050_Data_Struct`存储MPU数据
4. `while (MPU_Init()) {}`和`MPU_DMP_Init()`初始化MPU

## 用法

用`MPU_GetData()`获取MPU数据