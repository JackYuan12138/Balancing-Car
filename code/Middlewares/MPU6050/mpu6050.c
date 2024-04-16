#include "stm32f1xx_hal.h"
#include "mpu6050.h"
#include <math.h>

/**
 * @brief 获取MPU的姿态数据
 * @note 包括三个方向角、加速度和陀螺仪数据
 * @param[in,out] MPU_Data 陀螺仪和加速度数据结构体
 * @retval 0 成功
 * @retval 1 读取FIFO失败
 * @retval 2 读取姿态失败
 */
uint8_t MPU_GetData(MPU6050_Data_Struct* MPU_Data)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
    if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))return 1;
    /* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
     * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
    **/
    /*if (sensors & INV_XYZ_GYRO )
    send_packet(PACKET_TYPE_GYRO, gyro);
    if (sensors & INV_XYZ_ACCEL)
    send_packet(PACKET_TYPE_ACCEL, accel); */
    /* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
     * The orientation is set by the scalar passed to dmp_set_orientation during initialization.
    **/
    if (sensors & INV_WXYZ_QUAT) {
        q0 = quat[0] / q30;	// q30格式转换为浮点数
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;

        // 计算得到俯仰角/横滚角/航向角
        MPU_Data->pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
        MPU_Data->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
        MPU_Data->yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;

        // 将加速度和陀螺仪数据写入结构体
        MPU_Data->acc_x = accel[0];
        MPU_Data->acc_y = accel[1];
        MPU_Data->acc_z = accel[2];
        MPU_Data->gyro_x = gyro[0];
        MPU_Data->gyro_y = gyro[1];
        MPU_Data->gyro_z = gyro[2];
    } else return 2;
    return 0;
}

/**
 * @brief 初始化MPU6050
 * @retval 0 成功
 * @retval 1 错误
 */
uint8_t MPU_Init(void)
{
    uint8_t res;

    MPU_IIC_Init();//初始化IIC总线
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80);	//复位MPU6050
    HAL_Delay(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00);	//唤醒MPU6050
    MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
    MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
    MPU_Set_Rate(50);						//设置采样率50Hz
    MPU_Write_Byte(MPU_INT_EN_REG, 0X00);	//关闭所有中断
    MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00);	//I2C主模式关闭
    MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);	//关闭FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80);	//INT引脚低电平有效
    res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if (res == MPU_ADDR)//器件ID正确
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01);	//设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00);	//加速度与陀螺仪都工作
        MPU_Set_Rate(50);						//设置采样率为50Hz
    } else return 1;
    return 0;
}

/**
 * @brief 设置MPU6050陀螺仪传感器满量程范围
 * @param fsr 0 ±250dps;1 ±500dps;2 ±1000dps;3 ±2000dps
 * @retval 0 设置成功
 * @retval 其他 设置失败
 */
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3);
}

/**
 * @brief 设置MPU6050加速度传感器满量程范围
 * @param fsr 0 ±2g;1 ±4g;2 ±8g;3 ±16g
 * @retval 0 设置成功
 * @retval 其他 设置失败
 */
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3);
}

/**
 * @brief 设置MPU6050的数字低通滤波器
 * @param lpf 数字低通滤波频率(Hz)
 * @retval 0 设置成功
 * @retval 其他 设置失败
 */
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data = 0;
    if (lpf >= 188)data = 1;
    else if (lpf >= 98)data = 2;
    else if (lpf >= 42)data = 3;
    else if (lpf >= 20)data = 4;
    else if (lpf >= 10)data = 5;
    else data = 6;
    return MPU_Write_Byte(MPU_CFG_REG, data);
}

/**
 * @brief 设置MPU6050的采样率
 * @param rate 4~1000(Hz)
 * @retval 0 设置成功
 * @retval 其他 设置失败
 */
uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if (rate > 1000)rate = 1000;
    if (rate < 4)rate = 4;
    data = 1000 / rate - 1;
    data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data);
    return MPU_Set_LPF(rate / 2);	//自动设置LPF为采样率的一半
}


/**
 * @brief 得到温度值
 * @retval 温度值(扩大了100倍)
 */
short MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    short raw;
    float temp;
    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((uint16_t)buf[0] << 8) | buf[1];
    temp = 36.53 + ((double)raw) / 340;
    return temp * 100;;
}

/**
 * @brief 获取MPU的陀螺仪值
 * @param[in,out] MPU_Data MPU6050数据结构体
 * @retval 0 成功
 * @retval 其他 失败
 */
uint8_t MPU_Get_Gyroscope(MPU6050_Data_Struct* MPU_Data)
{
    uint8_t buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == 0) {
        MPU_Data->gyro_x = ((uint16_t)buf[0] << 8) | buf[1];
        MPU_Data->gyro_y = ((uint16_t)buf[2] << 8) | buf[3];
        MPU_Data->gyro_z = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;;
}

/**
 * @brief 获取MPU的加速度值
 * @param[in,out] MPU_Data MPU6050数据结构体
 * @retval 0 成功
 * @retval 其他 失败
 */
uint8_t MPU_Get_Accelerometer(MPU6050_Data_Struct* MPU_Data)
{
    uint8_t buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0) {
        MPU_Data->acc_x = ((uint16_t)buf[0] << 8) | buf[1];
        MPU_Data->acc_y = ((uint16_t)buf[2] << 8) | buf[3];
        MPU_Data->acc_z = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
}
