/**
*********************************************************************************
* @file    IIC.c
* @author  Alex
* @version V1.0
* @date    2024-04-02
* @brief   I2C通信配置及驱动
*********************************************************************************
*/

#include "stm32f1xx_hal.h"
#include "IIC.h"
#include "IIC_config.h"

#if 1	
#define IIC_SCL_1()  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SCL_PIN, GPIO_PIN_SET)		/* SCL = 1 */
#define IIC_SCL_0()  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SCL_PIN, GPIO_PIN_RESET)		/* SCL = 0 */

#define IIC_SDA_1()  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SDA_PIN, GPIO_PIN_SET)		/* SDA = 1 */
#define IIC_SDA_0()  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SDA_PIN, GPIO_PIN_RESET)		/* SDA = 0 */

#define IIC_SDA_READ()  HAL_GPIO_ReadPin(GPIO_PORT_IIC, IIC_SDA_PIN)
#else	

#define IIC_SCL_1()  GPIO_PORT_IIC->BSRR = IIC_SCL_PIN				/* SCL = 1 */
#define IIC_SCL_0()  GPIO_PORT_IIC->BRR = IIC_SCL_PIN				/* SCL = 0 */

#define IIC_SDA_1()  GPIO_PORT_IIC->BSRR = IIC_SDA_PIN				/* SDA = 1 */
#define IIC_SDA_0()  GPIO_PORT_IIC->BRR = IIC_SDA_PIN				/* SDA = 0 */

#define IIC_SDA_READ()  ((GPIO_PORT_IIC->IDR & IIC_SDA_PIN) != 0)
#endif

void IIC_GPIO_Init(void);

/**
 * @brief IIC延时函数
 */
static void IIC_Delay(void)
{
    uint8_t i;

    for (i = 0; i < 1 ; i++);
}

/**
 * @brief IIC启动
 */
void IIC_Start(void)
{
    IIC_SDA_1();
    IIC_SCL_1();
    IIC_Delay();
    IIC_SDA_0();
    IIC_Delay();
    IIC_SCL_0();
    IIC_Delay();
}

/**
 * @brief IIC停止
 */
void IIC_Stop(void)
{
    IIC_SDA_0();
    IIC_SCL_1();
    IIC_Delay();
    IIC_SDA_1();
}

/**
 * @brief IIC发送一个字节
 * @param _ucByte 要发送的字节
 */
void IIC_Send_Byte(uint8_t _ucByte)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (_ucByte & 0x80)
        {
            IIC_SDA_1();
        }
        else
        {
            IIC_SDA_0();
        }
        IIC_Delay();
        IIC_SCL_1();
        IIC_Delay();
        IIC_SCL_0();
        if (i == 7)
        {
            IIC_SDA_1(); 
        }
        _ucByte <<= 1;
        IIC_Delay();
    }
}

/**
 * @brief IIC读一个字节
 * @param ack 0表示不应答，1表示应答
 * @return 读到的字节
 */
uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t i;
    uint8_t value;

    value = 0;
    for (i = 0; i < 8; i++)
    {
        value <<= 1;
        IIC_SCL_1();
        IIC_Delay();
        if (IIC_SDA_READ())
        {
            value++;
        }
        IIC_SCL_0();
        IIC_Delay();
    }
    if(ack==0)
        IIC_NAck();
    else
        IIC_Ack();
    return value;
}

/**
 * @brief IIC等待应答
 * @return 0表示应答成功，1表示应答失败
 */
uint8_t IIC_Wait_Ack(void)
{
    uint8_t re;

    IIC_SDA_1();
    IIC_Delay();
    IIC_SCL_1();
    IIC_Delay();
    if (IIC_SDA_READ())
    {
        re = 1;
    }
    else
    {
        re = 0;
    }
    IIC_SCL_0();
    IIC_Delay();
    return re;
}

/**
 * @brief IIC应答
 */
void IIC_Ack(void)
{
    IIC_SDA_0();
    IIC_Delay();
    IIC_SCL_1();
    IIC_Delay();
    IIC_SCL_0();
    IIC_Delay();
    IIC_SDA_1();
}

/**
 * @brief IIC不应答
 */
void IIC_NAck(void)
{
    IIC_SDA_1();
    IIC_Delay();
    IIC_SCL_1();
    IIC_Delay();
    IIC_SCL_0();
    IIC_Delay();
}

/**
 * @brief IIC的GPIO初始化
 */
void IIC_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStructure.Pin = IIC_SCL_PIN | IIC_SDA_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(GPIO_PORT_IIC, &GPIO_InitStructure);

    IIC_Stop();
}

/**
 * @brief IIC检测设备
 * @param _Address 设备地址
 * @return 0表示设备存在，1表示设备不存在
 */
uint8_t IIC_CheckDevice(uint8_t _Address)
{
    uint8_t ucAck;

    IIC_GPIO_Init();

    IIC_Start();

    IIC_Send_Byte(_Address|IIC_WR);
    ucAck = IIC_Wait_Ack();

    IIC_Stop();

    return ucAck;
}
