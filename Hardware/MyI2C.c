/*
 * 软件 I2C 驱动（GPIO 模拟）
 * SCL: PB10, SDA: PB11
 * 开漏输出，每步 10us 延时适配时序
 * 模拟标准 I2C 协议：起始、停止、发送/接收字节、应答
 */
#include "stm32f10x.h"
#include "Delay.h"

/* ---- 引脚操作层 ---- */
void MyI2C_W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);
    Delay_us(10);
}

void MyI2C_W_SDA(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)BitValue);
    Delay_us(10);
}

uint8_t MyI2C_R_SDA(void)
{
    uint8_t BitValue;
    BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
    Delay_us(10);
    return BitValue;
}

/* 初始化：PB10(SCL)、PB11(SDA) 开漏输出，默认高电平（释放总线）*/
void MyI2C_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;       /* 开漏输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);        /* 释放总线 */
}

/* ---- I2C 协议层 ---- */

/* 起始信号：SCL 高电平时 SDA 从高到低 */
void MyI2C_Start(void)
{
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(0);
}

/* 停止信号：SCL 高电平时 SDA 从低到高 */
void MyI2C_Stop(void)
{
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(1);
}

/* 发送一个字节：高位先行，每 bit 在 SCL 高电平时被从机采样 */
void MyI2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        MyI2C_W_SDA(!!(Byte & (0x80 >> i)));   /* 取出指定位并输出 */
        MyI2C_W_SCL(1);
        MyI2C_W_SCL(0);
    }
}

/* 接收一个字节：释放 SDA，每 bit 在 SCL 高电平时采样 */
uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i, Byte = 0x00;
    MyI2C_W_SDA(1);                             /* 释放 SDA，让从机驱动 */
    for (i = 0; i < 8; i++)
    {
        MyI2C_W_SCL(1);
        if (MyI2C_R_SDA()) Byte |= (0x80 >> i); /* SDA 为高则置对应位 */
        MyI2C_W_SCL(0);
    }
    return Byte;
}

/* 发送应答位：0=应答(ACK)，1=非应答(NACK) */
void MyI2C_SendAck(uint8_t AckBit)
{
    MyI2C_W_SDA(AckBit);
    MyI2C_W_SCL(1);
    MyI2C_W_SCL(0);
}

/* 接收应答位：返回 0=应答(ACK)，1=非应答(NACK) */
uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t AckBit;
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    AckBit = MyI2C_R_SDA();
    MyI2C_W_SCL(0);
    return AckBit;
}
