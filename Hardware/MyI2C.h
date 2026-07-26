#ifndef __MYI2C_H
#define __MYI2C_H

#include "stm32f10x.h"

/* 软件 I2C 驱动（GPIO 模拟，PB10=SCL, PB11=SDA，开漏输出）*/
void MyI2C_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(void);

#endif
