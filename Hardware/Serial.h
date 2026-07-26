#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"

void Serial_Init(void);                     /* 串口1初始化 (PA9-TX, PA10-RX, 921600) */
void Serial_SendByte(uint8_t Byte);         /* 发送一个字节 */
void Serial_SendString(char *String);       /* 发送字符串 */
void Serial_Printf(char *format, ...);      /* 格式化打印（需开启 MicroLIB）*/

extern volatile uint8_t Serial_RxData;      /* 最近接收的字节 */
extern volatile uint8_t Serial_RxFlag;      /* 接收完成标志 */
void Serial_ClearRxFlag(void);              /* 清零接收标志 */

#define SERIAL_YAW_DEFAULT  90.0f           /* 默认目标偏航角 */

extern volatile float   Serial_TargetYaw;   /* A+angle 命令解析出的目标角度 */
extern volatile uint8_t Serial_YawUpdated;  /* 角度更新标志，主循环消费后清零 */

#endif
