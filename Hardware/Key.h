#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

void Key_Init(void);            /* 按键 GPIO 初始化 (PA0 PA1 上拉输入) */
uint8_t Key_GetNum(void);       /* 阻塞式获取按键号，返回 1/2，无按键返回 0 */
void Key_PA0_Init(void);        /* PA0 外部中断初始化（下降沿触发，用于触发雷达扫描） */

#endif
