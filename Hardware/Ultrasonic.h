#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f10x.h"

void Ultrasonic_Init(void);             /* HC-SR04 超声波初始化（TRIG-PB12, ECHO-PA7） */
float Ultrasonic_GetDistance(void);     /* 获取距离(cm)，超时返回 999.0 */

#endif
