#ifndef __RADAR_H
#define __RADAR_H

#include "stm32f10x.h"

/* 雷达扫描缓冲区：索引=角度(0~180)，值=距离(cm)，-1表示无数据 */
extern float scan_data[181];

/* 扫描进行中标志（外部可读，防止重复触发）*/
extern volatile uint8_t scan_ongoing;

void Radar_Init(void);              /* 初始化雷达模块（清空数据缓冲区）*/
void Radar_StartScan(void);         /* 执行一次雷达扫描：舵机从 0 到 180 度步进 2 度 */

#endif
