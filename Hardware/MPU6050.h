#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

/* 姿态数据结构体：俯仰角、横滚角、偏航角（单位：度） */
typedef struct {
    float Pitch;    /* 俯仰角（度） */
    float Roll;     /* 横滚角（度） */
    float Yaw;      /* 偏航角（度） */
} Attitude_t;

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                        int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_GetAttitude(Attitude_t *att);   /* 获取姿态角（互补滤波） */

#endif
