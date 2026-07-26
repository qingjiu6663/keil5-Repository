/*
 * MPU6050 六轴姿态传感器驱动（I2C 接口）
 * 量程：陀螺仪 ±2000/s，加速度计 ±16g（配置寄存器的值）
 * DLPF 设置为 5Hz（DLPF_CFG=6）
 * 姿态解算使用互补滤波：angle = 0.9*(angle + gyro*dt) + 0.1*accel_angle
 * YAW 轴（Z 轴）仅依靠陀螺仪积分，角速度接近 0 时冻结防止漂移
 */
#include "stm32f10x.h"
#include "MyI2C.h"
#include "MPU6050_Reg.h"
#include "MPU6050.h"
#include <math.h>

#define MPU6050_ADDRESS     0xD0        /* MPU6050 I2C 地址 */
#define PITCH_OFFSET        -20.0f      /* 俯仰角安装偏差补偿 */

/* 姿态角变量（静态，供互补滤波递归使用）*/
static float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
static uint16_t last_tick = 0;          /* TIM2 上次计数值，用于计算 dt */

/* MPU6050 写寄存器 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(Data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

/* MPU6050 读寄存器 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    MyI2C_ReceiveAck();
    Data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();
    return Data;
}

/*
 * MPU6050 初始化
 * DLPF_CFG=6 -> 5Hz 数字低通滤波
 * 陀螺仪量程 ±2000/s（0x18）
 * 加速度计量程 ±16g（0x18）
 * TIM2 用作 1MHz 计时器，计算姿态更新间隔 dt
 */
void MPU6050_Init(void)
{
    MyI2C_Init();
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);        /* 退出睡眠，选择 X 轴陀螺为时钟源 */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);        /* 所有轴正常工作 */
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);        /* 采样率分频 */
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);             /* DLPF_CFG=6 -> 5Hz */
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);        /* 陀螺仪 ±2000/s */
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);       /* 加速度计 ±16g */

    /* TIM2 配置为 1MHz 自由计数器 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM2->PSC = 72 - 1;
    TIM2->ARR = 0xFFFF;
    TIM2->CR1 = TIM_CR1_CEN;
    last_tick = TIM2->CNT;
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/* 读取 MPU6050 原始加速度和陀螺仪数据 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                        int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t DataH, DataL;
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    *AccX = (DataH << 8) | DataL;
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    *AccY = (DataH << 8) | DataL;
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    *AccZ = (DataH << 8) | DataL;
    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    *GyroX = (DataH << 8) | DataL;
    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    *GyroY = (DataH << 8) | DataL;
    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    *GyroZ = (DataH << 8) | DataL;
}

/*
 * 获取姿态角（互补滤波）
 * 1. 陀螺仪积分得到高频姿态
 * 2. 加速度计计算 Pitch/Roll 修正低频漂移（互补系数 0.9/0.1）
 * 3. YAW 仅靠陀螺仪积分，角速度 <1/s 时冻结防漂移
 * 4. 减去 PITCH_OFFSET 补偿安装偏差
 */
void MPU6050_GetAttitude(Attitude_t *att)
{
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    float ax, ay, az, gx, gy, gz;
    float pitch_acc, roll_acc;
    uint16_t now;
    float dt;

    MPU6050_GetData(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    /* 原始值转换为物理量 */
    ax = ax_raw / 2048.0f;     /* 加速度 ±16g -> 2048 LSB/g */
    ay = ay_raw / 2048.0f;
    az = az_raw / 2048.0f;
    gx = gx_raw / 16.4f;       /* 陀螺仪 ±2000/s -> 16.4 LSB/(/s) */
    gy = gy_raw / 16.4f;
    gz = gz_raw / 16.4f;

    /* 加速度计计算 Pitch 和 Roll */
    pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.29578f;
    roll_acc  = atan2f( ay, az) * 57.29578f;

    /* 计算时间间隔 dt */
    now = TIM2->CNT;
    dt = (float)(uint16_t)(now - last_tick) / 1000000.0f;
    last_tick = now;

    /* 互补滤波：陀螺仪积分 + 加速度计修正 */
    pitch = 0.90f * (pitch + gx * dt) + 0.1f * pitch_acc;
    roll  = 0.90f * (roll  + gy * dt) + 0.1f * roll_acc;
    /* YAW 仅靠 Z 轴陀螺积分，角速度接近 0 时冻结防漂移 */
    if (fabsf(gz) >= 1.0f)
        yaw = yaw + gz * dt;

    att->Pitch = pitch - PITCH_OFFSET;
    att->Roll  = roll;
    att->Yaw   = yaw;
}
