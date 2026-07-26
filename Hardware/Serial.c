#include "stm32f10x.h"
#include "Serial.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

volatile uint8_t Serial_RxData = 0;             /* 串口接收数据 */
volatile uint8_t Serial_RxFlag = 0;             /* 新数据到达标志 */

volatile float   Serial_TargetYaw  = SERIAL_YAW_DEFAULT;  /* A+angle 解析出的目标偏航角 */
volatile uint8_t Serial_YawUpdated = 0;          /* 目标偏航角更新标志 */

/* 串口1初始化：PA9=TX(复用推挽), PA10=RX(浮空输入), 921600-8N1，使能接收中断 */
void Serial_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     /* TX: 复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; /* RX: 浮空输入 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 921600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);      /* 使能接收中断 */

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

/* 串口发送一个字节：查询 TXE 标志，等待发送完毕 */
void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

/* 串口发送字符串（以 \0 结尾）*/
void Serial_SendString(char *String)
{
    uint16_t i;
    for (i = 0; String[i] != '\0'; i++)
        Serial_SendByte(String[i]);
}

/* 串口格式化打印：支持 printf 语法，需开启 MicroLIB */
void Serial_Printf(char *format, ...)
{
    char String[256];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    Serial_SendString(String);
}

/* ---- A+angle 命令解析状态机 ---- */
#define CMD_BUF_SIZE  8
static uint8_t cmd_buf[CMD_BUF_SIZE];   /* 角度数字缓存 */
static uint8_t cmd_idx = 0;             /* 当前缓存位置 */
static uint8_t in_cmd  = 0;             /* 正在解析 A 命令标志 */

extern volatile uint8_t TaskSelect;     /* 任务选择（定义在 main.c 中）*/

/* 解析命令缓存中的角度数值 */
static void ParseCmdBuffer(void)
{
    if (cmd_idx == 0) return;
    int32_t angle = 0;
    uint8_t i;
    for (i = 0; i < cmd_idx; i++)
        angle = angle * 10 + (cmd_buf[i] - '0');
    if (angle >= 0 && angle <= 360)
    {
        Serial_TargetYaw  = (float)angle;
        Serial_YawUpdated = 1;
    }
}

/*
 * USART1 中断处理
 * '1'~'6' 直接切换任务模式
 * "A+角度"（如 A90）设置目标偏航角，支持连续数字实时解析 */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t data = USART_ReceiveData(USART1);

        if (data == 'A' || data == 'a')          /* 开始角度命令 */
        {
            in_cmd  = 1;
            cmd_idx = 0;
            cmd_buf[0] = '\0';
        }
        else if (in_cmd)
        {
            if (data >= '0' && data <= '9')
            {
                if (cmd_idx < CMD_BUF_SIZE - 1)
                {
                    cmd_buf[cmd_idx++] = data;
                    cmd_buf[cmd_idx]   = '\0';
                }
                ParseCmdBuffer();               /* 每收到一位数字立即解析 */
            }
            else
            {
                if (cmd_idx > 0) ParseCmdBuffer();
                in_cmd  = 0;
                cmd_idx = 0;
            }
        }
        else if (data >= '1' && data <= '6')     /* 任务切换命令 1~6 */
        {
            TaskSelect = data - '0';
        }
        else if (data == 'M' || data == 'm')
        {
            extern volatile uint8_t Mode6_ToggleReq;
            Mode6_ToggleReq = 1;
        }

        Serial_RxData = data;
        Serial_RxFlag = 1;
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void Serial_ClearRxFlag(void) { Serial_RxFlag = 0; }
