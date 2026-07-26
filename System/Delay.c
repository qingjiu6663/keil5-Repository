/*
 * SysTick 延时函数
 * STM32F103 主频 72MHz，SysTick 时钟 = HCLK = 72MHz
 * Delay_us: 72 次计数 = 1us
 * 通过 LOAD/VAL/CTRL 寄存器实现精确阻塞延时
 */
#include "stm32f10x.h"

/* 微秒延时：设置 SysTick 为 72*xus 次计数，等待 COUNTFLAG 置位后关闭 */
void Delay_us(uint32_t xus)
{
    SysTick->LOAD = 72 * xus;               /* 设置重装载值 */
    SysTick->VAL = 0x00;                     /* 清空当前计数值 */
    SysTick->CTRL = 0x00000005;              /* HCLK 时钟源，启动定时器 */
    while (!(SysTick->CTRL & 0x00010000));   /* 等待计数到 0（COUNTFLAG 置位）*/
    SysTick->CTRL = 0x00000004;              /* 关闭定时器 */
}

/* 毫秒延时：循环调用 Delay_us(1000) */
void Delay_ms(uint32_t xms)
{
    while (xms--)
        Delay_us(1000);
}

/* 秒级延时：循环调用 Delay_ms(1000) */
void Delay_s(uint32_t xs)
{
    while (xs--)
        Delay_ms(1000);
}
