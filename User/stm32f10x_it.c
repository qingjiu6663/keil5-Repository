/**
  * @file    stm32f10x_it.c
  * @brief   STM32F10x 中断服务函数
  * @note    本文件包含所有异常处理函数和外设中断处理函数
  *          用户实际用到的中断：EXTI0（PA0 按键）
  *          其余默认为空（死循环以防止未处理的错误异常）
  */
#include "stm32f10x_it.h"

extern volatile uint8_t PA0_Pressed;

/******************************************************************************/
/*            Cortex-M3 处理器异常处理                                        */
/******************************************************************************/

void NMI_Handler(void)          {}  /* 非屏蔽中断（空）*/
void HardFault_Handler(void)    { while (1); }  /* 硬件错误：死循环 */
void MemManage_Handler(void)    { while (1); }  /* 内存管理错误：死循环 */
void BusFault_Handler(void)     { while (1); }  /* 总线错误：死循环 */
void UsageFault_Handler(void)   { while (1); }  /* 用法错误：死循环 */
void SVC_Handler(void)          {}  /* SVCall（空）*/
void DebugMon_Handler(void)     {}  /* 调试监视器（空）*/
void PendSV_Handler(void)       {}  /* 可挂起系统调用（空）*/
void SysTick_Handler(void)      {}  /* SysTick 定时器（空，延时使用查询方式）*/

/******************************************************************************/
/*                 STM32F10x 外设中断处理                                     */
/******************************************************************************/

/*
 * EXTI0 中断：PA0 按键下降沿触发
 * 置位 PA0_Pressed 标志，供 main.c 中 Task 5（雷达扫描）使用
 * 中断标志在函数内自动清除
 */
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        PA0_Pressed = 1;                    /* 标记按键按下 */
        EXTI_ClearITPendingBit(EXTI_Line0); /* 清除中断挂起位 */
    }
}
