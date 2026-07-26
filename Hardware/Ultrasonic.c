/*
 * HC-SR04 超声波测距模块驱动
 * TRIG: PB12（发送 15us 高电平触发）
 * ECHO: PA7（EXTI 中断捕获上升沿和下降沿）
 * 距离(cm) = 脉宽(us) / 58
 * TIM2 用作 1MHz 自由计数器，测量脉冲宽度
 */
#include "stm32f10x.h"
#include "Ultrasonic.h"
#include "Delay.h"

#define TRIG_PORT   GPIOB
#define TRIG_PIN    GPIO_Pin_12
#define ECHO_PORT   GPIOA
#define ECHO_PIN    GPIO_Pin_7

static volatile uint32_t ECHO_RisingTime  = 0;  /* ECHO 上升沿时刻 */
static volatile uint32_t ECHO_FallingTime = 0;  /* ECHO 下降沿时刻 */
static volatile uint8_t  ECHO_Ready       = 0;  /* ECHO 捕获完成标志 */

void Ultrasonic_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    /* TRIG: 推挽输出，初始低电平 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin   = TRIG_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TRIG_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);

    /* ECHO: 浮空输入（EXTI 检测边沿） */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin   = ECHO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ECHO_PORT, &GPIO_InitStructure);

    /* PA7 -> EXTI7，上升沿+下降沿中断 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource7);

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line      = EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode      = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger   = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd   = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* TIM2: 72MHz/72=1MHz 自由计数器，ARR=0xFFFF，溢出周期约 65.5ms */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period        = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler     = 71;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM2, ENABLE);
}

/* EXTI9-5 中断：ECOH 上升沿记录起始时刻，下降沿记录结束时刻并标记完成 */
void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line7) != RESET)
    {
        if (GPIO_ReadInputDataBit(ECHO_PORT, ECHO_PIN) == Bit_SET)
        {
            ECHO_RisingTime = TIM_GetCounter(TIM2);  /* 上升沿：记录起始时刻 */
            ECHO_Ready = 0;
        }
        else
        {
            ECHO_FallingTime = TIM_GetCounter(TIM2); /* 下降沿：记录结束时刻 */
            ECHO_Ready = 1;
        }
        EXTI_ClearITPendingBit(EXTI_Line7);
    }
}

/*
 * 获取超声波测距结果
 * 1. 发送 15us 高电平 TRIG 脉冲
 * 2. 等待 ECHO 下降沿（超时约 30ms，对应约 5m）
 * 3. 计算脉宽 -> 距离(cm) = 脉宽(us) / 58
 * 超时返回 999.0
 */
float Ultrasonic_GetDistance(void)
{
    uint32_t time_us;
    uint32_t timeout;

    GPIO_SetBits(TRIG_PORT, TRIG_PIN);      /* 发送 15us TRIG 脉冲 */
    Delay_us(15);
    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);

    ECHO_Ready = 0;
    timeout = 30000;
    while (!ECHO_Ready && timeout--)        /* 等待 ECHO 下降沿，约 30ms 超时 */
        Delay_us(1);

    if (!ECHO_Ready)
        return 999.0f;                      /* 超时 / 无障碍物 */

    if (ECHO_FallingTime >= ECHO_RisingTime)
        time_us = ECHO_FallingTime - ECHO_RisingTime;
    else
        time_us = (0xFFFF - ECHO_RisingTime) + ECHO_FallingTime + 1;  /* 处理溢出 */

    return (float)time_us / 58.0f;          /* 距离 = 脉宽(us) / 58 (cm) */
}
