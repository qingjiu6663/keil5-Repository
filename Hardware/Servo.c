/*
 * 舵机驱动（软件模拟 PWM）
 * 频率 50Hz（周期 20ms），脉宽 0.5ms~2.5ms 对应 0~180 度
 * PA6 由 TIM3 中断软件控制（非硬件 PWM）
 * 原理：Update 中断拉高 PA6 并设比较值，CC1 中断拉低 PA6
 */
#include "stm32f10x.h"
#include "Servo.h"

static uint16_t Servo_Pulse = 1500;     /* 当前脉宽 500~2500 */

void Servo_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   /* PA6 推挽输出（软件控制） */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* TIM3 定时：PSC=71 -> 1MHz(1us), ARR=19999 -> 50Hz(20ms) */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 19999;
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* OC1: Timing 模式，不输出到引脚，仅产生比较中断 */
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 1500;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    /* 使能更新中断（周期开始）和 CC1 中断（脉宽结束） */
    TIM_ITConfig(TIM3, TIM_IT_Update | TIM_IT_CC1, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM3, ENABLE);
}

/* 设置舵机角度：0度->500us, 90度->1500us, 180度->2500us */
void Servo_SetAngle(float Angle)
{
    if (Angle < 0.0f) Angle = 0.0f;
    if (Angle > 180.0f) Angle = 180.0f;
    Servo_Pulse = (uint16_t)(500.0f + Angle * 2000.0f / 180.0f);
}

/*
 * TIM3 中断处理
 * Update: 周期开始 -> PA6 拉高，设置比较值（决定脉宽）
 * CC1: 脉宽结束 -> PA6 拉低
 */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        GPIO_SetBits(GPIOA, GPIO_Pin_6);            /* 周期开始，拉高 PA6 */
        TIM_SetCompare1(TIM3, Servo_Pulse);          /* 设置比较值 */
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        GPIO_ResetBits(GPIOA, GPIO_Pin_6);           /* 脉宽结束，拉低 PA6 */
    }
}
