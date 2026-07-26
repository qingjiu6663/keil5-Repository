#include "stm32f10x.h"

/* LED 初始化：LED1-PB14, LED2-PB15, LED3-PA8, LED4-PA9，推挽输出，初始熄灭 */
void LED_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;   /* PA8(LED3) PA9(LED4) */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15; /* PB14(LED1) PB15(LED2) */
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);
    GPIO_ResetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_9);
}

void LED1_ON(void)  { GPIO_SetBits(GPIOB, GPIO_Pin_14); }
void LED1_OFF(void) { GPIO_ResetBits(GPIOB, GPIO_Pin_14); }
void LED1_Turn(void)
{
    if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_14) == 0)
        GPIO_SetBits(GPIOB, GPIO_Pin_14);
    else
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);
}

void LED2_ON(void)  { GPIO_SetBits(GPIOB, GPIO_Pin_15); }
void LED2_OFF(void) { GPIO_ResetBits(GPIOB, GPIO_Pin_15); }
void LED2_Turn(void)
{
    if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_15) == 0)
        GPIO_SetBits(GPIOB, GPIO_Pin_15);
    else
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
}

void LED3_ON(void)  { GPIO_SetBits(GPIOA, GPIO_Pin_8); }
void LED3_OFF(void) { GPIO_ResetBits(GPIOA, GPIO_Pin_8); }
void LED3_Turn(void)
{
    if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8) == 0)
        GPIO_SetBits(GPIOA, GPIO_Pin_8);
    else
        GPIO_ResetBits(GPIOA, GPIO_Pin_8);
}

void LED4_ON(void)  { GPIO_SetBits(GPIOA, GPIO_Pin_9); }
void LED4_OFF(void) { GPIO_ResetBits(GPIOA, GPIO_Pin_9); }
void LED4_Turn(void)
{
    if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_9) == 0)
        GPIO_SetBits(GPIOA, GPIO_Pin_9);
    else
        GPIO_ResetBits(GPIOA, GPIO_Pin_9);
}
