/*
 * OLED 0.96寸 128x64 显示屏驱动（SSD1306，I2C 接口）
 * SCL: PB8, SDA: PB9，开漏输出
 * 支持 4 行 x 16 列字符显示（8x16 字库）
 * 支持浮点数、整数、十六进制、二进制、有符号数显示
 */
#include "stm32f10x.h"
#include "OLED_Font.h"

/* 引脚宏定义：SCL-PB8, SDA-PB9 */
#define OLED_W_SCL(x)   GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)(x))
#define OLED_W_SDA(x)   GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(x))

/* I2C 引脚初始化：开漏输出，默认高电平 */
void OLED_I2C_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    OLED_W_SCL(1);
    OLED_W_SDA(1);
}

/* I2C 起始：SCL 高电平时 SDA 从高到低 */
void OLED_I2C_Start(void)
{
    OLED_W_SDA(1);
    OLED_W_SCL(1);
    OLED_W_SDA(0);
    OLED_W_SCL(0);
}

/* I2C 停止：SCL 高电平时 SDA 从低到高 */
void OLED_I2C_Stop(void)
{
    OLED_W_SDA(0);
    OLED_W_SCL(1);
    OLED_W_SDA(1);
}

/* I2C 发送一个字节：高位先行，不处理应答 */
void OLED_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        OLED_W_SDA(!!(Byte & (0x80 >> i)));
        OLED_W_SCL(1);
        OLED_W_SCL(0);
    }
    OLED_W_SCL(1);  /* 额外时钟，跳过应答 */
    OLED_W_SCL(0);
}

/* 写命令：从机地址 0x78 + 命令前缀 0x00 */
void OLED_WriteCommand(uint8_t Command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);
    OLED_I2C_SendByte(0x00);       /* Co=1, D/C#=0 写命令 */
    OLED_I2C_SendByte(Command);
    OLED_I2C_Stop();
}

/* 写数据：从机地址 0x78 + 数据前缀 0x40 */
void OLED_WriteData(uint8_t Data)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);
    OLED_I2C_SendByte(0x40);       /* Co=1, D/C#=1 写数据 */
    OLED_I2C_SendByte(Data);
    OLED_I2C_Stop();
}

/* 设置光标位置：Y=0~7（页），X=0~127（列）*/
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);                /* 设置页地址（Y） */
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4)); /* 设置列地址高4位 */
    OLED_WriteCommand(0x00 | (X & 0x0F));        /* 设置列地址低4位 */
}

/* 全屏清空：8 页 x 128 列，全部写 0x00 */
void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for (i = 0; i < 128; i++)
            OLED_WriteData(0x00);
    }
}

/* 显示一个字符（8x16 字库，占用 2 页）*/
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);      /* 上半部分 */
    for (i = 0; i < 8; i++)
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);  /* 下半部分 */
    for (i = 0; i < 8; i++)
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
}

/* 显示字符串 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
        OLED_ShowChar(Line, Column + i, String[i]);
}

/* 次方运算（用于进制转换显示）*/
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

/* 显示无符号十进制整数 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
}

/* 显示有符号十进制整数 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }
    for (i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
}

/* 显示十六进制数 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        else
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
    }
}

/* 显示二进制数 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
}

/* 显示浮点数：显示负号 + 整数部分 + 小数点 + 小数部分（四舍五入）*/
void OLED_ShowFloat(uint8_t Line, uint8_t Column, float Number, uint8_t IntLen, uint8_t DecLen)
{
    uint8_t i;
    uint32_t IntPart, DecPart, mul;
    uint8_t Negative = 0;

    if (Number < 0) { Negative = 1; Number = -Number; }

    IntPart = (uint32_t)Number;

    mul = 1;
    for (i = 0; i < DecLen; i++) mul *= 10;
    DecPart = (uint32_t)((Number - IntPart) * mul + 0.5f);  /* 四舍五入 */

    if (Negative) { OLED_ShowChar(Line, Column, '-'); Column++; }
    OLED_ShowNum(Line, Column, IntPart, IntLen);
    OLED_ShowChar(Line, Column + IntLen, '.');
    OLED_ShowNum(Line, Column + IntLen + 1, DecPart, DecLen);
}

/*
 * OLED 初始化（SSD1306 初始化序列）
 * 上电延时 -> I2C 初始化 -> 关闭显示 -> 参数配置 -> 开启显示 -> 清屏
 */
void OLED_Init(void)
{
    uint32_t i, j;
    for (i = 0; i < 1000; i++)
        for (j = 0; j < 1000; j++);    /* 上电延时 */

    OLED_I2C_Init();

    OLED_WriteCommand(0xAE);    /* 关闭显示 */
    OLED_WriteCommand(0xD5);    /* 设置显示时钟分频/振荡器频率 */
    OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8);    /* 设置多路复用率 */
    OLED_WriteCommand(0x3F);    /* 1/64 duty */
    OLED_WriteCommand(0xD3);    /* 设置显示偏移 */
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);    /* 设置显示起始行 */
    OLED_WriteCommand(0xA1);    /* 段重映射（左右方向正常）*/
    OLED_WriteCommand(0xC8);    /* COM 扫描方向（上下方向正常）*/
    OLED_WriteCommand(0xDA);    /* 设置 COM 引脚硬件配置 */
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81);    /* 设置对比度 */
    OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9);    /* 设置预充电周期 */
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB);    /* 设置 VCOMH 取消选择级别 */
    OLED_WriteCommand(0x30);
    OLED_WriteCommand(0xA4);    /* 全局显示开启（按 RAM 内容显示）*/
    OLED_WriteCommand(0xA6);    /* 正常显示（非反色）*/
    OLED_WriteCommand(0x8D);    /* 开启充电泵 */
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xAF);    /* 开启显示 */

    OLED_Clear();
}
