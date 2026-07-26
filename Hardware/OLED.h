#ifndef __OLED_H
#define __OLED_H

#include "stm32f10x.h"

/* OLED 0.96寸 128x64 I2C 驱动（SSD1306，PB8=SCL, PB9=SDA）*/
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowFloat(uint8_t Line, uint8_t Column, float Number, uint8_t IntLen, uint8_t DecLen);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

void OLED_SetCursor(uint8_t Y, uint8_t X);     /* 设置光标（页/列） */
void OLED_WriteData(uint8_t Data);              /* 写数据到 GDDRAM */

#endif
