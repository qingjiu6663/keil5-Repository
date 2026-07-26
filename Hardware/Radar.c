/*
 * 超声波雷达扫描模块
 * 舵机从 0 度扫描到 180 度（步进 2 度），每步测距一次
 * 扫描完成后在 OLED 上绘制扇形雷达俯视图
 * 原点在屏幕底部中央 (x=64, y=56)，带 3 条半圆弧标尺（15cm/30cm/60cm）
 */
#include "Radar.h"
#include "Servo.h"
#include "Ultrasonic.h"
#include "OLED.h"
#include "Serial.h"
#include "Delay.h"
#include <math.h>

float scan_data[181];                       /* 雷达扫描数据缓冲区 */
volatile uint8_t scan_ongoing = 0;          /* 扫描进行中标志 */

void Radar_Init(void)
{
    uint8_t i;
    for (i = 0; i <= 180; i++)
        scan_data[i] = -1.0f;               /* 初始化为无效值 */
    scan_ongoing = 0;
}

/*
 * 绘制雷达俯视图
 * 1. 清空帧缓冲
 * 2. 画 3 条半圆弧（15/30/60cm 标尺）
 * 3. 将有效数据点映射到像素坐标
 * 4. 全屏输出到 OLED（纯图形，无文字）
 */
static void Radar_Draw(uint8_t valid_pts)
{
    uint8_t page, col;
    static uint8_t fb[8][128];              /* 帧缓冲：8 页 x 128 列 */
    uint8_t radii[3] = {9, 18, 36};         /* 标尺半径（像素）*/
    float scale = 0.6f;                     /* 距离-像素比例 */
    uint8_t center_x = 64;
    uint8_t center_y = 56;
    int16_t ri, x, y, dx, dy_sq, dy;
    uint8_t angle;

    /* 清空帧缓冲 */
    for (page = 0; page < 8; page++)
        for (col = 0; col < 128; col++)
            fb[page][col] = 0;

    /* 画半圆弧标尺 */
    for (ri = 0; ri < 3; ri++)
    {
        int16_t R = (int16_t)radii[ri];
        for (x = center_x - R; x <= center_x + R; x++)
        {
            if (x < 0 || x > 127) continue;
            dx = x - (int16_t)center_x;
            dy_sq = R * R - dx * dx;
            if (dy_sq > 0)
            {
                dy = (int16_t)sqrtf((float)dy_sq);
                y = (int16_t)center_y - dy;
                if (y >= 0 && y < 64)
                    fb[y / 8][x] |= (uint8_t)(1 << (y % 8));
            }
        }
    }

    /* 绘制数据点（每 2 度一个点）*/
    for (angle = 0; angle <= 180; angle += 2)
    {
        if (scan_data[angle] < 0.0f || scan_data[angle] >= 990.0f)
            continue;

        float r_px = scan_data[angle] * scale;
        if (r_px > 60.0f) r_px = 60.0f;

        float rad = (float)angle * 3.14159265f / 180.0f;
        int16_t px = (int16_t)((float)center_x - r_px * cosf(rad));
        int16_t py = (int16_t)((float)center_y - r_px * sinf(rad));

        if (px >= 0 && px < 128 && py >= 0 && py < 64)
            fb[py / 8][px] |= (uint8_t)(1 << (py % 8));
    }

    /* 输出到 OLED */
    OLED_Clear();
    for (page = 0; page < 8; page++)
    {
        OLED_SetCursor(page, 0);
        for (col = 0; col < 128; col++)
            OLED_WriteData(fb[page][col]);
    }
}

/*
 * 执行一次完整的雷达扫描
 * 流程：舵机 0 度 -> 180 度，步进 2 度，每步测距并实时显示
 *       扫描完成后绘制雷达扇形俯视图
 *       串口输出 CSV 格式：角度,距离
 */
void Radar_StartScan(void)
{
    uint8_t i;
    uint8_t valid_pts = 0;

    scan_ongoing = 1;

    for (i = 0; i <= 180; i++)
        scan_data[i] = -1.0f;

    OLED_Clear();
    OLED_ShowString(1, 1, "Task:5 ");
    OLED_ShowString(2, 1, "Scan...");

    /* 主扫描循环 */
    for (i = 0; i <= 180; i += 2)
    {
        Servo_SetAngle((float)i);
        Delay_ms(80);

        float dist = Ultrasonic_GetDistance();
        scan_data[i] = dist;
        if (dist < 990.0f) valid_pts++;

        /* OLED 实时显示 */
        OLED_ShowString(2, 1, "Ang:   ");
        OLED_ShowNum(2, 5, i, 3);
        OLED_ShowString(3, 1, "Dist:   ");
        if (dist < 990.0f)
        {
            OLED_ShowNum(3, 6, (uint32_t)dist, 2);
            OLED_ShowString(3, 8, "cm");
        }
        else
        {
            OLED_ShowString(3, 6, "---");
        }

        Serial_Printf("%d,%.1f\r\n", i, dist);  /* 串口输出 */
    }

    Serial_Printf("Sweep Done!\r\n");
    Radar_Draw(valid_pts);                      /* 绘制雷达图 */
    scan_ongoing = 0;
}
