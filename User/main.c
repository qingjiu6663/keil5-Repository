#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "Serial.h"
#include "Key.h"
#include "Servo.h"
#include "PID.h"
#include "Ultrasonic.h"
#include <stdio.h>
#include <math.h>
#include "Radar.h"
uint8_t ID;
Attitude_t att;
float Yaw_Zero = 0.0f;
uint8_t YawZeroReady = 0;
PID_t g_ServoPID;               
PID_t Mode3_PID;                
PID_t Mode2_PID;                

volatile uint8_t TaskSelect = 1;

volatile uint8_t PA0_Pressed = 0;
volatile uint8_t Mode6_ToggleReq = 0;

#define MODE2_DEBOUNCE_THRESH  3           
static float    Mode2_PAW          = 60.0f;  
static float    Mode2_TargetYaw   = 60.0f;  
static uint8_t  Mode2_PAWLocked    = 0;      
static uint8_t  Mode2_DebounceCnt  = 0;      

static float    Mode2_ServoAngle   = 60.0f;  
static int8_t   Mode2_Dir          = 1;      
static uint8_t  Mode2_FirstRun     = 1;      

#define DIST_MA_WINDOW  5
static float  dist_ma_buf[DIST_MA_WINDOW] = {0.0f};

static float  dist_ma_sum = 0.0f;           
static uint8_t dist_ma_idx = 0;             

static float    Mode3_ServoAngle  = 90.0f;  

static uint8_t  Mode3_Locked      = 0;      
static uint8_t  Mode3_FirstRun    = 1;      

static uint8_t  Mode4_FirstRun    = 1;      
static uint8_t  Mode4_Locked      = 0;      
static float    Mode4_ServoAngle  = 90.0f;  

static float    Mode1_TargetYaw   = 90.0f;  
static int8_t   Mode1_Direction   = 1;      
static float    Mode1_ServoAngle  = 90.0f;  
static uint8_t  Mode1_FirstRun    = 1;      
static float    Mode1_YawZero    = 0.0f;   

static uint8_t  Mode6_FirstRun    = 1;      
static float    Mode6_YawZero     = 0.0f;   
static uint8_t  Mode6_SlaveControl = 0;      

#define MODE3_LOCK_THRESHOLD   1.0f         
#define MODE3_UNLOCK_THRESHOLD 2.0f         
#define MODE3_PID_OUT_LIMIT  30.0f          

static void OLED_ShowTaskTitles(void)
{
    OLED_Clear();
    if (TaskSelect == 1)
    {
        OLED_ShowString(1, 1, "Task:1 ");
        OLED_ShowString(2, 1, "Srv:");
        OLED_ShowString(3, 1, "T/Y:");
    }
    else if (TaskSelect == 2)
    {
        OLED_ShowString(1, 1, "Task:2 ");
        OLED_ShowString(2, 1, "Dist:");
        OLED_ShowString(3, 1, "Srv:");
        OLED_ShowString(4, 1, "PAW:");
    }
    else if (TaskSelect == 3)
    {
        OLED_ShowString(1, 1, "Task:3 ");
        OLED_ShowString(2, 1, "Yaw:");
        OLED_ShowString(3, 1, "Tgt:");
        OLED_ShowString(4, 1, "Ang:");
    }
    else if (TaskSelect == 4)
    {
        OLED_ShowString(1, 1, "Task:4 ");
        OLED_ShowString(2, 1, "Dist:");
        OLED_ShowString(3, 1, "Yaw:");
        OLED_ShowString(4, 1, "Tgt:");
    }
    else if (TaskSelect == 5)
    {
        OLED_ShowString(1, 1, "Task:5 ");
        OLED_ShowString(2, 1, "READY");
    }
    else if (TaskSelect == 6)
    {
        OLED_ShowString(1, 1, "Task:6 ");
        OLED_ShowString(2, 1, "Yaw:");
        OLED_ShowString(3, 1, "Srv:");
        OLED_ShowString(4, 1, "Comm:Host->Slv");
    }
}

int main(void)
{
    
    OLED_Init();
    MPU6050_Init();
    Serial_Init();
    Servo_Init();
    Key_PA0_Init();
    Ultrasonic_Init();
    Radar_Init();
    
    PID_Init(&g_ServoPID, 0.30f, 0.00f, 0.05f, -60.0f, 60.0f);    
    PID_Init(&Mode3_PID, 0.30f, 0.00f, 0.05f,                      
             -MODE3_PID_OUT_LIMIT, MODE3_PID_OUT_LIMIT);
    PID_Init(&Mode2_PID, 0.30f, 0.01f, 0.05f, -60.0f, 60.0f);    
    
    Delay_ms(500);
    MPU6050_GetAttitude(&att);
    Yaw_Zero = att.Yaw;
    
    OLED_ShowTaskTitles();
    static uint8_t LastTask = 0;            
    while (1)
    {
        
        if (TaskSelect != LastTask)
        {
            if (LastTask == 6)
                USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
            LastTask = TaskSelect;
            if (TaskSelect == 2)
            {
                Mode2_PAW = 60.0f;
                Mode2_PAWLocked = 0;
                Mode2_DebounceCnt = 0;
                Mode2_FirstRun = 1;
            }
            else if (TaskSelect == 1)
                Mode1_FirstRun = 1;
            else if (TaskSelect == 3)
                Mode3_FirstRun = 1;
            else if (TaskSelect == 4)
            {
                Mode4_FirstRun = 1;
                dist_ma_sum = 0.0f;
                dist_ma_idx = 0;
                { uint8_t i; for (i = 0; i < DIST_MA_WINDOW; i++) dist_ma_buf[i] = 0.0f; }
            }
            else if (TaskSelect == 6)
                Mode6_FirstRun = 1;
            else if (TaskSelect == 5)
                scan_ongoing = 0;
            OLED_ShowTaskTitles();
        }
        
        if (TaskSelect == 1)
        {
            
            if (Mode1_FirstRun)
            {
                Mode1_FirstRun = 0;
                Mode1_TargetYaw = 90.0f;
                Mode1_Direction = 1;
                Mode1_ServoAngle = 90.0f;
                Servo_SetAngle(Mode1_ServoAngle);
                MPU6050_GetAttitude(&att);
                Mode1_YawZero = att.Yaw;
                PID_Reset(&g_ServoPID);
                PID_SetSetpoint(&g_ServoPID, Mode1_TargetYaw);
            }
            
            Mode1_TargetYaw += Mode1_Direction * 2.0f;
            if (Mode1_TargetYaw >= 120.0f)
            {
                Mode1_TargetYaw = 120.0f;
                Mode1_Direction = -1;
            }
            else if (Mode1_TargetYaw <= 80.0f)
            {
                Mode1_TargetYaw = 80.0f;
                Mode1_Direction = 1;
            }
            PID_SetSetpoint(&g_ServoPID, Mode1_TargetYaw);
            MPU6050_GetAttitude(&att);
            float yaw_norm = att.Yaw - Mode1_YawZero;
            while (yaw_norm > 180.0f)  yaw_norm -= 360.0f;
            while (yaw_norm < -180.0f) yaw_norm += 360.0f;
            float pid_out = PID_Compute(&g_ServoPID, yaw_norm);
            Mode1_ServoAngle += pid_out;
            if (Mode1_ServoAngle > 180.0f) Mode1_ServoAngle = 180.0f;
            if (Mode1_ServoAngle < 0.0f)   Mode1_ServoAngle = 0.0f;
            Servo_SetAngle(Mode1_ServoAngle);
            OLED_ShowFloat(2, 5, Mode1_ServoAngle, 3, 0);
            OLED_ShowFloat(3, 3, Mode1_TargetYaw, 3, 1);
            OLED_ShowString(3, 9, "Y:");
            OLED_ShowFloat(3, 11, yaw_norm, 3, 1);
            Serial_Printf("%.1f,%.1f\r\n", yaw_norm, Mode1_ServoAngle);
        }
        
        else if (TaskSelect == 3)
        {
            if (Mode3_FirstRun)
            {
                Mode3_FirstRun = 0;
                Mode3_Locked = 0;
                Mode3_ServoAngle = 90.0f;
                PID_SetSetpoint(&Mode3_PID, Serial_TargetYaw);
                PID_Reset(&Mode3_PID);
                Servo_SetAngle(Mode3_ServoAngle);
            }
            
            if (Serial_YawUpdated)
            {
                Serial_YawUpdated = 0;
                PID_SetSetpoint(&Mode3_PID, Serial_TargetYaw);
                Mode3_Locked = 0;
                PID_Reset(&Mode3_PID);
                Serial_Printf("MODE3: New target=%.1f\r\n", Serial_TargetYaw);
            }
            MPU6050_GetAttitude(&att);
            float current_yaw = att.Yaw;
            float yaw_error   = current_yaw - Mode3_PID.Setpoint;
            
            if (fabsf(yaw_error) < MODE3_LOCK_THRESHOLD)
            {
                
                if (!Mode3_Locked)
                {
                    Mode3_Locked = 1;
                    PID_Reset(&Mode3_PID);
                    Serial_Printf("MODE3: Lock ang=%.1f yaw=%.1f\r\n",
                                  Mode3_ServoAngle, current_yaw);
                }
            }
            else if (fabsf(yaw_error) > MODE3_UNLOCK_THRESHOLD)
            {
                
                if (Mode3_Locked)
                {
                    Mode3_Locked = 0;
                    PID_Reset(&Mode3_PID);
                    Serial_Printf("MODE3: Unlock err=%.1f\r\n", yaw_error);
                }
                float pid_out = PID_Compute(&Mode3_PID, current_yaw);
                Mode3_ServoAngle += pid_out;
                if (Mode3_ServoAngle > 180.0f) Mode3_ServoAngle = 180.0f;
                if (Mode3_ServoAngle < 0.0f)   Mode3_ServoAngle = 0.0f;
                Servo_SetAngle(Mode3_ServoAngle);
            }
            
            
            OLED_ShowFloat(2, 5, current_yaw, 3, 1);        
            OLED_ShowFloat(3, 5, Mode3_PID.Setpoint, 3, 1); 
            OLED_ShowFloat(4, 5, Mode3_ServoAngle, 3, 0);   
            OLED_ShowString(4, 10, Mode3_Locked ? "L" : " ");
            Serial_Printf("%.1f,%.1f,%.1f,%d\r\n",
                          current_yaw, Mode3_PID.Setpoint, Mode3_ServoAngle, Mode3_Locked);
        }
        
        else if (TaskSelect == 2)
        {
            float dist = Ultrasonic_GetDistance();

            if (Mode2_FirstRun)
            {
                Mode2_FirstRun = 0;
                Mode2_TargetYaw = 60.0f;
                Mode2_Dir = 1;
                PID_Reset(&Mode2_PID);
                PID_SetSetpoint(&Mode2_PID, Mode2_TargetYaw);
            }

            uint8_t in_zone = 0;
            float new_paw = Mode2_PAW;

            if (dist >= 20.0f && dist < 25.0f)
                { new_paw = 60.0f;  in_zone = 1; }
            else if (dist >= 40.0f && dist < 45.0f)
                { new_paw = 100.0f; in_zone = 1; }

            if (in_zone)
            {
                Mode2_PAW = new_paw;
                if (!Mode2_PAWLocked)
                {
                    Mode2_DebounceCnt++;
                    if (Mode2_DebounceCnt >= MODE2_DEBOUNCE_THRESH)
                    {
                        Mode2_PAWLocked = 1;
                        Mode2_TargetYaw = Mode2_PAW;
                        PID_SetSetpoint(&Mode2_PID, Mode2_TargetYaw);
                        Serial_Printf("MODE2: PAW Locked at %.1f\r\n", Mode2_PAW);
                    }
                }
            }
            else
            {
                Mode2_PAWLocked = 0;
                Mode2_DebounceCnt = 0;
            }

            if (Mode2_PAWLocked)
            {
                
                Mode2_TargetYaw += Mode2_Dir * 4.0f;
                float half_range = 15.0f;
                if (Mode2_TargetYaw >= Mode2_PAW + half_range)
                    { Mode2_TargetYaw = Mode2_PAW + half_range; Mode2_Dir = -1; }
                if (Mode2_TargetYaw <= Mode2_PAW - half_range)
                    { Mode2_TargetYaw = Mode2_PAW - half_range; Mode2_Dir = 1; }
                PID_SetSetpoint(&Mode2_PID, Mode2_TargetYaw);
            }

            MPU6050_GetAttitude(&att);
            float yaw_rel2 = att.Yaw - Yaw_Zero;
            while (yaw_rel2 > 180.0f)  yaw_rel2 -= 360.0f;
            while (yaw_rel2 < -180.0f) yaw_rel2 += 360.0f;
            float pid_out2 = PID_Compute(&Mode2_PID, yaw_rel2);
            Mode2_ServoAngle += pid_out2;
            if (Mode2_ServoAngle > 180.0f) Mode2_ServoAngle = 180.0f;
            if (Mode2_ServoAngle < 0.0f)   Mode2_ServoAngle = 0.0f;
            Servo_SetAngle(Mode2_ServoAngle);

            float paw_current = Mode2_PAWLocked ? Mode2_PAW : Mode2_PAW;
            OLED_ShowFloat(1, 10, Mode2_TargetYaw, 3, 1);
            OLED_ShowFloat(2, 6, dist, 2, 1);
            OLED_ShowString(2, 10, "cm");
            OLED_ShowFloat(3, 5, Mode2_ServoAngle, 3, 0);
            OLED_ShowFloat(4, 5, paw_current, 3, 1);

            if (Mode2_PAWLocked)
                Serial_Printf("MODE2:%.1f,%.1f,PAW=%.1f,T=%.1f,Locked\r\n",
                              dist, Mode2_ServoAngle, paw_current, Mode2_TargetYaw);
            else if (in_zone)
                Serial_Printf("MODE2:%.1f,%.1f,PAW=%.1f,T=%.1f,Deb(%d/%d)\r\n",
                              dist, Mode2_ServoAngle, paw_current, Mode2_TargetYaw,
                              Mode2_DebounceCnt, MODE2_DEBOUNCE_THRESH);
            else
                Serial_Printf("MODE2:%.1f,%.1f,PAW=%.1f,Idle\r\n",
                              dist, Mode2_ServoAngle, paw_current);
        }
        
        else if (TaskSelect == 4)
        {
            if (Mode4_FirstRun)
            {
                Mode4_FirstRun = 0;
                Mode4_Locked = 0;
                Mode4_ServoAngle = 90.0f;
                PID_Reset(&Mode3_PID);
                Servo_SetAngle(Mode4_ServoAngle);
            }
            
            float dist_raw = Ultrasonic_GetDistance();
            if (dist_raw > 90.0f) dist_raw = 90.0f;
            if (dist_raw < 0.0f)  dist_raw = 0.0f;
            
            dist_ma_sum -= dist_ma_buf[dist_ma_idx];
            dist_ma_buf[dist_ma_idx] = dist_raw;
            dist_ma_sum += dist_ma_buf[dist_ma_idx];
            dist_ma_idx = (dist_ma_idx + 1) % DIST_MA_WINDOW;
            float dist = dist_ma_sum / (float)DIST_MA_WINDOW;
            
            float target_yaw = dist * 2.0f;
            if (target_yaw > 180.0f) target_yaw = 180.0f;
            if (target_yaw < 0.0f)   target_yaw = 0.0f;
            
            PID_SetSetpoint(&Mode3_PID, target_yaw);
            
            MPU6050_GetAttitude(&att);
            float current_yaw = att.Yaw;
            float yaw_error = current_yaw - Mode3_PID.Setpoint;
            
            if (fabsf(yaw_error) < MODE3_LOCK_THRESHOLD)
            {
                if (!Mode4_Locked)
                {
                    Mode4_Locked = 1;
                    PID_Reset(&Mode3_PID);
                    Serial_Printf("MODE4: Lock ang=%.1f yaw=%.1f\r\n",
                               Mode4_ServoAngle, current_yaw);
                }
            }
            else if (fabsf(yaw_error) > MODE3_UNLOCK_THRESHOLD)
            {
                if (Mode4_Locked)
                {
                    Mode4_Locked = 0;
                    PID_Reset(&Mode3_PID);
                    Serial_Printf("MODE4: Unlock err=%.1f\r\n", yaw_error);
                }
                float pid_out = PID_Compute(&Mode3_PID, current_yaw);
                Mode4_ServoAngle += pid_out;
                if (Mode4_ServoAngle > 180.0f) Mode4_ServoAngle = 180.0f;
                if (Mode4_ServoAngle < 0.0f)   Mode4_ServoAngle = 0.0f;
                Servo_SetAngle(Mode4_ServoAngle);
            }
            
            OLED_ShowFloat(2, 6, dist, 2, 1);
            OLED_ShowString(2, 10, "cm");
            OLED_ShowFloat(3, 5, current_yaw, 3, 1);
            OLED_ShowFloat(4, 5, target_yaw, 3, 1);
            OLED_ShowString(4, 10, Mode4_Locked ? "L" : " ");
            Serial_Printf("%.1f,%.1f,%.1f,%.1f,%d\r\n",
                          dist, target_yaw, current_yaw, Mode4_ServoAngle, Mode4_Locked);
        }
        
        else if (TaskSelect == 5)
        {
            if (PA0_Pressed && !scan_ongoing)
            {
                PA0_Pressed = 0;
                Radar_StartScan();
                
                while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0);
                PA0_Pressed = 0;
            }
        }
        
        else if (TaskSelect == 6)
        {
            
            if (Mode6_FirstRun)
            {
                Mode6_FirstRun = 0;
                MPU6050_GetAttitude(&att);
                Mode6_YawZero = att.Yaw;
                Servo_SetAngle(90.0f);
            }
            if (Mode6_SlaveControl == 0)
            {
                
                USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
                MPU6050_GetAttitude(&att);
                float yaw_rel = att.Yaw - Mode6_YawZero;
                while (yaw_rel > 180.0f)  yaw_rel -= 360.0f;
                while (yaw_rel < -180.0f) yaw_rel += 360.0f;
                if (yaw_rel < -90.0f) yaw_rel = -90.0f;
                if (yaw_rel >  90.0f) yaw_rel =  90.0f;
                float servo_angle = yaw_rel + 90.0f;
                Servo_SetAngle(servo_angle);
                Serial_Printf("A%d\r\n", (int)servo_angle);
                { uint32_t _t = 1000; while (_t--) { if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) { uint8_t _c = USART_ReceiveData(USART1); if (_c == 'M') Mode6_SlaveControl = 1; break; } } }
                OLED_ShowFloat(2, 5, servo_angle, 3, 1);
                OLED_ShowFloat(3, 5, servo_angle, 3, 1);
                OLED_ShowString(4, 1, "Host->Slv ");
            }
            else
            {
                
                USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
                if (Serial_YawUpdated)
                {
                    Serial_YawUpdated = 0;
                    float a = Serial_TargetYaw;
                    if (a < 0.0f) a = 0.0f;
                    if (a > 180.0f) a = 180.0f;
                    Servo_SetAngle(a);
                    OLED_ShowFloat(3, 5, a, 3, 1);
                }
                { if (Mode6_ToggleReq) { Mode6_ToggleReq = 0; Mode6_SlaveControl = 0; } }
                OLED_ShowFloat(2, 5, (float)(int)Serial_TargetYaw, 3, 1);
                OLED_ShowString(4, 1, "Slv->Host ");
            }
            
            if (PA0_Pressed)
            {
                PA0_Pressed = 0;
                TaskSelect = 1;
            }
        }
        Delay_ms(10);
    }
}
