#ifndef __PID_H
#define __PID_H

typedef struct {
    float Kp;           /* 比例增益 */
    float Ki;           /* 积分增益 */
    float Kd;           /* 微分增益 */

    float Setpoint;     /* 目标值 */
    float OutputMin;    /* 输出下限 */
    float OutputMax;    /* 输出上限 */

    float Integral;     /* 积分累积值 */
    float LastError;    /* 上次误差 */
} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max);
void PID_SetSetpoint(PID_t *pid, float setpoint);
float PID_Compute(PID_t *pid, float feedback);
void PID_Reset(PID_t *pid);

#endif
