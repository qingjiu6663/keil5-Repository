/*
 * PID 控制器（位置式 PID + 条件积分 + 积分限幅）
 * 条件积分：|误差|>5 度时冻结积分，防止大误差下积分饱和
 * 积分限幅：防止 Ki*Integral 独占输出（限为输出范围的 25%）
 */
#include "PID.h"

/* PID 初始化：设置增益、输出限幅，清零积分和上次误差 */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->OutputMin = out_min;
    pid->OutputMax = out_max;
    pid->Setpoint = 0.0f;
    pid->Integral = 0.0f;
    pid->LastError = 0.0f;
}

/* 设置 PID 目标值 */
void PID_SetSetpoint(PID_t *pid, float setpoint)
{
    pid->Setpoint = setpoint;
}

/*
 * PID 计算
 * 反馈误差 = 目标值 - 反馈值
 * 输出 = P(比例) + I(积分) + D(微分)，再限幅到输出范围
 * 条件积分：误差在 -5~5 度范围内才累积积分，超出时冻结
 */
float PID_Compute(PID_t *pid, float feedback)
{
    float error = pid->Setpoint - feedback;
    float P = pid->Kp * error;                     /* 比例项 */

    if (error > -5.0f && error < 5.0f)             /* 条件积分：小误差才累积 */
    {
        pid->Integral += error;

        float iMax = (pid->OutputMax - pid->OutputMin) * 0.25f;
        if (pid->Ki > 0.001f || pid->Ki < -0.001f)
        {
            float iLimit = iMax / pid->Ki;         /* 积分限幅 */
            if (pid->Integral > iLimit) pid->Integral = iLimit;
            if (pid->Integral < -iLimit) pid->Integral = -iLimit;
        }
    }

    float I = pid->Ki * pid->Integral;             /* 积分项 */

    float D = pid->Kd * (error - pid->LastError);  /* 微分项 */
    pid->LastError = error;

    float output = P + I + D;                      /* 总输出 */

    if (output > pid->OutputMax) output = pid->OutputMax;
    if (output < pid->OutputMin) output = pid->OutputMin;

    return output;
}

/* 重置 PID：清零积分和上次误差 */
void PID_Reset(PID_t *pid)
{
    pid->Integral = 0.0f;
    pid->LastError = 0.0f;
}
