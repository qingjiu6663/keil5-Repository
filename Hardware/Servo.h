#ifndef __SERVO_H
#define __SERVO_H

void Servo_Init(void);              /* 舵机初始化（TIM3 软件产生 50Hz 脉冲，PA6 输出） */
void Servo_SetAngle(float Angle);   /* 设置舵机角度 0~180 度 */

#endif
