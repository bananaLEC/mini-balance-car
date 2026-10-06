#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include "stm32f10x.h"                  // Device header

/* 建议调用周期 5ms，和姿态、平衡任务同频 */
#define MOTOR_TASK_PERIOD_MS  5u

void App_Motor_Init(void);

/* 复位调速系统：目标速度归零 + 清空PID历史数据，重新启动小车时调用 */
void App_Motor_Reset(void);

/* 输出使能开关。初始化后默认关闭，必须显式 Enable 电机才会转；车倒了或按键没按下
 * 时用 Disable 把输出彻底切断 */
void App_Motor_Enable(void);
void App_Motor_Disable(void);

/* 设定目标角速度，单位 rad/s。正数正转，负数反转，0 停转 */
void App_Motor_SetTarget_L(float RadPerSec);
void App_Motor_SetTarget_R(float RadPerSec);

/* 读取最后一次算出的占空比，单位 %，范围 -100~100，调试用 */
float App_Motor_GetDuty_L(void);
uint16_t App_Motor_GetSpeedRejectCount(void);   /* 假速度读数被拦下的次数 */
float App_Motor_GetDuty_R(void);

/* 调速系统的周期任务，需要在 main 里注册进调度器 */
void App_Motor_Update(void);

#endif
