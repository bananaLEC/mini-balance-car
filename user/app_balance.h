#ifndef APP_BALANCE_H
#define APP_BALANCE_H

#include "stm32f10x.h"

/* 建议的调用周期，和姿态任务同频 */
#define BAL_TASK_PERIOD_MS  5u

/* 平衡控制系统：速度环(ẋ->θ_ref) -> 角度环(θ->θ̇_ref) -> 角速度环(θ̇->θ̈) 三层串级，
 * θ̈ 经逆解算得 ẍ，积分后除 R_w 得目标轮速，交给电机调速系统。车身速度由编码器和
 * 陀螺重建：ẋ = R_w·ω轮 + l_p·θ̇。依赖先 App_Motor_Init()，Motor_Update 排其后 */

void App_Balance_Init(void);

/* 启动/停止平衡。启动时清零历史数据并重新计时，停止时立刻切断电机输出 */
void App_Balance_Enable(void);
void App_Balance_Disable(void);

/* 平衡控制当前是否已启动：1 = 已启动。按键启停靠它判断该开还是该关 */
uint8_t App_Balance_IsEnabled(void);

/* App_Balance_GetStopReason() 的返回值，用于分清电机突然停转的原因 */
#define BAL_STOP_NONE   (0u)   //没停 / 人为停机（按键、初始化）
#define BAL_STOP_TILT   (1u)   //车身倾角超过 BAL_MAX_ANGLE，判定为倒了
#define BAL_STOP_MPU    (2u)   //传感器数据失效（I2C 连续读失败）

uint8_t App_Balance_GetStopReason(void);   //上一次自动停机的原因码
float   App_Balance_GetStopPitch(void);    //自动停机瞬间的倾角，单位 度

/* 车身速度设定值，单位 m/s。0 表示原地不动 */
void App_Balance_SetTargetSpeed(float MeterPerSec);

/* 机械中位（重心在轮轴正上方时 pitch 的读数），单位 度。运行时改，不用重编 */
void  App_Balance_SetCenterAngle(float Deg);
float App_Balance_GetCenterAngle(void);

/* 平衡控制的进程函数，周期性调用 */
void App_Balance_Update(void);

/* 调试用：平衡环算出的目标轮速(rad/s) */
float App_Balance_GetTargetSpeed(void);

/* 调试用：重建出来的车身线速度(m/s) */
float App_Balance_GetCartSpeed(void);

#endif
