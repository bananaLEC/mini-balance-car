#ifndef APP_ATTITUDE_H
#define APP_ATTITUDE_H

#include "stm32f10x.h"

/* 姿态解算：由 MPU6050 六轴数据互补滤波递推得到 roll/pitch/yaw，单位度。
 * 无磁力计，yaw 没有绝对参考、必然缓慢漂移（约 1~10 度/分钟），需要归零时调用
 * App_Attitude_ResetYaw()，不要基于 yaw 做转向闭环控制。 */

/* 建议的调用周期，注册任务时用这个值，与模块内部的滤波系数配套 */
#define ATT_TASK_PERIOD_MS  5u

void  App_Attitude_Init(void);       //注意：内部会校准陀螺零偏，耗时约1秒，期间板子必须静止
void  App_Attitude_Update(void);     //周期性调用，建议 ATT_TASK_PERIOD_MS 毫秒一次
void  App_Attitude_ResetYaw(void);   //把当前朝向记为 yaw = 0

float App_Attitude_GetRoll(void);    //横滚角，单位度
float App_Attitude_GetPitch(void);   //俯仰角，单位度（平衡车用的就是这个）
float App_Attitude_GetYaw(void);     //偏航角，单位度（会漂移）

/* 俯仰角速度，单位 度/s，直接取陀螺，符号同 GetPitch。平衡环内环用这个 */
float App_Attitude_GetPitchRate(void);

#endif
