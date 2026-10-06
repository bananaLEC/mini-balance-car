#include "app_attitude.h"
#include "app_mpu6050.h"
#include <math.h>          // atan2f / sqrtf
#include "bsp_delay.h"

/* 安装方向适配区：板子转 90° 安装就交换 ATT_AX 和 ATT_AY，装反则翻转对应符号。
 * 改完必须重新确认每个轴的符号，再动下面的滤波参数。 */
#define ATT_AX  (App_MPU6050_GetAx())
#define ATT_AY  (App_MPU6050_GetAy())
#define ATT_AZ  (App_MPU6050_GetAz())
#define ATT_GX  (App_MPU6050_GetGx())
#define ATT_GY  (App_MPU6050_GetGy())
#define ATT_GZ  (App_MPU6050_GetGz())

#define ATT_SIGN_ROLL   (-1.0f)
#define ATT_SIGN_PITCH  (+1.0f)
#define ATT_SIGN_YAW    (+1.0f)

/* 互补滤波时间常数，单位秒。越大越平滑但跟随真实运动越慢，越小跟随快但读数越抖 */
#define ATT_TAU_S       (0.5f)
#define ATT_RAD2DEG     (57.29578f)

/* 零偏校准时采样的次数和间隔（200 * 5ms ≈ 1 秒） */
#define ATT_CALI_TIMES  200u
#define ATT_CALI_DELAY  5u

static float roll  = 0.0f;   //横滚角，单位度
static float pitch = 0.0f;   //俯仰角，单位度
static float yaw   = 0.0f;   //偏航角，单位度

static float gz_bias = 0.0f;      //陀螺Z轴零偏，单位 度/s
static uint32_t att_last_ms = 0;  //上次更新的时刻，单位 ms

//初始化姿态解算：内部静止采样约 1 秒校准陀螺零偏，期间板子必须保持不动
void App_Attitude_Init(void)
{
	float sum = 0.0f;
	uint32_t i;

	for(i = 0; i < ATT_CALI_TIMES; i++)
	{
		App_MPU6050_Update();   //必须先刷新，否则每次采到的都是同一个旧值
		sum += App_MPU6050_GetGz();
		Delay(ATT_CALI_DELAY);
	}
	gz_bias = sum / (float)ATT_CALI_TIMES;

	roll  = 0.0f;
	pitch = 0.0f;
	yaw   = 0.0f;

	att_last_ms = GetTick();
}

//更新三个姿态角，需周期性调用
void App_Attitude_Update(void)
{
	uint32_t now = GetTick();
	float dt = (now - att_last_ms) * 0.001f;
	float k;
	float roll_acc;
	float pitch_acc;

	att_last_ms = now;

	//限幅：调度器卡顿或调试暂停会让 dt 异常大，积分一步就跳很远
	if(dt < 0.001f)
	{
		dt = 0.001f;
	}
	else if(dt > 0.05f)
	{
		dt = 0.05f;
	}

	//互补滤波系数，即陀螺权重：dt 越大说明越久没更新，越多依赖加速度计
	k = ATT_TAU_S / (ATT_TAU_S + dt);

	//加速度计参考角：静止时准确，有运动加速度就被污染
	//用标准库 atan2f：分子分母同时缩放不影响结果，at_y=at_z=0 也能返回 0 而不是 NaN
	roll_acc  = atan2f(ATT_AY, ATT_AZ) * ATT_RAD2DEG;
	pitch_acc = atan2f(-ATT_AX, sqrtf(ATT_AY * ATT_AY + ATT_AZ * ATT_AZ)) * ATT_RAD2DEG;

	//递推：陀螺积分（短期准）+ 加速度计校正（长期准），积分的是已滤波的角度
	roll  = k * (roll  + ATT_GX * ATT_SIGN_ROLL  * dt) + (1.0f - k) * roll_acc;
	pitch = k * (pitch + ATT_GY * ATT_SIGN_PITCH * dt) + (1.0f - k) * pitch_acc;

	//yaw 没有磁力计校正，只能靠陀螺积分，所以要扣掉零偏来减缓漂移
	yaw = yaw + (ATT_GZ - gz_bias) * ATT_SIGN_YAW * dt;
}

//把当前朝向记为 0 度（yaw 会漂移，需要时手动归零）
void App_Attitude_ResetYaw(void)
{
	yaw = 0.0f;
}

float App_Attitude_GetRoll(void)
{
	return roll;
}

float App_Attitude_GetPitch(void)
{
	return pitch;
}

float App_Attitude_GetYaw(void)
{
	return yaw;
}

//俯仰角速度，单位 度/s。直接取陀螺 Y 轴，符号经 ATT_SIGN_PITCH 修正，与 GetPitch 方向一致
float App_Attitude_GetPitchRate(void)
{
	return ATT_GY * ATT_SIGN_PITCH;
}
