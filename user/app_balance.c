#include "app_balance.h"
#include "app_attitude.h"
#include "app_mpu6050.h"
#include "app_encoder.h"
#include "app_motor.h"
#include "pid.h"
#include <math.h>          // atanf / sinf / cosf / tanf / fabsf
#include "bsp_delay.h"

/* 角度 -> 弧度 */
#define BAL_DEG2RAD     (0.0174533f)

/* ===================== 需要按实车填的参数 ===================== */

/* 轮胎半径 R_w，单位 m */
#define WHEEL_R         (0.033f)

/* 摆杆长度：轮轴到车体质心的距离，单位 m。重建车身速度与逆解算都要用它 */
#define L_P             (0.075f)

/* 重力加速度 */
#define G_ACC           (9.8f)

/* 机械中位默认值：车身竖直平衡时 pitch 的读数，单位 度。填 0 表示把 pitch=0 当作竖直，
 * 实测后改这里重编，或用 App_Balance_SetCenterAngle() 运行时改；值不准车会朝一边爬跑掉 */
#define BAL_CENTER_ANGLE_DEFAULT (-0.30f)

/* 倾角符号：前倾时 GetPitch() 为正，正是逆解算要的约定（前倾 θ 为正），故取 +1.0f；
 * 换了 MPU 安装方向、前倾变负值时改成 -1.0f */
#define BAL_SIGN_PITCH  (+1.0f)


/* ===================== 速度环参数（最外环） =====================
 * 车身线速度 ẋ -> 目标倾角 θ_ref：ẍ_ref = PID(ẋ_ref - ẋ)，θ_ref = atan(ẍ_ref/g)
 * Ki 取 Kp 的 1/10 扛中位残差和坡度。输出单位 m/s²，不能照抄参考工程里度的 Kp */
#define BAL_KP_SPEED    (0.15f)
#define BAL_KI_SPEED    (0.015f)

/* ===================== 整定开关：速度环 =====================
 * 改 0 只留角度环和角速度环，车只立正不管速度；此时仍振荡说明问题在平衡环内部
 */
#define BAL_SPEED_LOOP_ENABLE  (1u)

/* 允许车倾斜的最大角度：决定速度环最多能用多大的倾角纠正速度 */
#define BAL_MAX_TILT_DEG    (10.0f)
#define BAL_MAX_TILT        (BAL_MAX_TILT_DEG * BAL_DEG2RAD)

/* 速度环输出（目标线加速度）的限幅，与最大倾角等价：ẍ = g·tanθ */
#define BAL_SPEED_OUT_MAX   (G_ACC * tanf(BAL_MAX_TILT))

/* 速度环积分限幅：积分项最多贡献输出限幅的一半 */
#define BAL_SPEED_IMAX      (0.25f * BAL_SPEED_OUT_MAX / BAL_KI_SPEED)


/* ===================== 角度环参数 ===================== */

/* 外环：角度 -> 目标角速度，只用比例。回正太软优先加大 BAL_KP_INNER。输出限幅 ±4π rad/s */
#define BAL_KP_OUTER    (2.0f)
#define BAL_OUTER_MAX   (4.0f * 3.14159265f)

/* 内环：角速度 -> 目标角加速度，PI。输出限幅 ±40π rad/s² */
#define BAL_KP_INNER    (14.0f)
#define BAL_KI_INNER    (10.0f)
#define BAL_INNER_MAX   (40.0f * 3.14159265f)

/* 内环积分限幅：积分项最多贡献输出限幅的一半 */
#define BAL_INNER_IMAX  (0.5f * BAL_INNER_MAX / BAL_KI_INNER)


/* ===================== 安全保护 ===================== */
/* 车身倾斜超过这个角度（度）就判定为倒了，留出更大的挽救余地 */
#define BAL_MAX_ANGLE   (45.0f)

/* 倾角超限后还要持续这么久（ms）才算真倒了，用来滤掉修正动作造成的尖峰 */
#define BAL_FALL_HOLD_MS  (100u)

/* 目标轮速 omega_ref 限幅，单位 rad/s。必须限：它由目标线加速度积分经 l_p/R_w ≈ 2.3 倍
 * 放大而来，不限能冲到几百 rad/s；40 rad/s 对应轮速约 1.3 m/s，再往上只是空转 */
#define BAL_OMEGA_REF_MAX   (40.0f)


static PID_TypeDef pid_speed;   //最外环：车身速度环
static PID_TypeDef pid_angle;   //外环：角度环
static PID_TypeDef pid_rate;    //内环：角速度环

static float omega_ref = 0.0f;        //目标轮速，单位 rad/s
static uint64_t balance_last_us = 0;  //上次运算的时刻
static uint8_t balance_enabled = 0u;  //平衡控制是否已启动

/* 自动停机的原因码和当时的倾角，调试用 */
static uint8_t balance_stop_reason = BAL_STOP_NONE;
static float   balance_stop_pitch  = 0.0f;

/* 倾角第一次超过 BAL_MAX_ANGLE 的时刻，0 = 当前没超，配合 BAL_FALL_HOLD_MS 计时 */
static uint32_t bal_fall_since_ms = 0u;

/* 机械中位，单位 度。上电取默认值，标定可用 App_Balance_SetCenterAngle() 改 */
static float bal_center_angle = BAL_CENTER_ANGLE_DEFAULT;


//初始化平衡控制系统
void App_Balance_Init(void)
{
	PID_Init(&pid_speed, BAL_KP_SPEED, BAL_KI_SPEED, 0.0f);
	PID_LimitConfig(&pid_speed, BAL_SPEED_OUT_MAX, -BAL_SPEED_OUT_MAX);
	PID_IntLimitConfig(&pid_speed, BAL_SPEED_IMAX, -BAL_SPEED_IMAX);

	PID_Init(&pid_angle, BAL_KP_OUTER, 0.0f, 0.0f);
	PID_LimitConfig(&pid_angle, BAL_OUTER_MAX, -BAL_OUTER_MAX);

	PID_Init(&pid_rate, BAL_KP_INNER, BAL_KI_INNER, 0.0f);
	PID_LimitConfig(&pid_rate, BAL_INNER_MAX, -BAL_INNER_MAX);
	PID_IntLimitConfig(&pid_rate, BAL_INNER_IMAX, -BAL_INNER_IMAX);

	//机械中位恢复为编译时的默认值
	bal_center_angle = BAL_CENTER_ANGLE_DEFAULT;

	App_Balance_Disable();
}

//设置车身速度设定值，单位 m/s。0 表示原地不动
void App_Balance_SetTargetSpeed(float MeterPerSec)
{
	PID_changeSp(&pid_speed, MeterPerSec);
}

//设置机械中位，单位 度。标定时用，不用重编
void App_Balance_SetCenterAngle(float Deg)
{
	bal_center_angle = Deg;
}

//读取当前机械中位，单位 度
float App_Balance_GetCenterAngle(void)
{
	return bal_center_angle;
}

//启动平衡控制：清零所有历史数据和目标轮速，重新开始计时
void App_Balance_Enable(void)
{
	omega_ref = 0.0f;
	balance_last_us = GetUs();

	//清掉上一次的停机记录，让调试打印反映本次运行
	balance_stop_reason = BAL_STOP_NONE;
	balance_stop_pitch  = 0.0f;
	bal_fall_since_ms   = 0u;

	PID_changeSp(&pid_speed, 0.0f);
	PID_Reset(&pid_speed);

	PID_Reset(&pid_angle);
	PID_Reset(&pid_rate);

	App_Motor_Enable();

	balance_enabled = 1u;
}

//停止平衡控制：立刻切断电机输出
void App_Balance_Disable(void)
{
	balance_enabled = 0u;
	omega_ref = 0.0f;

	//默认记成人为停机，下面两个自动停机的分支会改成对应原因码
	balance_stop_reason = BAL_STOP_NONE;

	App_Motor_Disable();
}

//读取上一次自动停机的原因码，取值见 app_balance.h 里的 BAL_STOP_xxx
uint8_t App_Balance_GetStopReason(void)
{
	return balance_stop_reason;
}

//读取自动停机瞬间的车身倾角，单位 度（人为停机时是 0）
float App_Balance_GetStopPitch(void)
{
	return balance_stop_pitch;
}

//平衡控制是否已启动：1 = 已启动，按键启停靠它判断该开还是该关
uint8_t App_Balance_IsEnabled(void)
{
	return balance_enabled;
}

//平衡控制的进程函数。必须周期性调用，且排在 App_Motor_Update 之前，
//否则本拍算出的目标轮速要等到下一拍才被执行
void App_Balance_Update(void)
{
	uint64_t now;
	float dt;

	float theta;         //车身倾角，rad
	float theta_dot;     //车身角速度，rad/s
	float x_dot;         //重建出来的车身线速度，m/s
	float x_ddot_ref;    //速度环输出：目标线加速度，m/s²
	float theta_ref;     //速度环输出：目标倾角，rad
	float theta_dot_ref; //角度环输出：目标角速度，rad/s
	float theta_ddot;    //角速度环输出：目标角加速度，rad/s²
	float x_ddot;        //逆解算输出：目标线加速度，m/s²

	//没启动就不运算，免得PID积分白累积
	if(balance_enabled == 0u)
	{
		return;
	}

	//传感器数据失效（I2C 连续读失败）必须马上停机，否则就是拿错数据去驱动电机
	if(App_MPU6050_IsOk() == 0u)
	{
		App_Balance_Disable();
		balance_stop_reason = BAL_STOP_MPU;
		return;
	}

	now = GetUs();
	dt = (now - balance_last_us) * 1.0e-6f;
	balance_last_us = now;

	//限幅：调度卡顿时 dt 会异常大，积分一步就冲过头
	if(dt < 1.0e-6f)
	{
		dt = 1.0e-6f;
	}
	else if(dt > 0.05f)
	{
		dt = 0.05f;
	}

	//【2】读传感器：倾角(rad) 和 角速度(rad/s)，乘 BAL_SIGN_PITCH 统一为前倾为正
	theta     = App_Attitude_GetPitch()     * BAL_DEG2RAD * BAL_SIGN_PITCH;
	theta_dot = App_Attitude_GetPitchRate() * BAL_DEG2RAD * BAL_SIGN_PITCH;

	//安全保护：倾角超限且持续 BAL_FALL_HOLD_MS 才算真倒了，滤掉修正动作造成的尖峰
	if(fabsf(theta) > BAL_MAX_ANGLE * BAL_DEG2RAD)
	{
		if(bal_fall_since_ms == 0u)
		{
			bal_fall_since_ms = GetTick();   //第一次超限，开始计时
		}
		else if((uint32_t)(GetTick() - bal_fall_since_ms) >= BAL_FALL_HOLD_MS)
		{
			App_Balance_Disable();
			balance_stop_reason = BAL_STOP_TILT;
			balance_stop_pitch  = App_Attitude_GetPitch();   //把倒下瞬间的角度记下来
			return;
		}
	}
	else
	{
		bal_fall_since_ms = 0u;   //回到范围内，计时清零
	}

	/* ---------- 速度环（最外环）---------- */

	//用编码器实测的轮速重建车身线速度：ẋ = R_w·ω轮 + l_p·θ̇，左右轮取平均
	x_dot = WHEEL_R * ((App_Encoder_GetSpeed_L() + App_Encoder_GetSpeed_R())
	                   * 0.5f * BAL_DEG2RAD)
	        + L_P * theta_dot;

	if(BAL_SPEED_LOOP_ENABLE != 0u)
	{
		x_ddot_ref = PID_computer(&pid_speed, x_dot);
	}
	else
	{
		//速度环关掉（整定用）：目标倾角就是机械中位，车只会立正，久了会慢慢漂
		x_ddot_ref = 0.0f;
	}

	theta_ref = atanf(x_ddot_ref / G_ACC);

	//限幅：最多允许倾斜 BAL_MAX_TILT_DEG
	if(theta_ref > BAL_MAX_TILT)
	{
		theta_ref = BAL_MAX_TILT;
	}
	if(theta_ref < -BAL_MAX_TILT)
	{
		theta_ref = -BAL_MAX_TILT;
	}

	//叠加上机械中位：目标倾角是相对"车竖直"而言的
	theta_ref += bal_center_angle * BAL_DEG2RAD;

	/* ---------- 角度环 ---------- */

	//【3】角度环PID：角度误差 -> 目标角速度
	PID_changeSp(&pid_angle, theta_ref);
	theta_dot_ref = PID_computer(&pid_angle, theta);

	/* ---------- 角速度环 ---------- */

	//【4】把角度环的输出作为角速度环的设定值
	PID_changeSp(&pid_rate, theta_dot_ref);

	//【5】角速度环PID：角速度误差 -> 目标角加速度
	theta_ddot = PID_computer(&pid_rate, theta_dot);

	//【6】逆解算：目标角加速度 -> 目标线加速度，ẍ = (g·sinθ − l_p·θ̈) / cosθ
	//【注意】θ̈ 前面必须是减号，写成加号等于把内环变成正反馈，车永远立不住；手扶慢倾时 θ̈≈0 看不出，只有闭环才暴露
	x_ddot = (G_ACC * sinf(theta) - L_P * theta_ddot) / cosf(theta);

	//【7】目标轮速 = 目标线加速度积分，再除以轮胎半径
	omega_ref += x_ddot / WHEEL_R * dt;

	//限幅：见 BAL_OMEGA_REF_MAX。限的是积分本身，不会冲飞之后再慢慢退回来
	if(omega_ref > BAL_OMEGA_REF_MAX)       { omega_ref = BAL_OMEGA_REF_MAX; }
	else if(omega_ref < -BAL_OMEGA_REF_MAX) { omega_ref = -BAL_OMEGA_REF_MAX; }

	//【8】把目标轮速交给左右电机的调速系统
	App_Motor_SetTarget_L(omega_ref);
	App_Motor_SetTarget_R(omega_ref);
}

//读取平衡环算出的目标轮速，单位 rad/s
float App_Balance_GetTargetSpeed(void)
{
	return omega_ref;
}

//读取重建出来的车身线速度，单位 m/s（调试用）
float App_Balance_GetCartSpeed(void)
{
	return WHEEL_R * ((App_Encoder_GetSpeed_L() + App_Encoder_GetSpeed_R())
	                  * 0.5f * BAL_DEG2RAD)
	       + L_P * (App_Attitude_GetPitchRate() * BAL_DEG2RAD);
}
