#include "pid.h"
#include "bsp_delay.h"


// 初始化PID。限幅默认取极大值即不限制，需要时用 PID_LimitConfig 设置
void PID_Init(PID_TypeDef *PID, float Kp, float Ki, float Kd)
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->Sp = 0.0f;

	PID->UpperLimit = 3.4e+38f;
	PID->LowerLimit = -3.4e+38f;
	PID->IntUpperLimit = 3.4e+38f;
	PID->IntLowerLimit = -3.4e+38f;

	PID_Reset(PID);
}

// 修改设定值，运行中随时可调
void PID_changeSp(PID_TypeDef *PID,float Sp)
{
	PID->Sp = Sp;
}

// 设置输出限幅
void PID_LimitConfig(PID_TypeDef *PID, float Upper, float Lower)
{
	PID->UpperLimit = Upper;
	PID->LowerLimit = Lower;
}

// 设置积分值限幅（anti-windup）。积分值与输出量纲不同，必须和输出限幅分开配
// 建议积分项对输出的最大贡献取输出限幅的一半，即 积分限幅约 0.5 * 输出限幅 / Ki
void PID_IntLimitConfig(PID_TypeDef *PID, float Upper, float Lower)
{
	PID->IntUpperLimit = Upper;
	PID->IntLowerLimit = Lower;
}

// 复位PID：历史数据清零，Kp/Ki/Kd 与限幅保留；时间戳取当前时刻，否则首拍 deltaT 等于开机至今
void PID_Reset(PID_TypeDef *PID)
{
	PID->t_k_1 = GetUs();
	PID->err_k_1 = 0.0f;
	PID->err_int_k_1 = 0.0f;
	PID->FirstRun = 1u;
}

// 执行一次PID运算。FB 为反馈值，返回控制量；按调用间隔自动算 deltaT，不可重入
float PID_computer(PID_TypeDef *PID, float FB)
{
	float err = PID->Sp - FB;

	uint64_t t_k = GetUs();
	float deltaT = (t_k - PID->t_k_1) * 1.0e-6f;

	float err_dev;
	float err_int;
	float CO;

	// 防止除零：同一微秒内连续调用两次时 deltaT 为 0
	if(deltaT < 1.0e-6f)
	{
		deltaT = 1.0e-6f;
	}

	if(PID->FirstRun != 0u)
	{
		// 首拍：上次误差不存在，积分和微分都无意义，微分会得到 err/deltaT 的巨值造成冲击，只输出比例项
		PID->FirstRun = 0u;
		err_int = 0.0f;
		err_dev = 0.0f;
	}
	else
	{
		err_dev = (err - PID->err_k_1) / deltaT;
		// 积分项：梯形积分，累积误差*时间
		err_int = PID->err_int_k_1 + (err + PID->err_k_1) * 0.5f * deltaT;
	}

	// 积分限幅：不夹住的话，车倒后误差长时间偏大，积分会累积到爆表，扶起后严重过冲
	if(err_int > PID->IntUpperLimit)
	{
		err_int = PID->IntUpperLimit;
	}
	if(err_int < PID->IntLowerLimit)
	{
		err_int = PID->IntLowerLimit;
	}

	CO = PID->Kp * err + PID->Ki * err_int + PID->Kd * err_dev;

	// 输出限幅
	if(CO > PID->UpperLimit)
	{
		CO = PID->UpperLimit;
	}
	if(CO < PID->LowerLimit)
	{
		CO = PID->LowerLimit;
	}

	// 保存本拍状态供下一拍使用
	PID->t_k_1 = t_k;
	PID->err_k_1 = err;
	PID->err_int_k_1 = err_int;

	return CO;
}
