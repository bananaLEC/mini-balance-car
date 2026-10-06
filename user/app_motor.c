#include "app_motor.h"
#include "pid.h"
#include "app_encoder.h"
#include "app_pwm.h"
#include "math.h"

/* 度 -> 弧度：π/180 */
#define MOTOR_DEG2RAD   (0.0174533f)

/* 调速环 PID。5ms 周期，输出占空比 %，反馈轮速 rad/s。
 * 平衡环要靠它接住车身，要求在 30~50ms 内把轮速拉到目标，太软车立不住。
 * 整定用 Motor_Speed_Test 看阶跃响应：太慢加 Kp，低速振荡就收 Kp，Ki 磨稳态误差。 */
#define MOTOR_KP        (3.0f)
#define MOTOR_KI        (6.0f)
#define MOTOR_KD        (0.0f)

/* 输出限幅：PID的输出直接当PWM占空比用，范围是 ±100% */
#define MOTOR_OUT_LIMIT (100.0f)

/* 积分限幅：积分项最多贡献输出限幅的一半，写成算式省得改 MOTOR_KI 时重新推算 */
#define MOTOR_INT_LIMIT (0.2f * MOTOR_OUT_LIMIT / MOTOR_KI)

/* 【尚未实现】调速输出每拍允许的最大变化量（软启动/软换向），防突变直接打满 PWM */

/* ===================== 静摩擦死区补偿 =====================
 * 齿轮箱有静摩擦门槛：占空比不够纹丝不动，刚过门槛又蹦一段。输出不为 0 时至少给
 * ±dz 跨过门槛，输出为 0 时保持 0，车才停得住。停着用起动门槛，转起来线性回落到
 * 维持门槛；固定一个大补偿会让车一直在"冲-过头"里循环。单位 %。
 * 输出小于 MOTOR_OUT_DEADBAND 直接当 0，两路在 0 附近的符号噪声才不会互相打架。
 * 两路门槛本来就不一样，必须分开填，填同一个值会让门槛低的那一路一直转得多。
 * 值从 Motor_Crawl_Test 量（轮子架空），架空是下界、落地负载更大门槛更高，取余量约 2%。
 *
 * 【别把 START 填 0】那不等于关掉补偿，而是把它反过来：轮速 0 时 dz = 0 照样起不来，
 * 转起来反而一直多给 RUN。要真关掉就四个值一起填 0。 */
#define MOTOR_DZ_START_L    (8.0f)    /* 左路起动门槛+余量，% */
#define MOTOR_DZ_START_R    (6.0f)    /* 右路起动门槛+余量，% */
#define MOTOR_DZ_RUN_L      (3.0f)    /* 左路维持门槛+余量，% */
#define MOTOR_DZ_RUN_R      (2.0f)    /* 右路维持门槛+余量，% */

/* 补偿回落的参考轮速，单位 rad/s：轮子转到这个速度，补偿就降到"维持"那一档 */
#define MOTOR_DZ_FADE_SPD   (3.0f)

/* 调速环输出的死区，单位 %（占空比）。比它更小的输出直接当作 0、不做补偿 */
#define MOTOR_OUT_DEADBAND  (0.5f)

/* ===================== 假读数的最后一道闸 =====================
 * 编码器假边沿（开关噪声串进信号线）会让读数瞬间跳到上百 rad/s，调速环以为轮子已在
 * 飞转，立刻把占空比打到反向极限，直接把车踹出去。不能靠"绝对值限幅"来挡：截在
 * 100 rad/s 对 Kp=3 仍是 300 的巨大误差，照样打满。要拦的是"变化得多快"——轮子角
 * 加速度有物理上限，一个 5ms 周期顶多几 rad/s，一步跳 10 rad/s 以上一定是假的。
 * 假读数时沿用上一拍的值，恢复的那一拍把对应 PID 重置一次。
 * 兜底降级：一路不可信就两路输出对称、跟可信的那一路走，两路都不可信就都维持上一拍。
 * 依据是两个轮子同轴、没有打滑，直行本来就该同速，比让坏的那一路自己乱算安全。 */
#define MOTOR_SPD_JUMP_MAX  (10.0f)   /* 每个 5ms 周期允许的轮速变化，rad/s */

static float motor_spd_use_L = 0.0f;   /* 上一拍采用的轮速，rad/s */
static float motor_spd_use_R = 0.0f;
static uint16_t motor_spd_reject = 0u; //假读数被拦下的次数

/* 调速环 PID 原始输出（未做死区补偿），单位 %。与 motor_duty_L/R 分开存，是为了兜底时
 * 能交换控制决策、但仍各按各的门槛补偿 */
static float motor_pidout_L = 0.0f;
static float motor_pidout_R = 0.0f;

/* 这一路是否正处在"假读数"状态：1 = 是。恢复的那一拍要把对应的 PID 重置一次 */
static uint8_t motor_glitch_L = 0u;
static uint8_t motor_glitch_R = 0u;

static PID_TypeDef pid_motor_L;//左电机调速系统的PID控制器
static PID_TypeDef pid_motor_R;//右电机调速系统的PID控制器

static uint8_t motor_enabled = 0u;//输出使能：上电默认关闭，必须显式使能

/* 最后一次算出来的占空比，单位 %。调试用：电机不转时看这里是 0 还是 ±100 */
static float motor_duty_L = 0.0f;
static float motor_duty_R = 0.0f;


void App_Motor_Init(void)
{
	motor_enabled = 0u;   //安全起见，初始化后不输出，等上层显式使能

	PID_Init(&pid_motor_L, MOTOR_KP, MOTOR_KI, MOTOR_KD);
	PID_LimitConfig(&pid_motor_L, MOTOR_OUT_LIMIT, -MOTOR_OUT_LIMIT);
	PID_IntLimitConfig(&pid_motor_L, MOTOR_INT_LIMIT, -MOTOR_INT_LIMIT);

	PID_Init(&pid_motor_R, MOTOR_KP, MOTOR_KI, MOTOR_KD);
	PID_LimitConfig(&pid_motor_R, MOTOR_OUT_LIMIT, -MOTOR_OUT_LIMIT);
	PID_IntLimitConfig(&pid_motor_R, MOTOR_INT_LIMIT, -MOTOR_INT_LIMIT);
}

void App_Motor_Enable(void)
{
	motor_enabled = 1u;
}

//关闭电机输出：立刻切断PWM，并把PID历史数据清掉
void App_Motor_Disable(void)
{
	motor_enabled = 0u;

	motor_duty_L = 0.0f;
	motor_duty_R = 0.0f;

	App_PWM_Set_L(0.0f);
	App_PWM_Set_R(0.0f);

	App_Motor_Reset();
}

//读取最后一次算出的占空比，单位 %（调试用）
float App_Motor_GetDuty_L(void)
{
	return motor_duty_L;
}

float App_Motor_GetDuty_R(void)
{
	return motor_duty_R;
}

//复位调速系统：目标速度归零 + 清掉PID累积的历史数据，重新启动小车时调用
void App_Motor_Reset(void)
{
	PID_changeSp(&pid_motor_L, 0.0f);
	PID_changeSp(&pid_motor_R, 0.0f);

	PID_Reset(&pid_motor_L);
	PID_Reset(&pid_motor_R);

	//假读数判据和兜底状态也跟着清零，重新开始
	motor_spd_use_L = 0.0f;
	motor_spd_use_R = 0.0f;

	motor_pidout_L = 0.0f;
	motor_pidout_R = 0.0f;
	motor_glitch_L = 0u;
	motor_glitch_R = 0u;
}

//读取假读数被拦下的次数（调试用）；这个数一直涨说明编码器那边还有干扰，得从接线/屏蔽下手
uint16_t App_Motor_GetSpeedRejectCount(void)
{
	return motor_spd_reject;
}

//设定左电机目标角速度，单位 rad/s
void App_Motor_SetTarget_L(float RadPerSec)
{
	PID_changeSp(&pid_motor_L, RadPerSec);
}

//设定右电机目标角速度，单位 rad/s
void App_Motor_SetTarget_R(float RadPerSec)
{
	PID_changeSp(&pid_motor_R, RadPerSec);
}

/* 假读数拦截：一步变化超过 MOTOR_SPD_JUMP_MAX 就沿用上一次的值，并更新 *pPrev。
 * *pOk 置 0 表示这一拍是假读数，上层据此走兜底逻辑 */
static float Motor_PlausSpeed(float Spd, float *pPrev, uint8_t *pOk)
{
	float delta = Spd - *pPrev;

	if(delta > MOTOR_SPD_JUMP_MAX || delta < -MOTOR_SPD_JUMP_MAX)
	{
		motor_spd_reject++;
		*pOk = 0u;
		return *pPrev;   /* 不动 *pPrev，等轮到真读数再更新 */
	}

	*pPrev = Spd;
	*pOk = 1u;
	return Spd;
}

/* 死区补偿：输出为正至少给 +dz，为负至少给 -dz。dz 由轮速决定，停着用 DzStart，
 * 随轮速升到 MOTOR_DZ_FADE_SPD 线性回落到 DzRun。四个值都填 0 等于没做 */
static float Motor_DeadZoneComp(float Duty, float Spd, float DzStart, float DzRun)
{
	float dz;
	float abs_spd;

	/* 输出太小：当作不想动给 0。此时 PID 积分还在累，涨到死区外自然会推出来 */
	if(Duty > -MOTOR_OUT_DEADBAND && Duty < MOTOR_OUT_DEADBAND)
	{
		return 0.0f;
	}

	abs_spd = fabsf(Spd);
	if(abs_spd >= MOTOR_DZ_FADE_SPD)
	{
		dz = DzRun;
	}
	else
	{
		/* 速度越低补偿越接近起动门槛。用线性过渡而不是分档，免得在临界速度上来回跳 */
		dz = DzRun + (DzStart - DzRun) * (1.0f - abs_spd / MOTOR_DZ_FADE_SPD);
	}

	if(Duty > 0.0f)
	{
		return Duty + dz;
	}
	return Duty - dz;
}

//调速系统周期任务：读编码器速度 -> PID运算 -> 输出PWM
//必须周期性调用，周期 5ms，且要排在平衡环之后
void App_Motor_Update(void)
{
	//没使能就直接切断输出，PID也不运算
	if(motor_enabled == 0u)
	{
		App_PWM_Set_L(0.0f);
		App_PWM_Set_R(0.0f);
		return;
	}

	//编码器返回的是 度/s，先换算成 rad/s
	float spd_L = App_Encoder_GetSpeed_L() * MOTOR_DEG2RAD;
	float spd_R = App_Encoder_GetSpeed_R() * MOTOR_DEG2RAD;

	uint8_t ok_L = 1u;   //这一拍两路读数可不可信
	uint8_t ok_R = 1u;

	//假读数拦截（见 MOTOR_SPD_JUMP_MAX）：假读数绝对不能进 PID
	spd_L = Motor_PlausSpeed(spd_L, &motor_spd_use_L, &ok_L);
	spd_R = Motor_PlausSpeed(spd_R, &motor_spd_use_R, &ok_R);

	//从"假读数"恢复的那一拍先重置这个环再算：PID 里 t_k_1 还停在干扰开始之前，
	//不重置的话恢复瞬间 deltaT 是一大截，积分会被灌进去一大步
	if(ok_L != 0u && motor_glitch_L != 0u)
	{
		PID_Reset(&pid_motor_L);
		motor_glitch_L = 0u;
	}
	if(ok_R != 0u && motor_glitch_R != 0u)
	{
		PID_Reset(&pid_motor_R);
		motor_glitch_R = 0u;
	}

	//哪一路可信就更新哪一路，PID输出先不加补偿（兜底要在这一层交换决策）
	if(ok_L != 0u)
	{
		motor_pidout_L = PID_computer(&pid_motor_L, spd_L);
	}
	else
	{
		motor_glitch_L = 1u;   //这一路在假读数中：不跑PID，免得拿旧数据攒积分
	}

	if(ok_R != 0u)
	{
		motor_pidout_R = PID_computer(&pid_motor_R, spd_R);
	}
	else
	{
		motor_glitch_R = 1u;
	}

	/* 兜底降级：一路不可信就两路输出对称、跟可信的那路走；两路都不可信就都维持上一拍 */
	if(ok_L == 0u && ok_R != 0u)
	{
		motor_pidout_L = motor_pidout_R;
	}
	if(ok_R == 0u && ok_L != 0u)
	{
		motor_pidout_R = motor_pidout_L;
	}

	//跨过静摩擦门槛。两路门槛不同，所以各按各的参数补偿，哪怕决策是抄来的
	motor_duty_L = Motor_DeadZoneComp(motor_pidout_L, spd_L, MOTOR_DZ_START_L, MOTOR_DZ_RUN_L);
	motor_duty_R = Motor_DeadZoneComp(motor_pidout_R, spd_R, MOTOR_DZ_START_R, MOTOR_DZ_RUN_R);

	App_PWM_Set_L(motor_duty_L);
	App_PWM_Set_R(motor_duty_R);
}
