#include "app_encoder.h"
#include "bsp_delay.h"

/*
 * 编码器引脚：左 M1 = PA6(A)/PA7(B)，右 M2 = PB6(A)/PB7(B)
 *
 * PB6 与 PA6 同属 EXTI6，一个端口只能映射一次，故右编码器改由 B 相 PB7 触发
 * （EXTI7），再读 A 相 PB6 判向。
 *
 * 解码（各相双边沿触发，读另一相电平判向）：
 *   左 A 相触发：A↑+B低 ++，A↑+B高 --，A↓+B低 --，A↓+B高 ++
 *   右 B 相触发：B↑+A高 ++，B↑+A低 --，B↓+A低 ++，B↓+A高 --
 */

/* 左编码器：A=PA6(EXTI6)，B=PA7(输入读取) */
#define ENC_L_A_PORT        GPIOA
#define ENC_L_A_PIN         GPIO_Pin_6
#define ENC_L_A_PIN_SRC     GPIO_PinSource6
#define ENC_L_B_PORT        GPIOA
#define ENC_L_B_PIN         GPIO_Pin_7

/* 右编码器：A=PB6(输入读取)，B=PB7(EXTI7) */
#define ENC_R_A_PORT        GPIOB
#define ENC_R_A_PIN         GPIO_Pin_6
#define ENC_R_B_PORT        GPIOB
#define ENC_R_B_PIN         GPIO_Pin_7
#define ENC_R_B_PIN_SRC     GPIO_PinSource7

/* 单相双边沿计数：编码器 11 线，11*2 = 22 计数/圈 */
#define ENC_COUNTS_PER_REV  22.0f
/* 减速比 22/12 * 22/10 * 23/10 = 2783/300 ≈ 9.27666 */
#define ENC_GEAR_RATIO      (2783.0f / 300.0f)
/* 一个计数对应的输出轴角度，≈1.764 度/计数 */
#define ENC_DEG_PER_COUNT   (360.0f / (ENC_COUNTS_PER_REV * ENC_GEAR_RATIO))

/* T 法测速参数：ENC_FILTER_N 用最近 N 个边沿的间隔一起求平均，越大越平滑但跟随越慢；
 * ENC_TIMEOUT_US 内无新边沿即认为停转、速度归零，它决定低速下限（200ms ≈ 8.8 度/s） */
#define ENC_FILTER_N        4u
#define ENC_TIMEOUT_US      200000u

/* 【假边沿抑制】两次边沿之间的最小间隔，单位 us。
 * EXTI 计数无数字滤波，电机开关噪声会制造假边沿让读数飞掉：实测见过位置一帧跳 1275 度
 * （723 个计数）、速度 350 rad/s（真实约 2 rad/s）。
 * 取 350us 的依据：跑车时轮速最高约 39 rad/s，按 1.764 度/计数折合边沿间隔约 770us，
 * 350us 留有 2.2 倍余量，真边沿不会被误杀，几十微秒一条的噪声边沿全部丢弃。
 * 只靠这一道不够，调速环还有变化率检查（见 app_motor.c 的 MOTOR_SPD_JUMP_MAX）。 */
#define ENC_MIN_EDGE_US     350u

/* 【速度读数上限】单位 度/s，超过即截断。100 rad/s ≈ 5729 度/s ≈ 3.3 m/s，
 * 远高于电机实际能达到的转速，不会截断真读数 */
#define ENC_SPEED_MAX       5729.0f

/* 一个编码器的全部状态，左右各一份 */
typedef struct
{
	volatile int64_t  count;                  /* 累计计数 */
	volatile int8_t   dir;                    /* 最近一次计数方向：+1 / -1 */
	volatile uint64_t t_last;                 /* 最近一次边沿的时刻(us) */
	volatile uint64_t t_edge[ENC_FILTER_N];   /* 最近 N 次边沿的时刻(us)，环形缓冲 */
	volatile uint8_t  head;                   /* 环形缓冲的写指针 */
	volatile uint8_t  filled;                 /* 已记录的有效边沿数 */
} Encoder_t;

static Encoder_t enc_l;
static Encoder_t enc_r;

static void Encoder_L_Init(void);
static void Encoder_R_Init(void);

/* 中断里每检测到一个有效边沿就调用一次：更新计数、方向和时间戳 */
static void Encoder_OnEdge(Encoder_t *e, int8_t inc)
{
	uint64_t now = GetUs();

	/* 距上次采纳的边沿太近，判为干扰丢弃；不更新时间戳和时间窗，免得假边沿成为下次判断基准 */
	if(e->filled > 0u && (now - e->t_last) < ENC_MIN_EDGE_US)
	{
		return;
	}

	e->count += inc;

	/* 方向刚变化：旧间隔跨越了换向过程，继续平均会在换向处拉出一段水平线，清空重来 */
	if (inc != e->dir && e->filled > 0u)
	{
		e->filled = 0u;
		e->head = 0u;
	}
	e->dir = inc;

	e->t_edge[e->head] = now;
	e->head++;
	if (e->head >= ENC_FILTER_N)
	{
		e->head = 0u;
	}
	if (e->filled < ENC_FILTER_N)
	{
		e->filled++;
	}

	e->t_last = now;
}

/* 读取累计计数（64 位读不是原子操作，中断里可能正在改，需要临界区） */
static int64_t Encoder_GetCount(const Encoder_t *e)
{
	int64_t  count;
	uint32_t primask;

	/* 保存并恢复中断状态，而不是简单地 __disable_irq()/__enable_irq()。
	 * 后者在「调用时本来就关着中断」的情况下会擅自把中断打开 ——
	 * 本文件当前的调用点都在主循环里（中断是开的），所以暂时不会出问题，
	 * 但这类函数很容易在中断上下文里被再次调用，届时就是个隐蔽的坑 */
	primask = __get_PRIMASK();
	__disable_irq();

	count = e->count;

	__set_PRIMASK(primask);

	return count;
}

/* T 法测速：由最近 N 个边沿的时刻算出平均边沿间隔，再换算成角速度 */
static float Encoder_GetSpeed(const Encoder_t *e)
{
	Encoder_t snap;
	uint64_t now;
	uint64_t span;
	uint32_t us_per_edge;
	uint8_t i;
	uint8_t n;
	uint8_t newest;
	uint8_t oldest;
	float spd;
	uint32_t primask;

	now = GetUs();   /* 先取时间：GetUs 的时间轴来自 SysTick 中断，不能放在临界区里 */

	/* 临界区内一次性复制状态，避免读到中断正在更新的半成品，这是毛刺来源之一。
	 * 用 PRIMASK 保存/恢复：调用方本来就关着中断时不会把中断擅自打开 */
	primask = __get_PRIMASK();
	__disable_irq();

	snap.dir    = e->dir;
	snap.t_last = e->t_last;
	snap.head   = e->head;
	snap.filled = e->filled;
	for (i = 0u; i < ENC_FILTER_N; i++)
	{
		snap.t_edge[i] = e->t_edge[i];
	}

	__set_PRIMASK(primask);

	n = snap.filled;
	if (n < 2u)                              /* 历史不足两个边沿，算不出间隔 */
	{
		return 0.0f;
	}
	if (now - snap.t_last > ENC_TIMEOUT_US)  /* 长时间没有新边沿：轮子已经停了 */
	{
		return 0.0f;
	}

	newest = (uint8_t)((snap.head + ENC_FILTER_N - 1u) % ENC_FILTER_N);
	oldest = (uint8_t)((snap.head + ENC_FILTER_N - n) % ENC_FILTER_N);

	span = snap.t_edge[newest] - snap.t_edge[oldest];   /* n-1 个间隔的总时长 */
	us_per_edge = (uint32_t)(span / (uint64_t)(n - 1u));

	if (us_per_edge == 0u)                   /* 两次边沿落在同一微秒，防止除零 */
	{
		return 0.0f;
	}

	/* 角速度 = 每个计数对应的角度 / 每个边沿耗费的时间 */
	spd = (float)snap.dir * ENC_DEG_PER_COUNT * 1000000.0f / (float)us_per_edge;

	/* 合理性截断：漏网的假边沿会让 us_per_edge 极小，速度算到几百 rad/s 并把调速环带飞 */
	if(spd > ENC_SPEED_MAX)       { spd = ENC_SPEED_MAX; }
	else if(spd < -ENC_SPEED_MAX) { spd = -ENC_SPEED_MAX; }

	return spd;
}

// 对编码器模块初始化
void App_Encoder_Init(void)
{
	Encoder_L_Init();
	Encoder_R_Init();
}

// 读取左轮胎旋转的角度（度）
float App_Encoder_GetPos_L(void)
{
	return (float)Encoder_GetCount(&enc_l) * ENC_DEG_PER_COUNT;
}

// 读取右轮胎旋转的角度（度）
float App_Encoder_GetPos_R(void)
{
	return (float)Encoder_GetCount(&enc_r) * ENC_DEG_PER_COUNT;
}

// 读取左轮胎旋转的角速度，单位：度/s
float App_Encoder_GetSpeed_L(void)
{
	return Encoder_GetSpeed(&enc_l);
}

// 读取右轮胎旋转的角速度，单位：度/s
float App_Encoder_GetSpeed_R(void)
{
	return Encoder_GetSpeed(&enc_r);
}

// 左编码器的初始化：PA6(A, EXTI双边沿) + PA7(B, 上拉输入)
static void Encoder_L_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	EXTI_InitTypeDef EXTI_InitStruct = {0};
	NVIC_InitTypeDef NVIC_InitStruct = {0};

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	// PA6、PA7 均配置为上拉输入
	GPIO_InitStruct.GPIO_Pin = ENC_L_A_PIN | ENC_L_B_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(ENC_L_A_PORT, &GPIO_InitStruct);

	// EXTI Line6 映射到 PA6
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, ENC_L_A_PIN_SRC);

	EXTI_InitStruct.EXTI_Line = EXTI_Line6;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStruct);

	// EXTI9_5 中断（Line5~Line9 共用），优先级分组在 main 中设置
	// 抢占优先级取 2（数值越小越优先），0/1 留给以后的控制周期中断，使控制环能打断编码器计数
	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
}

// 右编码器的初始化：PB6(A, 上拉输入) + PB7(B, EXTI双边沿)
static void Encoder_R_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	EXTI_InitTypeDef EXTI_InitStruct = {0};
	NVIC_InitTypeDef NVIC_InitStruct = {0};

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	// PB6、PB7 均配置为上拉输入
	GPIO_InitStruct.GPIO_Pin = ENC_R_A_PIN | ENC_R_B_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(ENC_R_A_PORT, &GPIO_InitStruct);

	// EXTI Line7 映射到 PB7（右编码器 B 相）
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, ENC_R_B_PIN_SRC);

	EXTI_InitStruct.EXTI_Line = EXTI_Line7;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
}

// EXTI9_5 的中断响应函数：Line6=左 A 相，Line7=右 B 相
void EXTI9_5_IRQHandler(void)
{
	// 左编码器 A 相（PA6）
	if (EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line6);

		uint8_t a = GPIO_ReadInputDataBit(ENC_L_A_PORT, ENC_L_A_PIN); // A 相当前电平
		uint8_t b = GPIO_ReadInputDataBit(ENC_L_B_PORT, ENC_L_B_PIN); // B 相当前电平

		int8_t inc;
		if (a == Bit_SET)          // A 上升沿
		{
			inc = (b == Bit_RESET) ? 1 : -1;
		}
		else                       // A 下降沿
		{
			inc = (b == Bit_RESET) ? -1 : 1;
		}
		Encoder_OnEdge(&enc_l, inc);
	}

	// 右编码器 B 相（PB7）
	if (EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line7);

		uint8_t a = GPIO_ReadInputDataBit(ENC_R_A_PORT, ENC_R_A_PIN); // A 相当前电平
		uint8_t b = GPIO_ReadInputDataBit(ENC_R_B_PORT, ENC_R_B_PIN); // B 相当前电平

		int8_t inc;
		if (b == Bit_SET)          // B 上升沿
		{
			inc = (a == Bit_SET) ? 1 : -1;
		}
		else                       // B 下降沿
		{
			inc = (a == Bit_RESET) ? 1 : -1;
		}
		//右电机实际接线方向与左电机相反，计数同步取反，保证前进时左右计数都增大
		Encoder_OnEdge(&enc_r, (int8_t)(-inc));
	}
}

void App_Encoder_ClearLeft(void)
{
	__disable_irq();
	enc_l.count = 0;
	enc_l.dir = 0;
	enc_l.filled = 0u;
	enc_l.head = 0u;
	__enable_irq();
}

void App_Encoder_ClearRight(void)
{
	__disable_irq();
	enc_r.count = 0;
	enc_r.dir = 0;
	enc_r.filled = 0u;
	enc_r.head = 0u;
	__enable_irq();
}

void App_Encoder_ClearAll(void)
{
	App_Encoder_ClearLeft();
	App_Encoder_ClearRight();
}
