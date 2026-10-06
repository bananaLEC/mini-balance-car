/* bsp_delay.c - 基于 SysTick 的延时与计时实现
 *
 * 依据：
 *   - ARM DDI 0337《Cortex-M3 Technical Reference Manual》第 8 章 SysTick：
 *       LOAD  = 自动重装值
 *       VAL   = 当前计数值，**递减**，从 LOAD 往下数到 0
 *       CTRL  = ENABLE / TICKINT / CLKSOURCE
 *       计数到 0 时硬件自动重装 LOAD，并在下一拍拉高 COUNTFLAG
 *   - CMSIS core_cm3.h：SysTick 结构体、NVIC_SetPriority
 *   - ST DS5319：STM32F103C8T6 主频上限 72MHz
 *
 * 时间轴设计（全部来自 SysTick 一个时基，不依赖任何其他计数器）：
 *   毫秒部分 = delay_ticks，由 SysTick 中断递增，写入者唯一
 *   微秒部分 = 毫秒部分 * 1000 + 本毫秒内已过的微秒数
 *             其中「本毫秒内已过的微秒数」直接由 (LOAD - VAL) 换算
 *
 * 为什么这样组合是自洽的：
 *   SysTick 重装（VAL 跳回 LOAD）与中断里 delay_ticks++ 是同一事件的两面。
 *   所以「先读 delay_ticks 再读 VAL」无论中间是否发生重装，结果都一致 ——
 *   要么读到旧毫秒值配接近 LOAD 的 VAL，要么读到新毫秒值配刚重装的 VAL，
 *   两者算出来的绝对微秒数相同。不需要重试循环，也不会跳变。
 *
 * 为什么之前用 DWT 是错的（本次修正）：
 *   原实现取 `base = CYCCNT; ticks = delay_ticks; now = CYCCNT;`，
 *   然后拿 (now - base) 当亚毫秒部分 —— 可 (now - base) 只是
 *   「读一次时间戳本身」耗掉的几十个周期，除以 72 之后截断成 0。
 *   结果 GetUs() 实际只有 1ms 分辨率，DelayUs(1) 会退化成睡到下个毫秒边界。
 *
 * 已知边界：
 *   - 毫秒计数依赖 SysTick 中断。若调用方长时间 __disable_irq()（超过 1ms），
 *     这段时间不会被计入。本项目里唯一的临界区在 app_encoder.c，
 *     只包住一次结构体拷贝，远小于 1ms；bsp_i2c 已不再使用临界区。
 *   - 不要在中断里调用 Delay / DelayUs（会死等）。
 */

#include "bsp_delay.h"

#define DELAY_SYSTICK_LOAD_MAX   (0x00FFFFFFu)

/* 毫秒时间戳。只由 Delay_TickInc()（即 SysTick 中断）写入 */
static volatile uint32_t delay_ticks;

/* SysTick 自动重装值，换算用。在 Delay_Init 里赋值后不再改动 */
static uint32_t systick_reload;

/* 每毫秒的 SysTick 周期数（= 时钟频率 / 1000），换算用 */
static uint32_t systick_cycles_per_ms;

/* 初始化是否完成。放在最后一步置位，避免初始化中途被重入
 * 时用到还没算好的除数 */
static uint8_t delay_init_done;

/* 本毫秒内已经过去的微秒数。
 * SysTick 递减计数，所以已过周期数 = LOAD - VAL。
 * 因为 VAL 的取值范围是 [0, LOAD]，减法在 32 位无符号下天然处理「重装瞬间」 */
static uint32_t Delay_SubMsUs(void)
{
	uint32_t elapsed_cycles = systick_reload - SysTick->VAL;

	if(systick_cycles_per_ms == 0u)
	{
		return 0u;
	}

	/* 可达到的最大微秒数就是 1000，不会溢出 */
	return (elapsed_cycles * 1000u) / systick_cycles_per_ms;
}

void Delay_Init(void)
{
	RCC_ClocksTypeDef clocks;
	uint32_t reload;

	if(delay_init_done != 0u)
	{
		return;
	}

	/* 取实际 HCLK。SystemCoreClock 由 SystemInit() 设好，
	 * 但以寄存器读出来的为准，避免工程改启动流程后两者不一致 */
	RCC_GetClocksFreq(&clocks);

	if(clocks.HCLK_Frequency == 0u)
	{
		clocks.HCLK_Frequency = SystemCoreClock;
	}

	/* 1ms 一拍的自动重装值。72MHz 下为 71999，远小于 24 位上限 */
	reload = clocks.HCLK_Frequency / 1000u;

	if(reload == 0u)
	{
		reload = 1u;
	}
	else if(reload > DELAY_SYSTICK_LOAD_MAX)
	{
		/* HCLK 超过 16777MHz 才会走到这里，实际不可能；保底防止写出非法值 */
		reload = DELAY_SYSTICK_LOAD_MAX;
	}

	systick_reload        = reload;
	systick_cycles_per_ms = reload;

	SysTick->CTRL &= ~SysTick_CTRL_ENABLE;   /* 配置期间先停表 */
	SysTick->LOAD  = reload - 1u;
	SysTick->VAL   = 0u;                     /* 清当前值与 COUNTFLAG */

	/* 优先级取 1，把 0 留给对时序最敏感的中断。
	 * SysTick 每 1ms 进一次中断，优先级给太高会让编码器、串口中断排队 */
	NVIC_SetPriority(SysTick_IRQn, 1u);

	SysTick->CTRL = SysTick_CTRL_CLKSOURCE |   /* 时钟源取 HCLK，不分频 */
	                SysTick_CTRL_TICKINT   |   /* 计数到 0 产生中断 */
	                SysTick_CTRL_ENABLE;

	delay_init_done = 1u;
}

/* SysTick 中断里调用：毫秒时间戳 +1 */
void Delay_TickInc(void)
{
	delay_ticks++;
}

uint32_t GetTick(void)
{
	if(delay_init_done == 0u)
	{
		Delay_Init();
	}

	return delay_ticks;
}

uint64_t GetUs(void)
{
	if(delay_init_done == 0u)
	{
		Delay_Init();
	}

	/* 先读毫秒戳，再读同一毫秒内的余量。
	 *
	 * 顺序不能反：若先读余量再读毫秒戳，一旦两者之间发生重装，就会把
	 * 「刚重装的小余量」和「已经 +1 的新毫秒戳」配在一起，算出来比真实时间
	 * **超前**最多 1ms。先读毫秒戳的话，最坏情况是「旧毫秒戳 + 刚重装的小余量」，
	 * 结果是**滞后**不超过 1ms —— 同一个量级，但这个方向对增量式算法更安全
	 * （滞后只会让这一次的差值偏小、下一次补回来，不会凭空跳出一个大差值）。 */
	return ((uint64_t)delay_ticks * 1000u) + (uint64_t)Delay_SubMsUs();
}

void Delay(uint32_t ms)
{
	uint32_t start;

	if(delay_init_done == 0u)
	{
		Delay_Init();
	}

	start = delay_ticks;

	/* 无符号差值比较，天然跨 49.7 天回绕。
	 * 毫秒计数的推进靠 SysTick 中断，所以本函数不能在中断里调用 */
	while((uint32_t)(delay_ticks - start) < ms)
	{
		/* 空等 */
	}
}

void DelayUs(uint32_t us)
{
	uint32_t start;

	if(delay_init_done == 0u)
	{
		Delay_Init();
	}

	start = (uint32_t)GetUs();

	/* 无符号差值比较，天然处理 32 位回绕。
	 *
	 * 这里**故意不换算成 SysTick 周期数**：那需要把时间戳的差值限幅在
	 * 「一个毫秒周期」以内，而延时本身可以跨很多毫秒（软 I2C 一次半周期
	 * 只有几微秒，但将来别的用途可能要等几百微秒甚至更久）。
	 * 一旦限幅，长延时的循环条件就永远成立了 —— 那是个死循环。
	 * 直接用微秒差值则没有任何上界限制。 */
	while((uint32_t)((uint32_t)GetUs() - start) < us)
	{
		/* 空等 */
	}
}
