/* bsp_si2c.c - 软件（位拆）I2C 主机实现
 *
 * 依据：NXP UM10204《I2C-bus specification and user manual》
 *       ST RM0008 第 9 章（GPIO）：开漏输出模式下，输出寄存器写 0 拉低，
 *       写 1 则引脚浮空，电平由外部上拉决定 —— 这正是 I2C 需要的线与特性。
 *
 * 时序做法：每一个电平跳变后面都跟一次明确的半周期延时，
 *   保证 SCL 高低电平宽度可控，不依赖 GPIO 写入本身耗时。
 *
 * 相比旧实现修正的问题：
 *   1. My_SI2C_ReceiveBytes 里「最后一个字节回 ACK、中间回 NACK」把应答给反了
 *      （规范的约定是：中间字节 ACK、最后一个字节 NACK）。
 *      RegReadBytes 里又是对的，同一个文件里两套相反的逻辑。
 *      现在统一成显式常量 SI2C_ACK / SI2C_NACK，不再用 0/1 裸值。
 *   2. 时钟只在部分边沿有延时，低电平半周期靠指令执行时间凑，不可控。
 *   3. My_SI2C_Init 先把数据寄存器写 1 再配模式，且 SCL/SDA 的时钟使能写了两遍。
 *      现在先配成开漏输出、放高，再使能时钟的做法被改成「先使能时钟 → 写输出寄存器 →
 *      配开漏」，全程不会出现毛刺（详见下面 Init 的注释）。
 *   4. 半周期延时是写死的魔数，现在放到 SI2C_TypeDef.HalfPeriodUs 里可配。
 */

#include "bsp_si2c.h"
#include "bsp_delay.h"

/* ---------------------------------------------------------------------------
 * 引脚操作
 * I2C 的线与特性要求「输出 0 = 拉低，输出 1 = 释放」，
 * 开漏输出配外部上拉正好实现这一点
 * ------------------------------------------------------------------------- */

static void SCL_Write(SI2C_TypeDef *SI2C, uint8_t Level)
{
	GPIO_WriteBit(SI2C->SCL_GPIOx, SI2C->SCL_GPIO_Pin,
	              (Level != 0u) ? Bit_SET : Bit_RESET);
}

static void SDA_Write(SI2C_TypeDef *SI2C, uint8_t Level)
{
	GPIO_WriteBit(SI2C->SDA_GPIOx, SI2C->SDA_GPIO_Pin,
	              (Level != 0u) ? Bit_SET : Bit_RESET);
}

static uint8_t SDA_Read(SI2C_TypeDef *SI2C)
{
	return (GPIO_ReadInputDataBit(SI2C->SDA_GPIOx, SI2C->SDA_GPIO_Pin) == Bit_SET) ? 1u : 0u;
}

/* 半周期延时。STM32F1 在 72MHz 下执行一条指令约 14ns，
 * 只有极短的半周期才需要靠空转凑，其余都走 DelayUs */
static void SI2C_HalfDelay(SI2C_TypeDef *SI2C)
{
	DelayUs(SI2C->HalfPeriodUs);
}

/* ---------------------------------------------------------------------------
 * 时钟延展（clock stretching）
 *
 * I2C 规范允许从机在需要时间处理时把 SCL 拉住不放，主机必须等它松开再继续。
 * **MPU6050 会这么做**（OLED 不会），所以接 MPU 时必须处理，否则会读出乱码
 * 或者干脆失败。
 *
 * 主机自己写 SCL 为高之后，回读引脚确认它真的变高了；被拉住就等，
 * 但等的时间有上限 —— 真被拉死时宁可让本次传输失败，也不能卡住整个控制环。
 * ------------------------------------------------------------------------- */

/* 松手后最多等多久（微秒）。从机拉 SCL 一般也就几微秒，给 2ms 足够宽松 */
#define SI2C_STRETCH_TIMEOUT_US   (2000u)

/* 把 SCL 放到 Level；若放的是高电平，则等到它真的变高（或超时）。
 * 返回 1 = 拿到了总线，0 = 超时（从机一直拉住不放） */
static uint8_t SI2C_ReleaseSCL(SI2C_TypeDef *SI2C, uint32_t TimeoutUs)
{
	uint64_t start;

	SCL_Write(SI2C, 1u);

	if(TimeoutUs == 0u)
	{
		/* 不检查的场合（回调里/极短脉冲）直接用 */
		return 1u;
	}

	start = GetUs();

	while((GPIO_ReadInputDataBit(SI2C->SCL_GPIOx, SI2C->SCL_GPIO_Pin) == Bit_RESET))
	{
		if((GetUs() - start) > (uint64_t)TimeoutUs)
		{
			return 0u;   /* 从机把 SCL 拉死了，本次传输作废 */
		}
	}

	return 1u;
}

/* ---------------------------------------------------------------------------
 * 基本信号
 * ------------------------------------------------------------------------- */

/* 起始条件：SCL 为高时，SDA 由高变低 */
static void SI2C_Start(SI2C_TypeDef *SI2C)
{
	SDA_Write(SI2C, 1u);
	SCL_Write(SI2C, 1u);
	SI2C_HalfDelay(SI2C);

	SDA_Write(SI2C, 0u);
	SI2C_HalfDelay(SI2C);

	SCL_Write(SI2C, 0u);   /* 拉低 SCL，准备发位 */
	SI2C_HalfDelay(SI2C);
}

/* 停止条件：SCL 为高时，SDA 由低变高 */
static void SI2C_Stop(SI2C_TypeDef *SI2C)
{
	SDA_Write(SI2C, 0u);
	SCL_Write(SI2C, 0u);
	SI2C_HalfDelay(SI2C);

	SCL_Write(SI2C, 1u);
	SI2C_HalfDelay(SI2C);

	SDA_Write(SI2C, 1u);   /* SCL 高时 SDA 上升 = 停止 */
	SI2C_HalfDelay(SI2C);
}

/* 重复起始条件。与起始条件的区别是：进入时 SCL 可能还是低的，
 * 要先确保两线都释放、SCL 拉高，再走一次起始 */
static void SI2C_RepeatStart(SI2C_TypeDef *SI2C)
{
	SDA_Write(SI2C, 1u);
	SCL_Write(SI2C, 0u);
	SI2C_HalfDelay(SI2C);

	SCL_Write(SI2C, 1u);
	SI2C_HalfDelay(SI2C);

	SDA_Write(SI2C, 0u);
	SI2C_HalfDelay(SI2C);

	SCL_Write(SI2C, 0u);
	SI2C_HalfDelay(SI2C);
}

/* 发一个字节，返回从机的应答：SI2C_ACK(1) = 收到应答，SI2C_NACK(0) = 没人应答。
 * 高位在前（MSB first，I2C 规范）。
 * 返回值：SI2C_ACK / SI2C_NACK / SI2C_BUS_STUCK（从机拉住 SCL 不放） */
static uint8_t SI2C_WriteByte(SI2C_TypeDef *SI2C, uint8_t Byte)
{
	int8_t  i;
	uint8_t ack;

	for(i = 7; i >= 0; i--)
	{
		/* SCL 为低时才允许改变 SDA */
		SCL_Write(SI2C, 0u);
		SDA_Write(SI2C, (uint8_t)((Byte >> i) & 0x01u));
		SI2C_HalfDelay(SI2C);

		/* 从机在 SCL 高电平期间采样，所以这里要确认高电平真的建立起来 */
		if(SI2C_ReleaseSCL(SI2C, SI2C_STRETCH_TIMEOUT_US) == 0u)
		{
			return SI2C_BUS_STUCK;
		}

		SI2C_HalfDelay(SI2C);
	}

	/* 第 9 个时钟：主机释放 SDA，读从机拉成什么电平 */
	SCL_Write(SI2C, 0u);
	SDA_Write(SI2C, 1u);       /* 释放 SDA，交给从机 */
	SI2C_HalfDelay(SI2C);

	if(SI2C_ReleaseSCL(SI2C, SI2C_STRETCH_TIMEOUT_US) == 0u)
	{
		return SI2C_BUS_STUCK;
	}

	SI2C_HalfDelay(SI2C);

	ack = (SDA_Read(SI2C) == 0u) ? SI2C_ACK : SI2C_NACK;

	SCL_Write(SI2C, 0u);
	SI2C_HalfDelay(SI2C);

	return ack;
}

/* 读一个字节。Ack 传 SI2C_ACK 表示「还要继续读」，SI2C_NACK 表示「这是最后一个」。
 * 时钟延展超时时返回 SI2C_BUS_STUCK 需要的信息由调用方通过 *pOk 拿到 */
static uint8_t SI2C_ReadByte(SI2C_TypeDef *SI2C, uint8_t Ack, uint8_t *pOk)
{
	int8_t  i;
	uint8_t value = 0u;

	if(pOk != 0)
	{
		*pOk = 1u;
	}

	SCL_Write(SI2C, 0u);
	SDA_Write(SI2C, 1u);       /* 主机必须释放 SDA，否则读到的是自己 */
	SI2C_HalfDelay(SI2C);

	for(i = 7; i >= 0; i--)
	{
		/* 拉高 SCL，从机把数据位摆上来。要等 SCL 真的变高再采样 */
		if(SI2C_ReleaseSCL(SI2C, SI2C_STRETCH_TIMEOUT_US) == 0u)
		{
			if(pOk != 0) { *pOk = 0u; }
			return 0xFFu;
		}

		SI2C_HalfDelay(SI2C);

		if(SDA_Read(SI2C) != 0u)
		{
			value |= (uint8_t)(0x01u << i);
		}

		SCL_Write(SI2C, 0u);
		SI2C_HalfDelay(SI2C);
	}

	/* 第 9 个时钟：主机回 ACK / NACK */
	SDA_Write(SI2C, (Ack != 0u) ? 0u : 1u);   /* ACK = 拉低 SDA */
	SI2C_HalfDelay(SI2C);

	if(SI2C_ReleaseSCL(SI2C, SI2C_STRETCH_TIMEOUT_US) == 0u)
	{
		if(pOk != 0) { *pOk = 0u; }
		return 0xFFu;
	}

	SI2C_HalfDelay(SI2C);

	SCL_Write(SI2C, 0u);
	SI2C_HalfDelay(SI2C);

	SDA_Write(SI2C, 1u);       /* 释放 SDA，恢复空闲 */

	return value;
}

/* 强制补一个停止位，把总线拉回空闲。
 * 不判应答、不检查 SCL，所以不会阻塞，中断里也能调 */
void SI2C_ForceStop(SI2C_TypeDef *SI2C)
{
	SCL_Write(SI2C, 0u);
	SDA_Write(SI2C, 0u);
	SI2C_HalfDelay(SI2C);

	SCL_Write(SI2C, 1u);
	SI2C_HalfDelay(SI2C);

	SDA_Write(SI2C, 1u);   /* SCL 高时 SDA 上升 = 停止位 */
	SI2C_HalfDelay(SI2C);
}

/* ---------------------------------------------------------------------------
 * 端口时钟使能
 * ------------------------------------------------------------------------- */

static void SI2C_EnablePortClock(GPIO_TypeDef *GPIOx)
{
	if(GPIOx == GPIOA)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	}
	else if(GPIOx == GPIOB)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	}
	else if(GPIOx == GPIOC)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	}
	else if(GPIOx == GPIOD)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	}
	else if(GPIOx == GPIOE)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	}
}

/* ---------------------------------------------------------------------------
 * 对外接口
 * ------------------------------------------------------------------------- */

void My_SI2C_Init(SI2C_TypeDef *SI2C)
{
	GPIO_InitTypeDef gpio;

	if(SI2C == 0)
	{
		return;
	}

	/* 半周期为 0 会退化成没有延时，总线速率不可控。给个保守的默认值 */
	if(SI2C->HalfPeriodUs == 0u)
	{
		SI2C->HalfPeriodUs = 2u;
	}

	/* 顺序很重要：
	 *   先使能端口时钟（不然写寄存器无效）
	 *   → 再把输出寄存器写成 1（释放两线，避免配置瞬间把线拉低）
	 *   → 最后配成开漏输出
	 * 旧实现是「先写 1 再使能时钟」，时钟还没开的那次写入是无效的，
	 * 配成开漏输出的瞬间可能产生一个低电平毛刺，被从机误当成起始条件 */
	SI2C_EnablePortClock(SI2C->SCL_GPIOx);
	SI2C_EnablePortClock(SI2C->SDA_GPIOx);

	GPIO_WriteBit(SI2C->SCL_GPIOx, SI2C->SCL_GPIO_Pin, Bit_SET);
	GPIO_WriteBit(SI2C->SDA_GPIOx, SI2C->SDA_GPIO_Pin, Bit_SET);

	/* 50MHz 输出速率：软 I2C 只需要几百 kHz，给最高档是为了让边沿更陡、
	 * 时序更接近理论值；开漏下真正决定上升沿的是外部上拉电阻 */
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode  = GPIO_Mode_Out_OD;

	gpio.GPIO_Pin = SI2C->SCL_GPIO_Pin;
	GPIO_Init(SI2C->SCL_GPIOx, &gpio);

	gpio.GPIO_Pin = SI2C->SDA_GPIO_Pin;
	GPIO_Init(SI2C->SDA_GPIOx, &gpio);

	/* 让总线进入明确的空闲态 */
	SDA_Write(SI2C, 1u);
	SCL_Write(SI2C, 1u);
	SI2C_HalfDelay(SI2C);
}

int My_SI2C_SendBytes(SI2C_TypeDef *SI2C, uint8_t Addr, const uint8_t *pData, uint16_t Size)
{
	uint16_t i;
	uint8_t  ack;

	if((pData == 0) && (Size > 0u))
	{
		return -2;
	}

	SI2C_Start(SI2C);

	/* 地址左对齐，最低位是读写方向位：0 = 写 */
	ack = SI2C_WriteByte(SI2C, (uint8_t)(Addr & 0xFEu));

	if(ack != SI2C_ACK)
	{
		SI2C_Stop(SI2C);
		return (ack == SI2C_BUS_STUCK) ? -3 : -1;
	}

	for(i = 0u; i < Size; i++)
	{
		ack = SI2C_WriteByte(SI2C, pData[i]);

		if(ack != SI2C_ACK)
		{
			SI2C_Stop(SI2C);
			return (ack == SI2C_BUS_STUCK) ? -3 : -2;
		}
	}

	SI2C_Stop(SI2C);

	return 0;
}

int My_SI2C_ReceiveBytes(SI2C_TypeDef *SI2C, uint8_t Addr, uint8_t *pBuffer, uint16_t Size)
{
	uint16_t i;
	uint8_t  ack;
	uint8_t  ok = 1u;

	if((pBuffer == 0) || (Size == 0u))
	{
		return 0;
	}

	SI2C_Start(SI2C);

	/* 地址左对齐，最低位是读写方向位：1 = 读 */
	ack = SI2C_WriteByte(SI2C, (uint8_t)(Addr | 0x01u));

	if(ack != SI2C_ACK)
	{
		SI2C_Stop(SI2C);
		return (ack == SI2C_BUS_STUCK) ? -3 : -1;
	}

	for(i = 0u; i < Size; i++)
	{
		/* 规范要求：除最后一个字节外都回 ACK，最后一个回 NACK，
		 * 从机看到 NACK 才知道主机读完了、可以释放总线 */
		pBuffer[i] = SI2C_ReadByte(SI2C,
		                           (i == (uint16_t)(Size - 1u)) ? SI2C_NACK : SI2C_ACK,
		                           &ok);

		if(ok == 0u)
		{
			SI2C_Stop(SI2C);
			return -3;   /* 从机把 SCL 拉死了 */
		}
	}

	SI2C_Stop(SI2C);

	return 0;
}

int My_SI2C_RegReadBytes(SI2C_TypeDef *SI2C, uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Size)
{
	uint16_t i;
	uint8_t  ack;
	uint8_t  ok = 1u;

	if((pBuffer == 0) || (Size == 0u))
	{
		return 0;
	}

	/* #1 起始 + 地址（写方向） */
	SI2C_Start(SI2C);

	ack = SI2C_WriteByte(SI2C, (uint8_t)(Addr & 0xFEu));

	if(ack != SI2C_ACK)
	{
		SI2C_Stop(SI2C);
		return (ack == SI2C_BUS_STUCK) ? -3 : -1;
	}

	/* #2 把要读的寄存器号告诉从机 */
	ack = SI2C_WriteByte(SI2C, Reg);

	if(ack != SI2C_ACK)
	{
		SI2C_Stop(SI2C);
		return (ack == SI2C_BUS_STUCK) ? -3 : -2;
	}

	/* #3 重复起始 + 地址（读方向），中间不能有停止位 */
	SI2C_RepeatStart(SI2C);

	ack = SI2C_WriteByte(SI2C, (uint8_t)(Addr | 0x01u));

	if(ack != SI2C_ACK)
	{
		SI2C_Stop(SI2C);
		return (ack == SI2C_BUS_STUCK) ? -3 : -1;
	}

	/* #4 读数据 */
	for(i = 0u; i < Size; i++)
	{
		pBuffer[i] = SI2C_ReadByte(SI2C,
		                           (i == (uint16_t)(Size - 1u)) ? SI2C_NACK : SI2C_ACK,
		                           &ok);

		if(ok == 0u)
		{
			SI2C_Stop(SI2C);
			return -3;
		}
	}

	SI2C_Stop(SI2C);

	return 0;
}

int My_SI2C_RegWriteBytes(SI2C_TypeDef *SI2C, uint8_t Addr, uint8_t Reg, const uint8_t *pData, uint16_t Size)
{
	uint16_t i;
	uint8_t  ack;

	if((pData == 0) && (Size > 0u))
	{
		return -2;
	}

	SI2C_Start(SI2C);

	ack = SI2C_WriteByte(SI2C, (uint8_t)(Addr & 0xFEu));

	if(ack != SI2C_ACK)
	{
		SI2C_Stop(SI2C);
		return (ack == SI2C_BUS_STUCK) ? -3 : -1;
	}

	ack = SI2C_WriteByte(SI2C, Reg);

	if(ack != SI2C_ACK)
	{
		SI2C_Stop(SI2C);
		return (ack == SI2C_BUS_STUCK) ? -3 : -2;
	}

	for(i = 0u; i < Size; i++)
	{
		ack = SI2C_WriteByte(SI2C, pData[i]);

		if(ack != SI2C_ACK)
		{
			SI2C_Stop(SI2C);
			return (ack == SI2C_BUS_STUCK) ? -3 : -2;
		}
	}

	SI2C_Stop(SI2C);

	return 0;
}
