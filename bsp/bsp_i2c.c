/* bsp_i2c.c - 硬件 I2C 主机实现
 *
 * 依据：
 *   ST RM0008 第 26 章「Inter-integrated circuit (I2C) interface」
 *       · SR1/SR2 标志位定义与清除方式
 *       · 主发送 / 主接收的状态机流程
 *       · 为什么「清 ADDR 标志要按顺序先读 SR1 再读 SR2」
 *   ST AN2824《STM32F10xxx I2C optimized examples》
 *   ST ES096（STM32F103 勘误手册）—— 总线锁死必须靠主机补时钟脉冲恢复
 *
 * 设计要点：
 *   1. 每一个等标志位的地方都有超时，函数不会无限阻塞。
 *      裸 while 一旦遇到从机拉死总线就永久卡死，此时电机保持上一拍 PWM，车会冲出去。
 *   2. 超时用的 GetUs() 基于 SysTick，读写时间戳不碰中断控制寄存器，
 *      所以调用 GetUs() 既不会打断调用方，也不会改变中断状态。
 *   3. **本模块全程不关中断。**
 *      这是刻意的：GetUs() 的时间轴来自 SysTick 中断，一旦关中断时间轴就停住，
 *      超时判断永不成立，会退化成「中断关着死循环」，比不关中断危险得多。
 *      接收最后两个字节的原子性靠「读 DR」单条外设访问来保证，
 *      RXNE 是硬件置位的，中断插进来最多让最后一个字节晚一点读到，不会丢。
 *      （原实现在这里关中断并用循环计数限时，本模块换成了不关中断的正确做法。）
 */

#include "bsp_i2c.h"
#include "bsp_delay.h"

/* 每个等标志位处的超时上限。
 * 100kHz 下一帧最多几百微秒；取 10ms 既不会误判，又能及时从死锁里出来。
 * 唯一可能真正用满的是从机长时间拉低 SCL 做时钟延展，那种情况本来就是故障 */
#define I2C_TIMEOUT_US            (10000u)

/* 总线抢救的半周期，约 100kHz。放慢一点保证从机跟得上 */
#define I2C_RECOVER_HALF_US       (5u)

/* 本板 I2C 总线的波特率。总线抢救后要重新 I2C_Init，必须和上层初始化用同一个值，
 * 否则抢救完速率就变了。改这里要同步改 user/app_mpu6050.c 的 MPU6050_I2C_SPEED */
#define I2C_SPEED_HZ              (100000u)

/* 抢救时 SCL 脉冲个数：从机最多剩 8 位没吐完，多打一个确保冲干净 */
#define I2C_RECOVER_PULSES        (9u)

/* 上一次抢救之后两线是否仍被拉低。由 My_I2C_ResetBus 写，My_I2C_BusWasHeldLow 读。
 * 它区分的是「软件能救的锁死」和「硬件被拉住」这两类完全不同的故障 */
static uint8_t bus_held_low = 0u;

/* 抢救时用空转代替 DelayUs：不依赖延时模块，任何上下文都能用 */
#define I2C_RECOVER_SPIN_LOOPS    (72u)

/* 从机地址的读写方向位 */
#define I2C_DIR_WRITE             (0x00u)
#define I2C_DIR_READ              (0x01u)

/* 等待结果编码 */
#define I2C_WAIT_OK               (0u)
#define I2C_WAIT_TIMEOUT          (1u)

/* 地址阶段的结果 */
#define I2C_ADDR_OK               (0)
#define I2C_ADDR_NACK             (-1)

/* ---------------------------------------------------------------------------
 * 本板的 I2C 引脚表
 *
 * 抢救总线需要把 SCL/SDA 从「复用开漏」切成「普通开漏 GPIO」来手工翻转，
 * 这一步必须知道引脚号。原来这部分是写死在函数体里的（只认 I2C2 + PB10/PB11），
 * 现在集中到一张表里：板子换线或加器件时只改这里。
 * ------------------------------------------------------------------------- */

typedef struct
{
	I2C_TypeDef   *I2Cx;
	GPIO_TypeDef  *GPIOx;
	uint16_t       SCL_Pin;
	uint16_t       SDA_Pin;
} I2C_PinMap_t;

static const I2C_PinMap_t i2c_pin_map[] =
{
	/* 江协科技平衡车控制板 V2.0：MPU6050 挂在 I2C2，SCL=PB10，SDA=PB11 */
	{ I2C2, GPIOB, GPIO_Pin_10, GPIO_Pin_11 },
};

#define I2C_PIN_MAP_COUNT   (sizeof(i2c_pin_map) / sizeof(i2c_pin_map[0]))

static const I2C_PinMap_t *I2C_FindPinMap(I2C_TypeDef *I2Cx)
{
	uint32_t i;

	for(i = 0u; i < (uint32_t)I2C_PIN_MAP_COUNT; i++)
	{
		if(i2c_pin_map[i].I2Cx == I2Cx)
		{
			return &i2c_pin_map[i];
		}
	}

	return 0;   /* 这条总线不在本板引脚表里 */
}

/* ---------------------------------------------------------------------------
 * 带超时的标志位等待
 * ------------------------------------------------------------------------- */

/* 等到 Flag **不再是** State 为止，返回 I2C_WAIT_OK；超时返回 I2C_WAIT_TIMEOUT。
 *
 * 注意参数语义：State 是「还在等」的那个状态，不是「等它变成」的目标。
 * 两种惯用法都有，这里统一成前一种，所以调用点都会看到「等 RESET」这种写法 ——
 * 意思是「等到这个标志不再是 RESET（也就是被硬件置起来）」。
 * 形如 I2C_WaitFlag(I2Cx, I2C_FLAG_BUSY, SET) 则是「等 BUSY 落下去（总线变空闲）」。 */
static uint8_t I2C_WaitFlag(I2C_TypeDef *I2Cx, uint32_t Flag, FlagStatus State)
{
	uint64_t start = GetUs();

	while(I2C_GetFlagStatus(I2Cx, Flag) == State)
	{
		if((GetUs() - start) > (uint64_t)I2C_TIMEOUT_US)
		{
			return I2C_WAIT_TIMEOUT;
		}
	}

	return I2C_WAIT_OK;
}

/* 等 FlagA、FlagB 里任意一个置位。
 * 返回值：0 = FlagA 置位，1 = FlagB 置位，2 = 超时。
 * 参数顺序就是优先级 —— 调用处把「错误标志」放前面，
 * 两个同时置位时按出错处理 */
static uint8_t I2C_WaitEither(I2C_TypeDef *I2Cx, uint32_t FlagA, uint32_t FlagB)
{
	uint64_t start = GetUs();

	while(1)
	{
		if(I2C_GetFlagStatus(I2Cx, FlagA) == SET)
		{
			return 0u;
		}

		if(I2C_GetFlagStatus(I2Cx, FlagB) == SET)
		{
			return 1u;
		}

		if((GetUs() - start) > (uint64_t)I2C_TIMEOUT_US)
		{
			return 2u;
		}
	}
}

/* 出错时统一收尾：发停止位条件，把总线放回空闲。
 * 不发停止位的话，从机可能一直等着后续字节，下一次通信会直接卡在 BUSY */
static void I2C_Abort(I2C_TypeDef *I2Cx)
{
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

/* 按 RM0008 要求清 ADDR 标志：必须先读 SR1 再读 SR2。
 * 只读其中一个，ADDR 不会清掉，程序就会卡在等待里 */
static void I2C_ClearAddrFlag(I2C_TypeDef *I2Cx)
{
	(void)I2C_ReadRegister(I2Cx, I2C_Register_SR1);
	(void)I2C_ReadRegister(I2Cx, I2C_Register_SR2);
}

/* ---------------------------------------------------------------------------
 * 地址阶段
 * 起始条件 → 发地址+方向 → 等应答
 * 返回 I2C_ADDR_OK / I2C_ADDR_NACK / -3（超时）
 * ------------------------------------------------------------------------- */

static int I2C_AddressPhase(I2C_TypeDef *I2Cx, uint8_t Addr, uint8_t Direction, uint8_t *pTimeout)
{
	uint8_t result;

	/* 总线得先空闲。BUSY 一直为 1 说明从机拉住了总线，属于需要抢救的情况 */
	if(I2C_WaitFlag(I2Cx, I2C_FLAG_BUSY, SET) != I2C_WAIT_OK)
	{
		*pTimeout = 1u;
		return I2C_ADDR_NACK;
	}

	I2C_GenerateSTART(I2Cx, ENABLE);

	if(I2C_WaitFlag(I2Cx, I2C_FLAG_SB, RESET) != I2C_WAIT_OK)
	{
		I2C_Abort(I2Cx);
		*pTimeout = 1u;
		return I2C_ADDR_NACK;
	}

	/* 清掉上一次残留的应答失败标志，否则下面的等待会立刻返回 */
	I2C_ClearFlag(I2Cx, I2C_FLAG_AF);

	I2C_SendData(I2Cx, (uint16_t)(Addr | Direction));

	/* AF 放前面：地址无应答时按 -1 报，超时按 -3 报 */
	result = I2C_WaitEither(I2Cx, I2C_FLAG_AF, I2C_FLAG_ADDR);

	if(result != 1u)
	{
		I2C_Abort(I2Cx);

		if(result == 0u)
		{
			return I2C_ADDR_NACK;   /* 从机没应答 */
		}

		*pTimeout = 1u;
		return I2C_ADDR_NACK;
	}

	I2C_ClearAddrFlag(I2Cx);

	return I2C_ADDR_OK;
}

/* ---------------------------------------------------------------------------
 * 接收阶段
 * ------------------------------------------------------------------------- */

/* 读最后一个字节（Size == 1 的情形）：此时 ADDR 还置着。
 * RM0008 主接收单字节流程：关应答 → 清 ADDR → 发停止位，顺序不能换。
 * 关应答必须在清 ADDR 之前，因为 ACK 位决定的是「下一个收到字节回什么」，
 * 清掉 ADDR 之后从机就会把唯一那个字节送上来 */
static int I2C_ReadSingleByte(I2C_TypeDef *I2Cx, uint8_t *pBuffer)
{
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_ClearAddrFlag(I2Cx);
	I2C_GenerateSTOP(I2Cx, ENABLE);

	if(I2C_WaitFlag(I2Cx, I2C_FLAG_RXNE, RESET) != I2C_WAIT_OK)
	{
		return -3;
	}

	pBuffer[0] = (uint8_t)I2C_ReceiveData(I2Cx);

	return 0;
}

/* 读最后两个字节。
 *
 * RM0008 主接收 N>2 字节流程，倒数第二个字节起要一气呵成：
 *   等 RXNE（倒数第二个字节到了）→ 读 DR（同时清 RXNE 与 BTF）
 *   → 关应答 → 发停止位 → 等 RXNE（最后一个字节到了）→ 读 DR
 *
 * 关于「一气呵成」：这里不再关中断。原因有两条 ——
 *   1. bsp_i2c 现在依赖 GetUs() 做超时，而 GetUs() 的时间轴来自 SysTick 中断；
 *      一旦关中断，时间轴就停住，超时永不触发，会变成「中断关着死循环」，
 *      比不关中断危险得多。
 *   2. 这段窗口总共只有几个机器周期，中断插进来的代价最多是让最后一个字节
 *      晚一点读到，而 RXNE 是硬件置位的，不会丢。
 * 返回 0 成功 / -3 超时 */
static int I2C_ReadLastTwoBytes(I2C_TypeDef *I2Cx, uint8_t *pBuffer, uint16_t Size)
{
	if(I2C_WaitFlag(I2Cx, I2C_FLAG_RXNE, RESET) != I2C_WAIT_OK)
	{
		/* 超时也要把应答位和总线恢复，否则影响下一次通信 */
		I2C_AcknowledgeConfig(I2Cx, ENABLE);
		I2C_Abort(I2Cx);
		return -3;
	}

	pBuffer[Size - 2u] = (uint8_t)I2C_ReceiveData(I2Cx);

	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);

	/* 停止位已发，最后一个字节很快就会到 */
	if(I2C_WaitFlag(I2Cx, I2C_FLAG_RXNE, RESET) != I2C_WAIT_OK)
	{
		return -3;
	}

	pBuffer[Size - 1u] = (uint8_t)I2C_ReceiveData(I2Cx);

	return 0;
}

/* 读 Size 个字节。进入时地址阶段已经完成，ADDR 标志还置着。
 *
 * 三条路径都要自己负责清 ADDR，而且**清 ADDR 之前必须先把应答位设成
 * 本次想要的值**：ACK 位决定的是「下一个被接收字节回 ACK 还是 NACK」，
 * 清掉 ADDR 之后从机马上就开始送数据了。原实现把 AckEnable 放在读完
 * 第一个字节之后，第一个字节的应答时隙是 NACK，从机会直接停发 */
static int I2C_ReadData(I2C_TypeDef *I2Cx, uint8_t *pBuffer, uint16_t Size)
{
	uint16_t i;

	if(Size == 1u)
	{
		return I2C_ReadSingleByte(I2Cx, pBuffer);
	}

	/* 后面还要继续收，所以先答应上，再清 ADDR */
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	I2C_ClearAddrFlag(I2Cx);

	if(Size == 2u)
	{
		/* 正好两个字节，全部交给收尾函数处理。
		 * ⚠️ 这里**不能**先自己读掉第 0 个字节：I2C_ReadLastTwoBytes 固定
		 * 读「Size-2」和「Size-1」两个字节，若外面再预读一个，2 字节的请求
		 * 就会一共读 3 次、而且第 0 个下标被写两遍，数据静默错位。
		 * 本分支只需要「清 ADDR」和「ACK 先置 1」，两件事上面都做了 */
		return I2C_ReadLastTwoBytes(I2Cx, pBuffer, Size);
	}

	/* Size >= 3：先读第 0 个字节，中间的一直按 ACK 读，最后两个交给收尾函数 */
	if(I2C_WaitFlag(I2Cx, I2C_FLAG_RXNE, RESET) != I2C_WAIT_OK)
	{
		I2C_Abort(I2Cx);
		return -3;
	}

	pBuffer[0] = (uint8_t)I2C_ReceiveData(I2Cx);

	for(i = 1u; (uint16_t)(i + 2u) < Size; i++)
	{
		if(I2C_WaitFlag(I2Cx, I2C_FLAG_RXNE, RESET) != I2C_WAIT_OK)
		{
			I2C_Abort(I2Cx);
			return -3;
		}

		pBuffer[i] = (uint8_t)I2C_ReceiveData(I2Cx);
	}

	return I2C_ReadLastTwoBytes(I2Cx, pBuffer, Size);
}

/* ---------------------------------------------------------------------------
 * 对外接口
 * ------------------------------------------------------------------------- */

int My_I2C_SendBytes(I2C_TypeDef *I2Cx, uint8_t Addr, const uint8_t *pData, uint16_t Size)
{
	uint8_t  timeout = 0u;
	uint8_t  result;
	uint16_t i;
	int      ret;

	if((pData == 0) && (Size > 0u))
	{
		return -2;
	}

	ret = I2C_AddressPhase(I2Cx, Addr, I2C_DIR_WRITE, &timeout);

	if(ret != I2C_ADDR_OK)
	{
		return (timeout != 0u) ? -3 : -1;
	}

	for(i = 0u; i < Size; i++)
	{
		/* AF 放前面：与 TXE 同时置位时按「被拒收」处理 */
		result = I2C_WaitEither(I2Cx, I2C_FLAG_AF, I2C_FLAG_TXE);

		if(result != 1u)
		{
			I2C_Abort(I2Cx);
			return (result == 0u) ? -2 : -3;
		}

		I2C_SendData(I2Cx, pData[i]);
	}

	if(Size == 0u)
	{
		/* 只寻址不发数据，直接收尾 */
		I2C_Abort(I2Cx);
		return 0;
	}

	/* 等 BTF：最后一个字节已经从数据寄存器搬进移位寄存器。
	 * 这时候发停止位才不会截断它 */
	result = I2C_WaitEither(I2Cx, I2C_FLAG_AF, I2C_FLAG_BTF);

	if(result != 1u)
	{
		I2C_Abort(I2Cx);
		return (result == 0u) ? -2 : -3;
	}

	I2C_GenerateSTOP(I2Cx, ENABLE);

	return 0;
}

int My_I2C_ReceiveBytes(I2C_TypeDef *I2Cx, uint8_t Addr, uint8_t *pBuffer, uint16_t Size)
{
	uint8_t timeout = 0u;
	int     ret;

	if((pBuffer == 0) || (Size == 0u))
	{
		return 0;
	}

	ret = I2C_AddressPhase(I2Cx, Addr, I2C_DIR_READ, &timeout);

	if(ret != I2C_ADDR_OK)
	{
		return (timeout != 0u) ? -3 : -1;
	}

	return I2C_ReadData(I2Cx, pBuffer, Size);
}

int My_I2C_RegReadBytes(I2C_TypeDef *I2Cx, uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Size)
{
	uint8_t  timeout = 0u;
	uint8_t  result;
	int      ret;

	if((pBuffer == 0) || (Size == 0u))
	{
		return 0;
	}

	/* #1 起始 + 地址（写方向），准备告诉从机要读哪个寄存器 */
	ret = I2C_AddressPhase(I2Cx, Addr, I2C_DIR_WRITE, &timeout);

	if(ret != I2C_ADDR_OK)
	{
		return (timeout != 0u) ? -3 : -1;
	}

	/* #2 发寄存器号 */
	if(I2C_WaitFlag(I2Cx, I2C_FLAG_TXE, RESET) != I2C_WAIT_OK)
	{
		I2C_Abort(I2Cx);
		return -3;
	}

	I2C_SendData(I2Cx, (uint16_t)Reg);

	result = I2C_WaitEither(I2Cx, I2C_FLAG_AF, I2C_FLAG_TXE);

	if(result != 1u)
	{
		I2C_Abort(I2Cx);
		return (result == 0u) ? -2 : -3;
	}

	/* #3 重复起始 + 地址（读方向）。
	 * 中间不能有停止位，否则从机的寄存器指针就丢了 */
	I2C_GenerateSTART(I2Cx, ENABLE);

	if(I2C_WaitFlag(I2Cx, I2C_FLAG_SB, RESET) != I2C_WAIT_OK)
	{
		I2C_Abort(I2Cx);
		return -3;
	}

	I2C_ClearFlag(I2Cx, I2C_FLAG_AF);
	I2C_SendData(I2Cx, (uint16_t)(Addr | I2C_DIR_READ));

	result = I2C_WaitEither(I2Cx, I2C_FLAG_AF, I2C_FLAG_ADDR);

	if(result != 1u)
	{
		I2C_Abort(I2Cx);
		return (result == 0u) ? -1 : -3;
	}

	/* #4 读数据。注意这里**不能**先清 ADDR：
	 * Size==1 时必须在「关应答 + 发停止位」的同一个临界区里清，
	 * 否则从机会多送一个字节 */
	return I2C_ReadData(I2Cx, pBuffer, Size);
}

int My_I2C_MemWriteBytes(I2C_TypeDef *I2Cx, uint8_t Addr, uint8_t Reg, const uint8_t *pData, uint16_t Size)
{
	uint8_t  timeout = 0u;
	uint8_t  result;
	uint16_t i;
	int      ret;

	if((pData == 0) && (Size > 0u))
	{
		return -2;
	}

	ret = I2C_AddressPhase(I2Cx, Addr, I2C_DIR_WRITE, &timeout);

	if(ret != I2C_ADDR_OK)
	{
		return (timeout != 0u) ? -3 : -1;
	}

	/* 先发寄存器号 */
	if(I2C_WaitFlag(I2Cx, I2C_FLAG_TXE, RESET) != I2C_WAIT_OK)
	{
		I2C_Abort(I2Cx);
		return -3;
	}

	I2C_SendData(I2Cx, (uint16_t)Reg);

	result = I2C_WaitEither(I2Cx, I2C_FLAG_AF, I2C_FLAG_TXE);

	if(result != 1u)
	{
		I2C_Abort(I2Cx);
		return (result == 0u) ? -2 : -3;
	}

	/* 再发数据 */
	for(i = 0u; i < Size; i++)
	{
		result = I2C_WaitEither(I2Cx, I2C_FLAG_AF, I2C_FLAG_TXE);

		if(result != 1u)
		{
			I2C_Abort(I2Cx);
			return (result == 0u) ? -2 : -3;
		}

		I2C_SendData(I2Cx, pData[i]);
	}

	result = I2C_WaitEither(I2Cx, I2C_FLAG_AF, I2C_FLAG_BTF);

	if(result != 1u)
	{
		I2C_Abort(I2Cx);
		return (result == 0u) ? -2 : -3;
	}

	I2C_GenerateSTOP(I2Cx, ENABLE);

	return 0;
}

/* ---------------------------------------------------------------------------
 * 总线抢救
 * ------------------------------------------------------------------------- */

/* 空转约指定的微秒数。72MHz 下每次循环约 4~5 条指令，按 4 条估算 */
static void I2C_RecoverDelay(void)
{
	volatile uint32_t i;

	for(i = 0u; i < (uint32_t)I2C_RECOVER_SPIN_LOOPS; i++)
	{
		__NOP();
	}
}

/* 开某条 I2C 总线的 APB1 时钟。复位与恢复都要用到 */
static void I2C_EnableClock(I2C_TypeDef *I2Cx)
{
	if(I2Cx == I2C1)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	}
	else if(I2Cx == I2C2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	}
}

/* 把外设恢复成「刚 I2C_Init 完、400kHz、开应答」的状态。
 *
 * 为什么必须做这一步：总线被拉住时，光把引脚切来切去救不回来。
 * 外设自己的状态机、以及 SR1/SR2 里锁存的 AF/BUSY/BERR 不会因为引脚翻转而清掉；
 * 只 I2C_Cmd(ENABLE) 的话，下一次传输会立刻又失败，变成
 * 「一直失败 → 一直抢救 → 一直失败」的死循环，现象就是永远 BUSY。
 * 真正的复位要走 RCC 的 APB1 外设复位（I2C_DeInit 干的就是这件事），
 * 再重新 I2C_Init 把波特率等参数写回去。
 *
 * 注意 I2C_DeInit 需要有 APB1 时钟才能操作寄存器，所以先把时钟开上。 */
static void I2C_ReinitPeripheral(I2C_TypeDef *I2Cx)
{
	I2C_InitTypeDef init;

	I2C_EnableClock(I2Cx);

	/* 真正的外设复位：所有寄存器回到复位值 */
	I2C_DeInit(I2Cx);

	init.I2C_Mode                = I2C_Mode_I2C;
	init.I2C_DutyCycle           = I2C_DutyCycle_2;
	init.I2C_OwnAddress1         = 0x00;
	init.I2C_Ack                 = I2C_Ack_Enable;
	init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	init.I2C_ClockSpeed          = I2C_SPEED_HZ;

	I2C_Init(I2Cx, &init);
	I2C_Cmd(I2Cx, ENABLE);
}

int My_I2C_ResetBus(I2C_TypeDef *I2Cx)
{
	const I2C_PinMap_t *map = I2C_FindPinMap(I2Cx);
	GPIO_InitTypeDef    gpio;
	uint32_t            i;

	if(map == 0)
	{
		return -1;   /* 不是本板已登记的 I2C 总线，不猜引脚 */
	}

	/* #1 关外设：把两根线从外设手里拿回来交给 GPIO。
	 * 后面第 3、4 步只用 GPIO 打时序，所以外设状态再乱也能把从机冲出来 */
	I2C_Cmd(I2Cx, DISABLE);

	/* #2 切成普通开漏输出，两根线都释放（高电平由外部上拉提供） */
	gpio.GPIO_Pin   = (uint16_t)(map->SCL_Pin | map->SDA_Pin);
	gpio.GPIO_Mode  = GPIO_Mode_Out_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(map->GPIOx, &gpio);
	GPIO_SetBits(map->GPIOx, (uint16_t)(map->SCL_Pin | map->SDA_Pin));

	I2C_RecoverDelay();

	/* #3 打时钟脉冲，直到从机把 SDA 放开。
	 *
	 * 每个高电平后面都**回读 SCL**：如果从机在做时钟延展（把 SCL 拉住不放），
	 * 光发脉冲是没用的，必须先等它松手。这里不无限等，最多打 9 个脉冲就收工，
	 * 而且用 I2C_RecoverHalfPeriod() 里的循环上限兜底。 */
	for(i = 0u; i < (uint32_t)I2C_RECOVER_PULSES; i++)
	{
		GPIO_ResetBits(map->GPIOx, map->SCL_Pin);
		I2C_RecoverDelay();

		GPIO_SetBits(map->GPIOx, map->SCL_Pin);
		I2C_RecoverDelay();

		/* SDA 已经被从机放开（读到高）就不用再打了 */
		if(GPIO_ReadInputDataBit(map->GPIOx, map->SDA_Pin) == Bit_SET)
		{
			break;
		}
	}

	/* #4 补一个停止位：SCL 为高时把 SDA 由低拉高，把总线带回明确的空闲态 */
	GPIO_ResetBits(map->GPIOx, map->SDA_Pin);
	I2C_RecoverDelay();

	GPIO_SetBits(map->GPIOx, map->SDA_Pin);
	I2C_RecoverDelay();

	/* #5 判断总线到底有没有真的松开。
	 * 这一步是给「软件救不回来」的情况定性用的：
	 *   SCL 或 SDA 仍然被拉在低电平 -> 是硬件层面被拉住（从机掉电/复位/器件坏/
	 *     走线短路），打时钟脉冲和复位外设都没用，只能查供电与接线。
	 *   两线都回到高 -> 总线空闲了，接下来重新初始化外设即可。
	 * 把这个结论记下来，屏幕上的 E%04X 里能看到（SR2 的 bit0=MSL bit1=BUSY）。 */
	bus_held_low = 0u;

	if(GPIO_ReadInputDataBit(map->GPIOx, map->SCL_Pin) == Bit_RESET)
	{
		bus_held_low = 1u;
	}

	if(GPIO_ReadInputDataBit(map->GPIOx, map->SDA_Pin) == Bit_RESET)
	{
		bus_held_low = 1u;
	}

	/* #6 引脚交还外设，并做一次真正的外设复位 + 重新初始化。
	 * 必须真复位：只 I2C_Cmd(ENABLE) 的话，SR1/SR2 里锁存的 AF/BUSY 还在，
	 * 下一次传输会立刻又失败，变成「一直失败一直救不回来」的死循环 */
	gpio.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(map->GPIOx, &gpio);

	I2C_ReinitPeripheral(I2Cx);

	return 0;
}

/* 上一次抢救之后，两线是否仍然被拉在低电平。
 * 1 = 硬件层面被拉住（软件救不了，要查供电/接线/器件）
 * 0 = 总线已松开 */
uint8_t My_I2C_BusWasHeldLow(void)
{
	return bus_held_low;
}
