#include "app_mpu6050.h"
#include "bsp_si2c.h"
#include "bsp_delay.h"
#include "bsp_usart.h"

/* 温度换算参数，来自 PS-MPU-6000A-00 第 6.3 节 */
#define MPU6050_TEMP_SENSITIVITY   340.0f     //灵敏度，单位 LSB/°C
#define MPU6050_TEMP_OFFSET        (-521.0f)  //温度偏移，单位 LSB（测量条件35°C）
#define MPU6050_TEMP_OFFSET_REF    35.0f      //上面这个偏移对应的温度，单位 °C

/* MPU6050 的 I2C 从机地址（左对齐），AD0 接地就是 0xd0 */
#define MPU6050_ADDR               0xd0

/* ---------- 软件 I2C 参数 ----------
 * MPU6050 挂在 PB10(SCL) / PB11(SDA)，与原来硬件 I2C2 用的引脚相同，
 * 接线一根都不用动 —— 只是改成用 GPIO 打时序。
 *
 * 半周期取 3us（约 166kHz）。MPU6050 支持到 400kHz，3us 有充足余量；
 * 一轮 14 字节读取约 14*9*2*3us ≈ 0.76ms，占 5ms 控制周期约 15%。
 * ⚠️ 别往小调：半周期直接换算成 CPU 占用；而且线长、上拉不够时速率越高越容易出错。 */
#define MPU6050_SI2C_HALF_US    (3u)

static SI2C_TypeDef mpu_si2c = {GPIOB, GPIO_Pin_10, GPIOB, GPIO_Pin_11, MPU6050_SI2C_HALF_US};

/* I2C 读失败恢复策略：
 *   连续失败 >= MPU6050_FAIL_LIMIT 次 -> App_MPU6050_IsOk() 返回 0，
 *   平衡环立刻切断电机输出（这一点不能松：姿态角靠陀螺积分，一直拿同一个旧样本
 *   积分会越跑越偏，用错数据驱动电机比停车危险得多）。
 *   但**不再永久停机**：只要重新读到有效帧，IsOk() 自动恢复 1，平衡环接着跑。
 *   原来只要连续 3 次（15ms）失败就永久停机，偶发丢包会让车必然躺下，太脆。
 *   软件 I2C 不会「外设卡死」，所以失败通常只是这一帧没读上，下一帧自然重来。 */
#define MPU6050_FAIL_LIMIT      (3u)

static float ax, ay, az;
static float temperature;
static float gx, gy, gz;

static uint8_t  mpu_fail_cnt = 0u;      //I2C 连续读失败次数

/* 【诊断】I2C 读失败的现场快照。
 *   失败是「传感器失效」的直接原因，要知道是哪一种才好修：
 *   从机不应答(-1)、数据被拒(-2)、还是从机拉死 SCL(-3) */
static int      mpu_dbg_err    = 0;     //最后一次失败的返回值（-1/-2/-3）
static uint16_t mpu_dbg_fails  = 0u;    //累计失败次数

static int     reg_write(uint8_t reg, uint8_t value);
static uint8_t reg_read(uint8_t reg, int *pStatus);
static uint8_t mpu_whoami_ok(void);

//初始化mpu6050
void App_MPU6050_Init(void)
{
	uint8_t cfg_ok;

	/* #1 把 MPU6050 挂到一条**软件（位拆）I2C** 上，引脚仍是 PB10/PB11。
	 *
	 * 为什么不用 STM32 的硬件 I2C2：那个外设的状态机非常难伺候 ——
	 * 一旦被干扰打断就会锁存 AF/BUSY 之类的状态，必须靠「打时钟脉冲 + 复位外设」
	 * 才能拉回来，而且每次拉回来没读几帧又失败。本项目实测就是这种表现。
	 * 软件 I2C 完全用 GPIO + 延时打时序，出问题最多是这一次传输失败，
	 * 下一个起始条件就重新来，不存在「外设卡住」这回事 —— 这正是我们想要的鲁棒性。
	 *
	 * 代价是 CPU 占用：一轮 14 字节读取约 18KB 个半周期，半周期 3us 时约 0.5ms，
	 * 占 5ms 控制周期的 10%。可接受，但半周期别往小调（见 MPU6050_SI2C_HALF_US）。 */
	My_SI2C_Init(&mpu_si2c);

	//#2设置MPU6050的参数
	//每一条都判返回值：任何一条没写进去，传感器就工作在错误的量程/滤波下，
	//姿态角的量纲会整个错掉，而现象只是"车立不住"，很难反查到根因
	cfg_ok = 1u;

	if(reg_write(0x6b, 0x80) != 0) { cfg_ok = 0u; }   //复位
	Delay(100);

	if(reg_write(0x6b, 0x00) != 0) { cfg_ok = 0u; }   //唤醒MPU6050

	//数字低通滤波 DLPF_CFG=3：加速度 44Hz / 陀螺 42Hz 带宽，延迟 4.9ms
	//不配的话传感器跑在 8kHz 无滤波状态，噪声直接进入姿态积分
	if(reg_write(0x1a, 0x03) != 0) { cfg_ok = 0u; }

	//采样率分频：开 DLPF 后陀螺输出 1kHz，(1+4) 分频 = 200Hz，
	//正好配合 5ms 的姿态任务周期，一个周期一个新样本
	if(reg_write(0x19, 0x04) != 0) { cfg_ok = 0u; }

	if(reg_write(0x1b, 0x18) != 0) { cfg_ok = 0u; }   //陀螺量程 +-2000°/s

	if(reg_write(0x1c, 0x00) != 0) { cfg_ok = 0u; }   //加速度量程 +-2g

	mpu_fail_cnt = 0u;

	if(cfg_ok == 0u)
	{
		My_USART_Printf(USART1, "MPU6050 CONFIG WRITE FAILED\r\n");
	}

	//上电自检：读 WHO_AM_I，正常固定为 0x68
	//读不到说明 I2C 根本没通，提前报出来比后面对着乱跳的欧拉角找原因省事
	//注意必须分开判「读失败」和「值不对」：读失败时返回的 0 会被误当成一个有效 ID
	if(!mpu_whoami_ok())
	{
		My_USART_Printf(USART1, "MPU6050 NOT FOUND! check PB10/PB11\r\n");
	}
}

//更新MPU6050的原始数据
void App_MPU6050_Update()
{
	uint8_t buf[14];

	//从 0x3B 起一次性连读 14 字节：AX,AY,AZ(6) + 温度(2) + GX,GY,GZ(6)
	//必须连读，分开读会读到不同时刻的数据，拼起来没有意义
	//
	//读失败直接返回，让 ax..gz 保持上一次的值。绝不能拿 buf 去算：失败时 buf 是
	//没被写过的栈垃圾，算出的姿态角会突然乱跳，平衡环能直接把车甩出去
	{
		int i2c_ret = My_SI2C_RegReadBytes(&mpu_si2c, MPU6050_ADDR, 0x3b, buf, 14);

		if(i2c_ret != 0)
		{
			/* 软件 I2C 的失败码：
			 *   -1 从机没应答地址   -> 模块掉电/接错，或线被拉住
			 *   -2 数据/寄存器被拒  -> 传输中途被打断
			 *   -3 从机把 SCL 拉死  -> 时钟延展超时，器件异常
			 * 总线本身不需要抢救：软件 I2C 没有「外设卡死」这回事，
			 * 下一个起始条件就是全新的时序。所以这里只记数、不再调 ResetBus，
			 * 另外每次失败都补一个停止位把从机放回空闲 */
			mpu_dbg_err = i2c_ret;
			mpu_dbg_fails++;

			SI2C_ForceStop(&mpu_si2c);

			if(mpu_fail_cnt < 255u)
			{
				mpu_fail_cnt++;
			}

			return;   /* 保持上一次的有效数据，不要拿栈垃圾去算姿态 */
		}
	}

	mpu_fail_cnt = 0u;

	int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
	int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
	int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);
	
	//±2g量程(AFS_SEL=0)的灵敏度是 16384 LSB/g，所以系数 = 1/16384 = 6.1035e-5
	ax = ax_raw * 6.1035e-5f;
	ay = ay_raw * 6.1035e-5f;
	az = az_raw * 6.1035e-5f;
	
	int16_t temperature_raw = (int16_t)((buf[6] << 8) | buf[7]);

	
	// T = (TEMP_OUT - 偏移) / 灵敏度 + 偏移对应温度 = TEMP_OUT/340 + 36.5324
	// 芯片测的是晶圆温度，比环境温度高 10℃ 左右属正常
	temperature = (temperature_raw - MPU6050_TEMP_OFFSET) / MPU6050_TEMP_SENSITIVITY
	              + MPU6050_TEMP_OFFSET_REF;
	
	int16_t gx_raw = (int16_t)((buf[8] << 8) | buf[9]);
	int16_t gy_raw = (int16_t)((buf[10] << 8) | buf[11]);
	int16_t gz_raw = (int16_t)((buf[12] << 8) | buf[13]);
	
	//±2000°/s量程(FS_SEL=3)的灵敏度是 16.4 LSB/(°/s)，所以系数 = 1/16.4 = 6.0976e-2
	gx = gx_raw * 6.0976e-2f;
	gy = gy_raw * 6.0976e-2f;
	gz = gz_raw * 6.0976e-2f;
	
	
}

//获取三轴加速度
float App_MPU6050_GetAx(void)
{
	return ax;
}

float App_MPU6050_GetAy(void)
{
	return ay;
}

float App_MPU6050_GetAz(void)
{
	return az;
}

//获取温度计的值
float App_MPU6050_GetTemperature(void)
{
	return temperature;
}

//获取三轴角速度
float App_MPU6050_GetGx(void)
{
	return gx;
}
	
float App_MPU6050_GetGy(void)
	{
	return gy;
}

float App_MPU6050_GetGz(void)
	{
	return gz;
}

//读取器件ID：通信正常时固定返回0x68，用来快速确认I2C通不通
//读失败时返回 0（0 不可能是合法的 WHO_AM_I），调用方据此判断即可
uint8_t App_MPU6050_GetID(void)
{
	int status = -1;

	return reg_read(0x75, &status);
}

//传感器数据是否有效：I2C 连续失败到 MPU6050_FAIL_LIMIT 次返回 0
//平衡环必须看这个标志，失效时立刻停机，不能用旧样本积分出的姿态角去驱动电机
uint8_t App_MPU6050_IsOk(void)
{
	return (mpu_fail_cnt < MPU6050_FAIL_LIMIT) ? 1u : 0u;
}

//向寄存器写值：reg 寄存器地址，value 写入的值
//返回值：0 成功，非 0 失败（透传 bsp_i2c 的错误码）
static int reg_write(uint8_t reg, uint8_t value)
{
	uint8_t bytesToSend[] = {reg, value};

	//两字节连写：寄存器地址 + 数据。这是 MPU6050 单寄存器写的标准做法
	return My_SI2C_SendBytes(&mpu_si2c, MPU6050_ADDR, bytesToSend, 2);
}

//读取寄存器的值：reg 要读取的寄存器地址
//pStatus 非空时回填 I2C 的返回值（0 成功，负数失败）。
//读失败时返回值没有意义，调用方**必须**先看 pStatus 再用返回值
static uint8_t reg_read(uint8_t reg, int *pStatus)
{
	uint8_t regValue = 0;
	int     ret;

	//读寄存器必须用"寄存器读"接口（内含重复起始）
	//不能拆成先 SendBytes 再 ReceiveBytes，那样中间会多一个停止位
	ret = My_SI2C_RegReadBytes(&mpu_si2c, MPU6050_ADDR, reg, &regValue, 1);

	if(pStatus != 0)
	{
		*pStatus = ret;
	}

	return regValue;
}

//上电自检用：读 WHO_AM_I 并确认它是 0x68。
//必须把"读失败"和"值不对"分开判 —— 读失败时 reg_read 返回的是初值 0，
//直接拿去和 0x68 比较虽然也能发现不对，但报不出真正的原因
static uint8_t mpu_whoami_ok(void)
{
	int     status = -1;
	uint8_t id     = reg_read(0x75, &status);

	return ((status == 0) && (id == 0x68u)) ? 1u : 0u;
}

/* ===================== 诊断读数接口 =====================
 * 把 I2C 失败的现场暴露给显示层，方便不接电脑也能看出是哪种失败。
 * 参数都可以传 0，表示不关心那一项 */

//累计失败次数与抢救次数
void App_MPU6050_GetDiag(uint16_t *pFails, uint16_t *pResets, uint8_t *pBusHeld)
{
	if(pFails   != 0) { *pFails   = mpu_dbg_fails; }
	/* 软件 I2C 没有「外设状态要抢救」这回事，这两个字段保留只为不改显示层接口 */
	if(pResets  != 0) { *pResets  = 0u; }
	if(pBusHeld != 0) { *pBusHeld = 0u; }
}

//最后一次失败的返回值
void App_MPU6050_GetLastError(int *pErr, uint16_t *pSr1, uint16_t *pSr2)
{
	if(pErr != 0) { *pErr = mpu_dbg_err; }
	/* 软件 I2C 没有 SR1/SR2 寄存器，恒返回 0 */
	if(pSr1 != 0) { *pSr1 = 0u; }
	if(pSr2 != 0) { *pSr2 = 0u; }
}

/* 把失败码翻译成一句人能读的话。
 *
 * 软件 I2C 的失败码就是 My_SI2C_* 的返回值，含义很直白：
 *   -1 从机没应答地址    -> 模块掉电 / 接线松了 / 上拉缺失；先量模块 VCC
 *   -2 数据或寄存器被拒  -> 传输中途被打断，通常是干扰或总线速率偏高
 *   -3 从机把 SCL 拉死   -> 时钟延展超时，器件异常或线被短路
 * 返回静态常量字符串，可直接拿去打印 */
const char *App_MPU6050_DescribeError(void)
{
	switch(mpu_dbg_err)
	{
		case 0:  return "ok";
		case -1: return "noACK:check pwr";
		case -2: return "data NACK";
		case -3: return "SCL stuck";
		default: return "unknown err";
	}
}




