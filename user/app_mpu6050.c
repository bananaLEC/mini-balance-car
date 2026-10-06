#include "app_mpu6050.h"
#include "bsp_i2c.h"
#include "bsp_delay.h"
#include "bsp_usart.h"

/* 温度换算参数，来自 PS-MPU-6000A-00 第 6.3 节 */
#define MPU6050_TEMP_SENSITIVITY   340.0f     //灵敏度，单位 LSB/°C
#define MPU6050_TEMP_OFFSET        (-521.0f)  //温度偏移，单位 LSB（测量条件35°C）
#define MPU6050_TEMP_OFFSET_REF    35.0f      //上面这个偏移对应的温度，单位 °C

/* MPU6050 的 I2C 从机地址（左对齐），AD0 接地就是 0xd0 */
#define MPU6050_ADDR               0xd0

/* I2C 读失败恢复策略：连续失败 >= MPU6050_FAIL_LIMIT 次则 App_MPU6050_IsOk() 返回 0，
 * 平衡环必须停机；失败期间每 MPU6050_RECOVER_MS 毫秒用 My_I2C_ResetBus 抢救一次总线；
 * 成功读到一帧即清零计数。姿态角由陀螺积分而来，一直拿同一个旧样本积分会越跑越偏，
 * 此时宁可停车也不能继续驱动电机（5ms 一周期，3 次才 15ms，已经很宽松）。 */
#define MPU6050_FAIL_LIMIT      (3u)
#define MPU6050_RECOVER_MS      (200u)

static float ax, ay, az;
static float temperature;
static float gx, gy, gz;

static uint8_t  mpu_fail_cnt = 0u;      //I2C 连续读失败次数
static uint32_t mpu_recover_ms = 0u;    //上次抢救总线的时刻

static int     reg_write(uint8_t reg, uint8_t value);
static uint8_t reg_read(uint8_t reg, int *pStatus);
static uint8_t mpu_whoami_ok(void);

//初始化mpu6050
void App_MPU6050_Init(void)
{
	uint8_t cfg_ok;

	//1.初始化 MPU6050 的 I2C2 总线（PB10/PB11），先开时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	I2C_InitTypeDef I2C_InitStruct = {0};
	
	I2C_InitStruct.I2C_ClockSpeed = 400000;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_Mode  = I2C_Mode_I2C;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;//主机自身地址，作为主机时随便填
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;//必须使能应答，结构体清零后默认是关闭的
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	I2C_Init(I2C2,&I2C_InitStruct);
	
	
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
	if(My_I2C_RegReadBytes(I2C2, MPU6050_ADDR, 0x3b, buf, 14) != 0)
	{
		if(mpu_fail_cnt < 255u)
		{
			mpu_fail_cnt++;
		}

		//连续失败到限值后抢救一次总线（手工打时钟脉冲把从机冲回空闲），
		//每隔 MPU6050_RECOVER_MS 毫秒试一次，读到一帧即清零停下
		if(mpu_fail_cnt >= MPU6050_FAIL_LIMIT)
		{
			uint32_t now = GetTick();

			if((uint32_t)(now - mpu_recover_ms) >= MPU6050_RECOVER_MS)
			{
				mpu_recover_ms = now;
				My_I2C_ResetBus(I2C2);
			}
		}

		return;
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

	//两字节连写：寄存器地址 + 数据。这是 MPU6050 单寄存器写的标准做法，
	//等价于 My_I2C_MemWriteBytes，但少一次函数跳转
	return My_I2C_SendBytes(I2C2, 0xd0, bytesToSend, 2);
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
	ret = My_I2C_RegReadBytes(I2C2, 0xd0, reg, &regValue, 1);

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




