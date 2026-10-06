#include "balance_test.h"
#include "app_motor.h"
#include "app_encoder.h"
#include "app_balance.h"
#include "app_attitude.h"
#include "app_mpu6050.h"
#include "app_usart1.h"
#include "delay.h"

/* 度/s -> rad/s */
#define TEST_DEG2RAD    (0.0174533f)

/* 每个目标速度保持多久，单位 ms */
#define TEST_HOLD_MS    (600u)

/* 打印间隔，单位 ms。打印是阻塞的，太快会把控制周期拖长 */
#define TEST_PRINT_MS   (20u)

/* 【中位标定】采样周期和打印周期，单位 ms */
#define TEST_CENTER_PERIOD_MS  (5u)
#define TEST_CENTER_PRINT_MS   (200u)

/* 【中位标定】pitch 平滑的时间常数，单位 s；越大越稳但跟手越慢 */
#define TEST_CENTER_TAU_S      (1.0f)


/* 【第一个测】电机调速环测试
 * 做之前：轮子架空或放地上都行，架空更容易看出跟随好坏
 * 目标速度依次 10 -> 20 -> 0 -> -20 -> -10 (rad/s)，输出 target,actual_L,actual_R（0.01 rad/s）
 * 判定：实际值快速跟上目标并稳在附近 = 合格；一直跟不上 = Kp 或 Ki 太小；振荡啸叫 = Kp 或 Ki 太大 */
void Motor_Speed_Test(void)
{
	const float target[5] = {10.0f, 20.0f, 0.0f, -20.0f, -10.0f};
	uint8_t i;
	uint16_t t;

	App_Motor_Init();
	App_Motor_Enable();

	My_USART_Printf(USART1, "target,actual_L,actual_R (0.01rad/s)\r\n");

	for(i = 0u; i < 5u; i++)
	{
		App_Motor_SetTarget_L(target[i]);
		App_Motor_SetTarget_R(target[i]);

		for(t = 0u; t < TEST_HOLD_MS; t += 5u)
		{
			App_Motor_Update();

			if((t % TEST_PRINT_MS) == 0u)
			{
				My_USART_Printf(USART1, "%d,%d,%d\r\n",
				    (int)(target[i] * 100.0f),
				    (int)(App_Encoder_GetSpeed_L() * TEST_DEG2RAD * 100.0f),
				    (int)(App_Encoder_GetSpeed_R() * TEST_DEG2RAD * 100.0f));
			}

			Delay(5);
		}
	}

	//测试结束，切断输出
	App_Motor_Disable();
	My_USART_Printf(USART1, "done, motors stopped\r\n");
}


/* 【第二个测】平衡环符号验证
 * 做之前：把车拿在手里、轮子悬空，不要放地上。MPU6050 和姿态模块由 main 初始化，这里不再重复初始化
 * 输出 pitch(0.01度), rate(0.01度/s), omega_ref(0.01rad/s), cart_speed(mm/s)
 * 判定：
 *   1) 把车往前倾，轮子应往前转，omega_ref 的符号即"前倾"那一侧；轮子往后转说明整个环反了
 *   2) 把车往前轻推，cart_speed 应为正且 omega_ref 变负；cart_speed 符号与实际推动方向相反说明重建公式错了 */

//打印当前姿态、目标轮速和重建出来的车身速度
static void Balance_Sign_Debug(void)
{
	My_USART_Printf(USART1, "%d,%d,%d,%d\r\n",
	    (int)(App_Attitude_GetPitch() * 100.0f),          //倾角 0.01度
	    (int)(App_Attitude_GetPitchRate() * 100.0f),      //角速度 0.01度/s
	    (int)(App_Balance_GetTargetSpeed() * 100.0f),     //目标轮速 0.01rad/s
	    (int)(App_Balance_GetCartSpeed() * 1000.0f));     //车身线速度 mm/s
}

void Balance_Sign_Test(void)
{
	App_Motor_Init();
	App_Balance_Init();

	My_USART_Printf(USART1, "HOLD THE CAR, WHEELS OFF THE GROUND!\r\n");
	My_USART_Printf(USART1, "pitch,rate,omega_ref\r\n");

	App_Balance_Enable();

	while(1)
	{
		App_MPU6050_Update();
		App_Attitude_Update();
		App_Balance_Update();
		App_Motor_Update();

		Balance_Sign_Debug();

		Delay(5);
	}
}


/* 【第四个测】机械中位标定：测出 app_balance.c 里的 BAL_CENTER_ANGLE_DEFAULT
 * 做之前：把车拿在手里、轮子悬空。本测试电机保持断开，不会突然转起来
 * 中位 = 重心正好在轮轴正上方、松手不往任何一边倒的姿态，对应的传感器读数就是要填的值。
 *   差 1~2 度靠速度环积分扛得住，只是启动时会先朝一边爬一下；差 5 度以上积分限幅用光，车会一直跑掉。
 * 输出 inst,avg,center（均为 0.01 度，avg 除以 100 即度数）：inst 为瞬时 pitch，看手稳不稳；
 *   avg 为 pitch 的 1 秒平滑值，稳定之后读这一列；center 为当前生效的中位设定值，与 avg 相等即标定对了。
 * 步骤：
 *   1) 手持小车在地面上缓慢前倾、后倾，找松手不往任何一边倒的姿态，反复 2~3 次记下 avg 的平均值
 *   2) 把这个数除以 100 填进 BAL_CENTER_ANGLE_DEFAULT 后重编，或调试时直接调 App_Balance_SetCenterAngle()
 *   3) 再跑一遍本测试，把车摆回中位姿态，看 center 与 avg 是否相等
 * 注意：数字长时间不动是 MPU6050 掉线（I2C 不通），不是车真的稳 */
void Balance_Center_Test(void)
{
	float pitch;
	float avg;
	float k;
	uint16_t cnt = 0u;
	uint8_t warned = 0u;

	App_Motor_Init();
	App_Motor_Disable();   //标定期间绝对不能让轮子转起来

	My_USART_Printf(USART1, "HOLD THE CAR, WHEELS OFF THE GROUND!\r\n");
	My_USART_Printf(USART1, "inst,avg,center (0.01deg)\r\n");

	//先把平滑值打满成当前读数，免得开头一段从 0 慢慢爬上来
	App_MPU6050_Update();
	App_Attitude_Update();
	avg = App_Attitude_GetPitch();

	//一阶低通系数，和 app_attitude.c 里互补滤波是同一个写法
	k = TEST_CENTER_TAU_S / (TEST_CENTER_TAU_S + (float)TEST_CENTER_PERIOD_MS * 0.001f);

	while(1)
	{
		App_MPU6050_Update();
		App_Attitude_Update();

		if(App_MPU6050_IsOk() == 0u)
		{
			//读数是死的，提示一次就够，不然串口会被刷屏
			if(warned == 0u)
			{
				warned = 1u;
				My_USART_Printf(USART1, "MPU6050 DATA LOST, value is frozen\r\n");
			}
			Delay(TEST_CENTER_PERIOD_MS);
			continue;
		}
		warned = 0u;

		pitch = App_Attitude_GetPitch();

		avg = k * avg + (1.0f - k) * pitch;

		//每 200ms 打一行，太密会刷爆串口
		cnt++;
		if(cnt >= (TEST_CENTER_PRINT_MS / TEST_CENTER_PERIOD_MS))
		{
			cnt = 0u;

			My_USART_Printf(USART1, "%d,%d,%d\r\n",
			    (int)(pitch * 100.0f),
			    (int)(avg * 100.0f),
			    (int)(App_Balance_GetCenterAngle() * 100.0f));
		}

		Delay(TEST_CENTER_PERIOD_MS);
	}
}
