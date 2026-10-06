#include "stm32f10x.h"
#include "task.h"
#include "bsp_delay.h"
#include "app_usart1.h"
#include "app_bluetooth.h"
#include "app_pwm.h"
#include "app_encoder.h"
#include "app_mpu6050.h"
#include "app_attitude.h"
#include "pwm_test.h"
#include "encoder_test.h"
#include "mpu6050_test.h"
#include "motor_test.h"
#include "app_balance.h"
#include "app_motor.h"
#include "app_oled.h"
#include "balance_test.h"
#include "button.h"



//
// 调试打印任务：姿态角 ×100 按整数打印，单位 0.01 度，不用 %f（软件浮点要几毫秒）
// My_USART_Printf 阻塞约 2ms，故周期 50ms；跑控制环时必须保持注释，只台面调试时开
//
/*static void Attitude_Debug_Task(void)
{
	My_USART_Printf(USART1, "%d,%d,%d\r\n",
	                (int)(App_Attitude_GetRoll()  * 100.0f),
	                (int)(App_Attitude_GetPitch() * 100.0f),
	                (int)(App_Attitude_GetYaw()   * 100.0f));
}
*/
/*
// ================= 备选方案：JustFloat 协议（留着不用）=================
// 启用步骤：去掉本段注释；文件开头加 #include <string.h>；把 Attitude_Debug_Task 里的
// My_USART_Printf 换成 Send_JustFloat(roll, pitch, yaw)；VOFA+ 数据格式改成 JustFloat
//
// 帧格式：N 个 float（小端） + 帧尾 00 00 80 7F。
// 省掉 vsprintf 的软件浮点格式化，无精度损失，速率可以更高。
static void Send_JustFloat(float a, float b, float c)
{
	uint8_t buf[16];

	memcpy(&buf[0], &a, 4);
	memcpy(&buf[4], &b, 4);
	memcpy(&buf[8], &c, 4);

	buf[12] = 0x00;   //帧尾固定这4个字节
	buf[13] = 0x00;
	buf[14] = 0x80;
	buf[15] = 0x7F;

	My_USART_SendBytes(USART1, buf, 16);
}
*/

/* ===================== 调试打印：平衡环关键量（有线 + 蓝牙两路同时发）=====================
 * 有线走 USART1（J2 座，PA9/PA10），蓝牙走 USART2（J5 座，PA2-TXD/PA3-RXD，JDY-29 透传，
 * 波特率不用管）。两路格式相同，VOFA+ / 串口助手数据格式选 FireWater。
 *
 * 一行 9 个整数：
 *   pitch,omega_ref,spd_L,spd_R,duty_L,duty_R,enabled,reason,stop_pitch
 *     pitch      车身倾角，0.01 度
 *     omega_ref  平衡环算出的目标轮速，0.01 rad/s
 *     spd_L/R    编码器实测轮速，0.01 rad/s
 *     duty_L/R   电机调速环输出占空比，%
 *     enabled    平衡环是否在跑，1/0
 *     reason     最后一次停机原因，0=没停/人为，1=倾角超限，2=传感器失效
 *     stop_pitch 自动停机瞬间的倾角，0.01 度
 *
 * 车立不住时按下表判断：
 *   duty 顶到 ±100 而 spd 上不去 -> 电机带不动（电池/电流/接线），不是算法问题
 *   duty 很小、车慢慢倒下        -> 控制环力量不够，增益太小或模型参数不对
 *   reason=1 且 stop_pitch≈45   -> 倒到 45 度被保护停机，是结果不是原因
 *   reason=2                     -> 电机一转就弄坏 I2C，传感器数据断了
 *   一启动 omega_ref 就单向猛涨  -> 中位值不对，或速度环符号反了
 *
 * My_USART_Printf 阻塞约 2~3ms/行，App_Bluetooth_Printf 非阻塞（实际发送由 USART2 中断做）。
 * 蓝牙 9600 下一行 44 字节占 46ms 空中时间，周期不得小于 100ms，否则缓冲排不空会整条丢。
 * 阻塞打印不能放进控制环，故单独占一个 100ms 任务；嫌控制环抖就把下面 Task_Add 那句注释掉。
 */
#define DEBUG_DEG2RAD  (0.0174533f)   //度 -> 弧度：π/180

static void Balance_Debug_Task(void)
{
	//先一次取齐 9 个量：两路必须是同一采样时刻的值，不能发完有线再取一遍
	int pitch      = (int)(App_Attitude_GetPitch() * 100.0f);
	int omega_ref  = (int)(App_Balance_GetTargetSpeed() * 100.0f);
	int spd_l      = (int)(App_Encoder_GetSpeed_L() * DEBUG_DEG2RAD * 100.0f);
	int spd_r      = (int)(App_Encoder_GetSpeed_R() * DEBUG_DEG2RAD * 100.0f);
	int duty_l     = (int)App_Motor_GetDuty_L();
	int duty_r     = (int)App_Motor_GetDuty_R();
	int enabled    = (int)App_Balance_IsEnabled();
	int reason     = (int)App_Balance_GetStopReason();
	int stop_pitch = (int)(App_Balance_GetStopPitch() * 100.0f);

	My_USART_Printf(USART1, "%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
	    pitch, omega_ref, spd_l, spd_r, duty_l, duty_r, enabled, reason, stop_pitch);

	App_Bluetooth_Printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
	    pitch, omega_ref, spd_l, spd_r, duty_l, duty_r, enabled, reason, stop_pitch);
}

/* 状态提示：只在按键事件时发一句，阻塞发送无所谓 */
static void Balance_Log(const char *Msg)
{
	My_USART_Printf(USART1, "%s\r\n", Msg);
}

/* ===================== 按键：平衡环的启停 =====================
 * K1~K4 = PB1/PB0/PA5/PA4，一端接 GND，按下为低，与 my_lib/button.c 约定一致。
 * 这里只用 K1 做启停，其余三个要用再加一句 My_Button_Init。
 *
 * 必须有按键：倒车后 App_Balance_Disable() 会切断电机，没按键只能断电重上电。
 */
#define KEY_START_MAX_TILT_DEG  (10.0f)   //允许启动的最大车身倾角，单位 度

static Button_TypeDef key_start;   //K1：单击启动，再单击停机

//K1 单击回调：button.c 在松开约 200ms 确认没有第二击后才触发，点完要稍等；双击不触发
//
static void Balance_Key_Click(uint8_t clicks)
{
	float pitch;

	if(clicks != 1u)
	{
		return;
	}

	//已经在跑了：这一下就是停机
	if(App_Balance_IsEnabled() != 0u)
	{
		App_Balance_Disable();
		Balance_Log("balance OFF");
		return;
	}

	//启动前确认车身大致竖直，否则下一拍会被判"倒了"，不如直接把原因打出来
	pitch = App_Attitude_GetPitch();

	if(pitch > -KEY_START_MAX_TILT_DEG && pitch < KEY_START_MAX_TILT_DEG)
	{
		App_Attitude_ResetYaw();   //朝向基准归零，以后加转向环要用
		App_Balance_Enable();
		Balance_Log("balance ON");
	}
	else
	{
		Balance_Log("tilt too large, hold car upright first");
	}
}

//按键进程，注册到调度器周期调用，10ms 与 button.c 内部的消抖时间一致
static void Key_Task(void)
{
	My_Button_Proc(&key_start);
}

//按键初始化：只用 K1（PB1）
static void Key_Init(void)
{
	Button_InitTypeDef key_cfg = {GPIOB, GPIO_Pin_1};

	My_Button_Init(&key_start, &key_cfg);
	My_Button_SetClickCb(&key_start, Balance_Key_Click);
}

int main(void)
{
	// 设置中断优先级分组：2 位抢占优先级 + 2 位子优先级
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	Delay_Init();        // 显式初始化
	App_USART1_Init();    // 调试串口（有线，J2 座，PA9/PA10）
	App_Bluetooth_Init(); // 蓝牙串口（无线，J5 座，PA2-TXD / PA3-RXD，中断收发）
	App_PWM_Init();      // 电机 PWM 和方向控制引脚
	App_Encoder_Init();  // 左右电机编码器和外部中断
	App_MPU6050_Init();  // 六轴传感器
	App_Attitude_Init(); // 姿态解算（内部要静止校准陀螺零偏，约1秒，期间别碰板子）
	// ==== 各模块测试程序：要用时把它挪到这里，并把下面正式入口整段注释掉 ====
	//PWM_Test();
	//Motor_Sign_Test();
	//Motor_Crawl_Test();  //低速死区/爬行测试：轮子架空，看"咯噔"是否静摩擦门槛造成
	//MPU6050_Test();
	//Motor_Speed_Test();
	//Encoder_M_Method_Test();
	//Encoder_T_Method_Test();
	//Encoder_Test();
	//Balance_Center_Test(); //标定机械中位：轮子悬空，只读 pitch，电机不转
	//Balance_Sign_Test();   //内部是 while(1)，调进去就回不来

	// ==== 正式运行入口 ====
	App_Motor_Init();     // 左右电机调速环的 PID
	App_Balance_Init();   // 平衡环的三层 PID（内部先停机，等按键/显式使能）
	Key_Init();           // K1 = PB1，单击启停平衡环
	App_OLED_Init();      // 状态屏（PB8/PB9 软I2C），初始化失败只打一句串口提示，不影响控制

	// 注册任务：注册顺序就是调用顺序
	// 【必须】传感器读取排在姿态解算前，否则每周期都在积分同一个旧样本
	// 【必须】平衡环排在电机调速环前，否则这一拍的目标轮速要等下一拍才执行
	// 控制环里不能有阻塞打印，会把控制周期拖慢；打印一律单独占一个任务
	Task_Add(App_MPU6050_Update,  ATT_TASK_PERIOD_MS);
	Task_Add(App_Attitude_Update, ATT_TASK_PERIOD_MS);
	Task_Add(App_Balance_Update,  BAL_TASK_PERIOD_MS);
	Task_Add(App_Motor_Update,    MOTOR_TASK_PERIOD_MS);
	Task_Add(Key_Task,            10u);   // 按键消抖，与 button.c 内部的消抖时间一致
	Task_Add(App_OLED_Task,       200u);  // 状态屏重画：只写帧缓冲并启动传输，不碰 I2C
	//Task_Add(Attitude_Debug_Task, 50u);   // 调试用：20Hz 打印姿态角
	Task_Add(Balance_Debug_Task, 100u);   // 调试用：10Hz 把平衡环关键量发给 VOFA+（有线）和手机蓝牙
	                                      // 蓝牙 9600 下一行占 46ms 空中时间，周期不得小于 100ms，否则数据被丢

	// 上电后平衡环默认停机：立稳车后单击 K1 启动，再单击停机；按键前电机不动
	// 看门狗还没做

	while(1)
	{
		Task_Run();
		App_OLED_Pump();   // 空闲时每次搬 8 字节上屏，把整屏刷新切碎，不占控制环时间
	}
}
