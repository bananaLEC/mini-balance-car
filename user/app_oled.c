#include "app_oled.h"
#include "bsp_oled.h"
#include "bsp_si2c.h"
#include "bsp_usart.h"
#include "app_attitude.h"
#include "app_balance.h"
#include "app_motor.h"
#include "app_encoder.h"
#include "app_mpu6050.h"

/* 原理图 U4 的 O-SCL / O-SDA 接 PB8 / PB9，和江协 OLED 例程的软 I2C 引脚一致。
 * MPU6050 走硬件 I2C2（PB10/PB11），和这条软总线没有冲突 */
static SI2C_TypeDef oled_si2c = {GPIOB, GPIO_Pin_8, GPIOB, GPIO_Pin_9, 2u};

static OLED_TypeDef oled;
static uint8_t oled_ok = 0u;        // 屏幕是否初始化成功，失败就整个模块静默

#define OLED_DEG2RAD  (0.0174533f)   // 度 -> 弧度：π/180

static int oled_i2c_write(uint8_t addr, const uint8_t *pdata, uint16_t size)
{
	return My_SI2C_SendBytes(&oled_si2c, addr, pdata, size);
}

// 把"放大 100 倍的整数"按 +xx.xx 画到当前光标处。
// 全程整数运算，不碰 %f：MicroLIB 的软件浮点格式化一次要几毫秒
static void Draw_Fix2(int value)
{
	int absv = (value < 0) ? -value : value;

	OLED_Printf(&oled, "%c%02d.%02d", (value < 0) ? '-' : '+', absv / 100, absv % 100);
}

static void Draw_Row(uint8_t row)
{
	OLED_SetCursor(&oled, 0, (int16_t)row * 8);
}

// 8x8 字体，一屏 16 列 x 8 行。所有数值都取整数副本再画，
// 一行里的每个量取自同一时刻，避免画到一半数据跳变
static void Redraw(void)
{
	int pitch = (int)(App_Attitude_GetPitch() * 100.0f);                    // 0.01 度
	int wl    = (int)(App_Encoder_GetSpeed_L() * OLED_DEG2RAD * 10.0f);     // 0.1 rad/s
	int wr    = (int)(App_Encoder_GetSpeed_R() * OLED_DEG2RAD * 10.0f);
	int dl    = (int)App_Motor_GetDuty_L();                                 // %
	int dr    = (int)App_Motor_GetDuty_R();
	int cart  = (int)(App_Balance_GetCartSpeed() * 100.0f);                 // 0.01 m/s
	int wref  = (int)(App_Balance_GetTargetSpeed() * 100.0f);               // 0.01 rad/s
	int temp  = (int)App_MPU6050_GetTemperature();                          // 度

	Draw_Row(0);
	OLED_Printf(&oled, "BAL:%s RSN:%d",
	            (App_Balance_IsEnabled() != 0u) ? "ON " : "OFF",
	            (int)App_Balance_GetStopReason());

	Draw_Row(1);
	OLED_DrawString(&oled, "PIT:");
	Draw_Fix2(pitch);
	OLED_DrawString(&oled, "deg");

	Draw_Row(2);
	OLED_Printf(&oled, "wL:%+04d wR:%+04d", wl, wr);

	Draw_Row(3);
	OLED_Printf(&oled, "dL:%+04d dR:%+04d", dl, dr);

	Draw_Row(4);
	OLED_DrawString(&oled, "V:");
	Draw_Fix2(cart);
	OLED_DrawString(&oled, "m/s");

	Draw_Row(5);
	OLED_DrawString(&oled, "Wref:");
	Draw_Fix2(wref);
	OLED_DrawString(&oled, "r/s");

	Draw_Row(6);
	OLED_Printf(&oled, "TMP:%+03dC", temp);
}

void App_OLED_Init(void)
{
	My_SI2C_Init(&oled_si2c);

	if(OLED_Bind(&oled, oled_i2c_write, OLED_I2C_ADDR) != OLED_OK)
	{
		My_USART_Printf(USART1, "OLED BIND FAILED\r\n");
		return;
	}

	if(OLED_Init(&oled) != OLED_OK)
	{
		My_USART_Printf(USART1, "OLED NOT FOUND! check PB8/PB9\r\n");
		return;
	}

	oled_ok = 1u;

	OLED_Clear(&oled);
	Redraw();
	OLED_SendBuffer(&oled);   // 上电先整屏刷一次：此时控制环还没注册，阻塞约100ms无所谓
}

void App_OLED_Task(void)
{
	// 上一帧还没搬完就跳过这周期：redraw 会改写帧缓冲，
	// 而 OLED_Pump 正在按旧内容一段段往屏幕发，两者不能同时碰缓冲区
	if(oled_ok == 0u || OLED_IsBusy(&oled) != 0u)
	{
		return;
	}

	Redraw();

	(void)OLED_StartSendBuffer(&oled);
}

void App_OLED_Pump(void)
{
	uint8_t more = 0u;

	if(oled_ok == 0u)
	{
		return;
	}

	// 一次只搬 8 字节（软 I2C 上约 1ms），剩下的留给主循环下一圈。
	// 整屏 128 段摊在空闲时间里发完，控制任务最多被推迟 1ms
	(void)OLED_Pump(&oled, &more);
}
