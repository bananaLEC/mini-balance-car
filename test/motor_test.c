#include "motor_test.h"
#include "app_pwm.h"
#include "app_encoder.h"
#include "app_usart1.h"
#include "delay.h"

/* 电机/编码器符号一致性测试
 * 做之前必须把两个轮子架空，否则车会直接窜出去
 * 输出 side,duty,speed：side 1=左 2=右，duty 为占空比百分比（正负代表两个转向），speed 为 0.01度/s
 * 判定（每路分别看）：duty 与 speed 同号 = 通过；duty>0 而 speed<0 = 这一路反了 */
void Motor_Sign_Test(void)
{
	int8_t dir;
	uint8_t side;

	My_USART_Printf(USART1, "LIFT THE WHEELS OFF THE GROUND!\r\n");
	My_USART_Printf(USART1, "side,duty,speed\r\n");

	for (side = 1u; side <= 2u; side++)
	{
		for (dir = 1; dir >= -1; dir -= 2)
		{
			float duty = 30.0f * (float)dir;
			float spd;

			if (side == 1u)
			{
				App_PWM_Set_L(duty);
			}
			else
			{
				App_PWM_Set_R(duty);
			}

			Delay(800);     //等电机转起来、测速读数稳定

			if (side == 1u)
			{
				spd = App_Encoder_GetSpeed_L();
			}
			else
			{
				spd = App_Encoder_GetSpeed_R();
			}

			My_USART_Printf(USART1, "%d,%d,%d\r\n",
			                (int)side, (int)duty, (int)spd);

			//先把这一路停住再测下一个，避免惯性互相干扰
			if (side == 1u)
			{
				App_PWM_Set_L(0.0f);
			}
			else
			{
				App_PWM_Set_R(0.0f);
			}

			Delay(500);
		}
	}

	//测试结束，确保两个电机都停住
	App_PWM_Set_L(0.0f);
	App_PWM_Set_R(0.0f);

	My_USART_Printf(USART1, "done, motors stopped\r\n");
}


/* 低速死区 / 爬行测试（开环给固定占空比，用来查"咯噔"从哪来）
 * 做之前必须把两个轮子架空，放到地上车会自己窜出去
 * 每档占空比保持 0.8 秒，先从 0% 加到 20%，再从 20% 减回 0。
 * 每行 duty(0.01%),pos_L,pos_R(0.01度),spd_L,spd_R(0.01rad/s)
 *   pos 只看有没有在变，比看速度可靠：轮子极慢时 T 法测速会因长时间没边沿而归零，位置一定在累加
 * 看什么：
 *   上行段第一个让 pos 开始累加的占空比 = 起动力（静摩擦门槛）
 *   下行段 pos 仍在累加的最低占空比 = 维持力（动摩擦门槛）
 *   两值之差就是"咯噔"的来源：门槛以下轮子不动，一过门槛就蹦一段，中间没有慢慢转的状态。
 *   起动力即后面静摩擦补偿要填的值：占空比不为 0 就至少给这么多，轮子不会卡在门槛下干等。
 *   若 3% 上下就平滑起转、上下行门槛几乎重合，则咯噔来自齿轮箱背隙，软件治不了 */
#define CRAWL_HOLD_MS    (800u)    /* 每档占空比保持多久 */
#define CRAWL_PRINT_MS   (50u)     /* 打印间隔 */
#define CRAWL_STEP_PCT   (2.0f)    /* 占空比每档步长，% */
#define CRAWL_STEPS      (10u)     /* 档数：0% ~ 20% */

//给两路电机一个固定占空比，保持 CRAWL_HOLD_MS，其间定期打印
static void Motor_Crawl_Step(float Duty)
{
	uint16_t t;

	App_PWM_Set_L(Duty);
	App_PWM_Set_R(Duty);

	for(t = 0u; t < CRAWL_HOLD_MS; t += 5u)
	{
		if((t % CRAWL_PRINT_MS) == 0u)
		{
			My_USART_Printf(USART1, "%d,%d,%d,%d,%d\r\n",
			    (int)(Duty * 100.0f),
			    (int)(App_Encoder_GetPos_L() * 100.0f),
			    (int)(App_Encoder_GetPos_R() * 100.0f),
			    (int)(App_Encoder_GetSpeed_L() * 0.0174533f * 100.0f),
			    (int)(App_Encoder_GetSpeed_R() * 0.0174533f * 100.0f));
		}

		Delay(5);
	}
}

void Motor_Crawl_Test(void)
{
	uint8_t i;

	/* 先给 0，确认不是调速环在偷偷给占空比 */
	App_PWM_Set_L(0.0f);
	App_PWM_Set_R(0.0f);
	Delay(200);   /* 停稳一下，让前面残留的运动停下 */

	My_USART_Printf(USART1, "duty(0.01%%),pos_L,pos_R(0.01deg),spd_L,spd_R(0.01rad/s)\r\n");

	My_USART_Printf(USART1, "--- rising 0 to 20%% ---\r\n");
	for(i = 0u; i <= CRAWL_STEPS; i++)
	{
		Motor_Crawl_Step((float)i * CRAWL_STEP_PCT);
	}

	My_USART_Printf(USART1, "--- falling 20%% to 0 ---\r\n");
	for(i = CRAWL_STEPS; i > 0u; i--)
	{
		Motor_Crawl_Step((float)(i - 1u) * CRAWL_STEP_PCT);
	}

	App_PWM_Set_L(0.0f);
	App_PWM_Set_R(0.0f);
	My_USART_Printf(USART1, "done, motors stopped\r\n");
}
