#include "app_pwm.h"
#include "math.h"

/* PWM 计数周期，实际计数范围 0~MOTOR_PWM_PERIOD。
 * 频率 = 72MHz / (0+1) / (3599+1) = 20kHz，占空比分辨率 3600 档。
 * 取 20kHz：72kHz 下一个 6% 的脉冲只有 0.83us，而 TB6612 开、关各要几百 ns，
 * 送到电机的占空比被削掉一大截，轮子低速起不动、静摩擦门槛偏高都有它一份。
 * 代价是 20kHz 落在可听范围边缘，可能有轻微高频啸叫。 */
#define MOTOR_PWM_PERIOD      3599U      // 20kHz @ 72MHz 主频（PSC=0）

/* 平衡车控制板 V2.0 原理图中的电机引脚定义 */
#define MOTOR_L_IN1_PIN       GPIO_Pin_12    /* PB12 -> AIN1 */
#define MOTOR_L_IN2_PIN       GPIO_Pin_13    /* PB13 -> AIN2 */
#define MOTOR_R_IN1_PIN       GPIO_Pin_14    /* PB14 -> BIN1 */
#define MOTOR_R_IN2_PIN       GPIO_Pin_15    /* PB15 -> BIN2 */

/* ===================== 电机方向符号 =====================
 * +1 = 正常，-1 = 这一路电机极性接反了，把"力"的方向翻过来（换驱动模块或重插电机
 * 线之后，若 duty 给正、轮子却反转，就用它翻一下；直接对调电机插头的两根线等效）。
 * 【注意】这里只翻电机方向，编码器的方向在 app_encoder.c 里另算，两边必须配套，
 * 否则调速环会变成正反馈，轮子自己冲到满速。 */
#define MOTOR_SIGN_L  (+1.0f)
#define MOTOR_SIGN_R  (+1.0f)

static float Motor_LimitDuty(float duty)
{
	// 防止传入超过范围的占空比，统一限制在 -100%~100%
	if (duty > 100.0f) return 100.0f;
	if (duty < -100.0f) return -100.0f;
	return duty;
}

/* 初始化电机驱动 TB6612FNG。PB12/PB13 = 左 AIN1/AIN2，PB14/PB15 = 右 BIN1/BIN2，
 * PA0/TIM2_CH1 = 左 PWMA，PA1/TIM2_CH2 = 右 PWMB */
void App_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {0};
	TIM_OCInitTypeDef TIM_OCInitStruct = {0};

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	                       
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// 配置四个方向控制脚为推挽输出
	GPIO_InitStruct.GPIO_Pin = MOTOR_L_IN1_PIN | MOTOR_L_IN2_PIN |
	                           MOTOR_R_IN1_PIN | MOTOR_R_IN2_PIN;
	                           
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	// 上电时先让四个方向脚为低电平，避免电机突然转动
	GPIO_ResetBits(GPIOB, MOTOR_L_IN1_PIN | MOTOR_L_IN2_PIN |
	                      MOTOR_R_IN1_PIN | MOTOR_R_IN2_PIN);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = MOTOR_PWM_PERIOD;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = ENABLE;
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM2, &TIM_OCInitStruct);
	TIM_OC2Init(TIM2, &TIM_OCInitStruct);

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

/* 设置左电机速度。Duty > 0 正转，Duty < 0 反转，Duty = 0 停止，范围 -100.0f~100.0f */
void App_PWM_Set_L(float Duty)
{
	uint16_t CCR;

	Duty = Motor_LimitDuty(Duty * MOTOR_SIGN_L);
	if (Duty > 0.0f)
	{
		GPIO_SetBits(GPIOB, MOTOR_L_IN1_PIN);
		GPIO_ResetBits(GPIOB, MOTOR_L_IN2_PIN);
	}
	else if (Duty < 0.0f)
	{
		GPIO_ResetBits(GPIOB, MOTOR_L_IN1_PIN);
		GPIO_SetBits(GPIOB, MOTOR_L_IN2_PIN);
	}
	else
	{
		GPIO_ResetBits(GPIOB, MOTOR_L_IN1_PIN | MOTOR_L_IN2_PIN);
	}

	// 把百分比转换成定时器比较值，例如 30% 对应约 300
	CCR = (uint16_t)(fabsf(Duty) * (float)(MOTOR_PWM_PERIOD + 1U) / 100.0f);
	if (CCR > MOTOR_PWM_PERIOD) CCR = MOTOR_PWM_PERIOD;
	TIM_SetCompare1(TIM2, CCR);
}

/* 设置右电机速度。Duty > 0 正转，Duty < 0 反转，Duty = 0 停止 */
void App_PWM_Set_R(float Duty)
{
	uint16_t CCR;

	Duty = Motor_LimitDuty(Duty * MOTOR_SIGN_R);
	//右电机实际接线方向与左电机相反，这里把 IN1/IN2 对调，
	//使"正占空比 = 车前进"这个约定对左右轮保持一致
	if (Duty > 0.0f)
	{
		GPIO_ResetBits(GPIOB, MOTOR_R_IN1_PIN);
		GPIO_SetBits(GPIOB, MOTOR_R_IN2_PIN);
	}
	else if (Duty < 0.0f)
	{
		GPIO_SetBits(GPIOB, MOTOR_R_IN1_PIN);
		GPIO_ResetBits(GPIOB, MOTOR_R_IN2_PIN);
	}
	else
	{
		GPIO_ResetBits(GPIOB, MOTOR_R_IN1_PIN | MOTOR_R_IN2_PIN);
	}

	CCR = (uint16_t)(fabsf(Duty) * (float)(MOTOR_PWM_PERIOD + 1U) / 100.0f);
	if (CCR > MOTOR_PWM_PERIOD) CCR = MOTOR_PWM_PERIOD;
	TIM_SetCompare2(TIM2, CCR);
}
