#include "encoder_test.h"
#include "app_usart1.h"
#include "delay.h"
#include "app_encoder.h"

// 编码器位置测试。轮子架空手转轮子，看 pos_l,pos_r 的增减方向和实际转动是否一致
void Encoder_Test(void)
{
	App_USART1_Init();
	App_Encoder_Init();
	while(1)
	{
		float pos_l = App_Encoder_GetPos_L();
		float pos_r = App_Encoder_GetPos_R();
		
		My_USART_Printf(USART1,"%.2f,%.2f\n",pos_l,pos_r);
		
		Delay(50);
	}
}

static float last_pos_L = 0.0f;
static float last_pos_R = 0.0f;

// M法测速：轮子架空，每 1ms 取一次位置差，omega = 位置差 / 0.001，单位 rad/s。慢速时读数跳变大
void Encoder_M_Method_Test(void)
{
	App_Encoder_Init();
	App_USART1_Init();
	
	while(1)
	{
		Delay(1);
		
		float pos_L = App_Encoder_GetPos_L();
		float pos_R = App_Encoder_GetPos_R();
		
		float M_L = pos_L - last_pos_L;
		float M_R = pos_R - last_pos_R;
		
		float omega_L = M_L / 0.001f;//左轮角速度，rad/s
		float omega_R = M_R / 0.001f;//右轮角速度，rad/s
		
		My_USART_Printf(USART1,"%f,%f,%f,%f\n",pos_L,pos_R,omega_L,omega_R);
		
		last_pos_L = pos_L;
		last_pos_R = pos_R;
		
	}
	
}

// T法测速：轮子架空，用相邻边沿的时间差求速度，慢速下比 M 法稳
void Encoder_T_Method_Test(void)
{
	App_USART1_Init();
	App_Encoder_Init();
	
	while(1)
	{
		Delay(1);
		
		float omega_L = App_Encoder_GetSpeed_L();
		float omega_R = App_Encoder_GetSpeed_R();
		
		My_USART_Printf(USART1,"%f,%f\n",omega_L,omega_R);
	}
	
}
