#include "pwm_test.h"
#include "app_pwm.h"
#include "delay.h"



// 电机 PWM 测试：开环给固定占空比让轮子转，轮子必须架空
// 当前只跑 30% 一档，60%/90% 两档在下面的注释里，取消注释即可启用；结束后占空比归 0
void PWM_Test(void)
{
	// 30% 占空比运行 2 秒
	App_PWM_Set_L(30);
	App_PWM_Set_R(30);
	
	Delay(2000);
	
/*	// 60% 运行 2 秒
	App_PWM_Set_L(60);
	App_PWM_Set_R(60);
	
	Delay(2000);
	
	// 90% 运行 2 秒
	App_PWM_Set_L(90);
	App_PWM_Set_R(90);
	
	Delay(2000);
*/
	// 结束后占空比归 0，电机停转
	App_PWM_Set_L(0);
	App_PWM_Set_R(0);
}
	

