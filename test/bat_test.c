#include "bat_test.h"
#include "app_usart1.h"
#include "adc_bat.h"
#include "delay.h"

void Bat_Test(void)
{
	App_USART1_Init();
	Adc_Bat_Init();
	while(1)
	{
		// 每 10 ms 打印一个电压点，单位 V；ADC 已停用，读数恒为 0
		float volt = Adc_Bat_Get();
		
		My_USART_Printf(USART1,"ADC_DISABLED_PB2=%.3fV\r\n",volt);
		
		Delay(10);
	}
}
