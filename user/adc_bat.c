#include "adc_bat.h"

static volatile float vbat = 0.0f;

void Adc_Bat_Init(void)
{
	/* ADC 功能停用；PB2 只是预留 GPIO，不是 ADC 通道 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

float Adc_Bat_Get(void)
{
	/* ADC 已停用，接口保留以兼容原有调用 */
	vbat = 0.0f;
	return vbat;
}
