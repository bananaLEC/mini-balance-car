#ifndef __ADC_BAT_H
#define __ADC_BAT_H
#include "stm32f10x.h"                  // Device header

void Adc_Bat_Init(void);
float Adc_Bat_Get(void);

#endif
