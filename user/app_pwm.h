#ifndef APP_PWM_H
#define APP_PWM_H

#include "stm32f10x.h"                  // Device header


void App_PWM_Init(void);
void App_PWM_Set_L(float Duty);
void App_PWM_Set_R(float Duty);

#endif
