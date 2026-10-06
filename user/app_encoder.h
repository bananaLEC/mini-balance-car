#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include "stm32f10x.h"                  // Device header

void App_Encoder_Init(void);
float App_Encoder_GetPos_L(void);
float App_Encoder_GetPos_R(void);
void App_Encoder_ClearLeft(void);
void App_Encoder_ClearRight(void);
void App_Encoder_ClearAll(void);

float App_Encoder_GetSpeed_L(void);
float App_Encoder_GetSpeed_R(void);
#endif
