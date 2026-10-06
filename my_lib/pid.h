#ifndef PID_H
#define PID_H

#include "stm32f10x.h"                  // Device header

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float Sp;// 设定值

	float UpperLimit;
	float LowerLimit;

	float IntUpperLimit;// 积分值限幅，和输出限幅量纲不同，要分开配
	float IntLowerLimit;

	uint64_t t_k_1; 	// 上次运算时刻，单位微秒
	float err_k_1;  	// 上次误差
	float err_int_k_1;  // 上次积分值
	uint8_t FirstRun;	// 首拍标志，为 1 时跳过积分和微分项

}PID_TypeDef;


void PID_Init(PID_TypeDef *PID, float Kp, float Ki, float Kd);
void PID_changeSp(PID_TypeDef *PID, float Sp);
void PID_LimitConfig(PID_TypeDef *PID, float Upper, float Lower);
void PID_IntLimitConfig(PID_TypeDef *PID, float Upper, float Lower);
void PID_Reset(PID_TypeDef *PID);
float PID_computer(PID_TypeDef *PID, float FB);


#endif
