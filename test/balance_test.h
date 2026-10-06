#ifndef BALANCE_TEST_H
#define BALANCE_TEST_H

#include "stm32f10x.h"                  // Device header

/* 【第四个测】机械中位标定：车拿在手里、轮子悬空，读出中位填进 app_balance.c */
void Balance_Center_Test(void);

/* 【第二个测】平衡环符号验证：车拿在手里、轮子悬空 */
void Balance_Sign_Test(void);

/* 【第一个测】电机调速环测试：轮子架空或放地上 */
void Motor_Speed_Test(void);

#endif
