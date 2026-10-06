#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include "stm32f10x.h"                  // Device header

void Motor_Sign_Test(void);
void Motor_Crawl_Test(void);   //低速死区/爬行测试：轮子必须架空，查"咯噔"是不是静摩擦造成的

#endif
