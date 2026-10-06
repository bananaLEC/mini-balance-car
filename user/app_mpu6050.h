#ifndef APP_MPU6050_H
#define APP_MPU6050_H

#include "stm32f10x.h"                  // Device header

void App_MPU6050_Init(void);

void App_MPU6050_Update(void);

float App_MPU6050_GetAx(void);
float App_MPU6050_GetAy(void);
float App_MPU6050_GetAz(void);

float App_MPU6050_GetTemperature(void);

float App_MPU6050_GetGx(void);
float App_MPU6050_GetGy(void);
float App_MPU6050_GetGz(void);

uint8_t App_MPU6050_GetID(void);   //读WHO_AM_I，正常返回0x68

/* 传感器数据是否有效：I2C 连续读失败到一定次数后返回 0。
 * 平衡环必须看这个标志，返回 0 时立刻停机，不能拿旧数据接着算 */
uint8_t App_MPU6050_IsOk(void);
#endif
