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

/* ---------- 诊断接口：用来判断 I2C 到底是哪种失败 ---------- */

/* 累计的读失败次数 / 总线抢救次数 / 抢救后总线是否仍被拉住。
 * pBusHeld 返回 1 表示硬件层面被拉住，软件救不回来，要查供电与接线。
 * 参数可传 0 表示不关心 */
void App_MPU6050_GetDiag(uint16_t *pFails, uint16_t *pResets, uint8_t *pBusHeld);

/* 最后一次失败的现场：返回值(-1 无应答 / -2 数据被拒 / -3 超时) 与 SR1/SR2。
 * SR1/SR2 是 16 位寄存器（错误标志在 bit8~bit11），所以用 uint16_t。
 * 参数可传 0 表示不关心 */
void App_MPU6050_GetLastError(int *pErr, uint16_t *pSr1, uint16_t *pSr2);

/* 把上面的现场翻译成一句英文短句，可直接显示。
 * 返回静态常量字符串，不需要释放 */
const char *App_MPU6050_DescribeError(void);

#endif
