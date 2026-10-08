/* bsp_i2c.h - 硬件 I2C 主机接口
 *
 * 依据：
 *   ST RM0008 第 26 章「Inter-integrated circuit (I2C) interface」
 *   ST AN2824《STM32F10xxx I2C optimized examples》
 *   ST ES096（STM32F103 勘误手册）—— 总线锁死与恢复
 *
 * 返回值统一约定：
 *    0  成功
 *   -1  寻址失败（从机没应答地址）
 *   -2  数据被拒收（从机没应答数据）
 *   -3  超时（总线被拉住 / 外设状态机异常）
 *
 * ⚠️ 调用方**必须判断返回值**。I2C 挂死时返回 -3，若不判断就会拿旧数据接着跑。
 *
 * 本板接线（江协科技平衡车控制板 V2.0，依据本项目 user/app_mpu6050.c 整理）：
 *   MPU6050 走 I2C2：SCL = PB10，SDA = PB11。
 *   注意 I2C1 的默认引脚 PB6/PB7 在板上是右编码器，不要占用。
 */

#ifndef _BSP_I2C_H_
#define _BSP_I2C_H_

#include "stm32f10x.h"

/* 本板从机挂在哪条总线上，供 My_I2C_ResetBus 查询引脚用。
 * I2Cx 传了表里没有的总线，ResetBus 会直接返回（不做事） */
#define BSP_I2C_MPU6050         I2C2

/* 向从机写多个字节。Addr 左对齐：A6..A0 + 0 */
int My_I2C_SendBytes(I2C_TypeDef *I2Cx, uint8_t Addr, const uint8_t *pData, uint16_t Size);

/* 从从机读多个字节。Addr 左对齐 */
int My_I2C_ReceiveBytes(I2C_TypeDef *I2Cx, uint8_t Addr, uint8_t *pBuffer, uint16_t Size);

/* 读从机某个寄存器：起始 → 地址+写 → 寄存器号 → 重复起始 → 地址+读 → 数据 → 停止 */
int My_I2C_RegReadBytes(I2C_TypeDef *I2Cx, uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Size);

/* 写从机某个寄存器：起始 → 地址+写 → 寄存器号 → 数据 → 停止 */
int My_I2C_MemWriteBytes(I2C_TypeDef *I2Cx, uint8_t Addr, uint8_t Reg, const uint8_t *pData, uint16_t Size);

/* 总线卡死（从机把 SDA 拉住不放，BUSY 一直为 1）时的抢救。
 *
 * 为什么需要它：从机在传输中途复位或掉电，会把 SDA 一直拉低。
 * 这时复位主机 I2C 外设没有用 —— 从机仍在等时钟，必须由主机补发时钟脉冲
 * 把从机没吐完的那几位冲干净，它才会释放 SDA（依据 ST ES096 / AN2824）。
 *
 * 做法：关 I2C 外设 → 两线切成开漏 GPIO → 打 SCL 脉冲直到从机放开 SDA
 *       → 补一个停止位 → 交还外设并**真正复位外设再重新 I2C_Init**。
 *
 * 最后这步复位是必须的：只重新使能外设的话，SR1/SR2 里锁存的 AF/BUSY 还在，
 * 下一次传输会立刻又失败，表现为永远救不回来。
 *
 * 返回值： 0 已执行抢救 / -1 这条总线不在本板引脚表里，未做事 */
int My_I2C_ResetBus(I2C_TypeDef *I2Cx);

/* 上一次 My_I2C_ResetBus 之后，SCL/SDA 是否仍然被拉在低电平。
 *
 * 用来把两类故障分开：
 *   1 = 总线仍被拉住 —— 硬件问题：从机掉电/复位把线钳住、器件损坏、走线短路。
 *       打时钟脉冲和复位外设都救不回来
 *   0 = 总线已松开 —— 说明「锁死」已被解开。但**不等于通信就正常了**：
 *       如果每次都能救开、可读几次又失败，那是链路本身不稳
 *       （典型是总线速率对上升沿要求过高、上拉偏弱、线太长），
 *       要往「降速、加上拉、缩短走线」的方向查，而不是继续加抢救逻辑
 * 没抢救过时为 0 */
uint8_t My_I2C_BusWasHeldLow(void);

#endif
