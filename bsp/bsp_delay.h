/* bsp_delay.h - 基于 SysTick 的延时与计时接口
 *
 * 依据 ARM DDI 0337《Cortex-M3 技术参考手册》第 8 章（SysTick）。
 *
 * 设计要点：
 *   1. 毫秒时间戳由 SysTick 中断维护，**写入者只有中断**，不会重复计数。
 *   2. 微秒时间戳 = 毫秒戳 × 1000 + (SysTick->LOAD - SysTick->VAL) 换算出的
 *      本毫秒内已过时间。因为 SysTick 重装与中断递增是同一事件，
 *      「先读毫秒戳再读 VAL」不会产生跳变，也不依赖重试。
 *   3. 不读写任何中断控制寄存器，因此**在临界区内调用是安全的**，
 *      不会改变调用方的中断状态。
 *
 * 与旧实现的差别（这两条是本次重构修掉的真实缺陷）：
 *   - 旧实现把 SysTick 计数的自增同时放在中断和 GetUs() 里，两边都加，
 *     同一拍可能被计两次，时间轴会跳；
 *   - 旧 GetUs() 内部用 __disable_irq()/__enable_irq() 包住读数，
 *     在临界区里调用会意外开中断，i2c.c 为此另外写了一套绕过 GetUs 的
 *     等待函数。现在这些问题从根上没有了。
 *
 * ⚠️ 已知边界：毫秒计数依赖 SysTick 中断。调用方若长时间关中断（> 1ms），
 *    这段时间不会被计入时间轴。本项目里唯一的临界区在 app_encoder.c，
 *    只包一次结构体拷贝，远小于 1ms。
 */

#ifndef _DELAY_H_
#define _DELAY_H_

#include "stm32f10x.h"

/* 首次调用完成 SysTick 初始化；重复调用直接返回，可放心到处调 */
void Delay_Init(void);

/* 毫秒级延时，单位 ms。不要在中断里调用（会死等） */
void Delay(uint32_t ms);

/* 微秒级延时，单位 us。不要在中断里调用（会死等） */
void DelayUs(uint32_t us);

/* 当前系统时间，单位 ms。开机后约 49.7 天回绕一次，做差值运算时按无符号处理即可 */
uint32_t GetTick(void);

/* 当前系统时间，单位 us。分辨率约 1us（72MHz 下 1us = 72 个周期）。
 * 64 位宽，实际上不会回绕；做差值运算时按无符号处理即可 */
uint64_t GetUs(void);

/* 供 SysTick 中断服务函数调用，用于推进毫秒时间戳。
 * 不要在别处调用 —— 毫秒计数只允许有一个写入者，就是中断 */
void Delay_TickInc(void);

#endif

