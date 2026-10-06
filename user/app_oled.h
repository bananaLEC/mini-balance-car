#ifndef APP_OLED_H
#define APP_OLED_H

#include "stm32f10x.h"

/*
 * 状态屏：128x64 的 SSD1306 四针模块（原理图 U4），软 I2C 接 PB8=SCL / PB9=SDA，
 * 和江协 OLED 例程的引脚约定一致；MPU6050 在硬件 I2C2（PB10/PB11）上，两条总线互不干扰。
 *
 * 刷新方式是为平衡环让路的：
 *   App_OLED_Task  只把内容画进内存帧缓冲并启动一次传输（不碰 I2C）
 *   App_OLED_Pump  在主循环空闲时每次只往屏幕搬 8 字节（约 0.2ms）
 * 整屏 1024 字节约 25ms 的传输被切成 128 段摊在空闲时间里，
 * 任何一个控制任务都不会被整屏刷新卡住。
 */

void App_OLED_Init(void);
void App_OLED_Task(void);   // 注册进调度器，建议 200ms 周期
void App_OLED_Pump(void);   // 主循环 while(1) 里 Task_Run() 之后调用

#endif
