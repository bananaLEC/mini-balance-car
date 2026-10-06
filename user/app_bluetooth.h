#ifndef APP_BLUETOOTH_H
#define APP_BLUETOOTH_H

#include "stm32f10x.h"

/* JDY-29 蓝牙透传模块驱动（USART2，PA2-TXD / PA3-RXD，接核心板 J5 蓝牙座）
 * 发送为中断驱动 + 环形缓冲：调用方塞进缓冲区即返回，实际移位发送由 USART2 中断完成，
 * 不会像 USART1 的阻塞式 printf 那样吃掉大半个控制环周期；接收同理，由中断收进缓冲区供上层取 */

/* 与模块的串口波特率，必须和模块设置一致（JDY-29 出厂默认 9600 = AT+BAUD4）。
 * 9600 下 8N1 为 960 字节/秒，一行约 44 字节要占 46ms 空中时间，打印周期不要低于 100ms，
 * 再密会把 256 字节发送缓冲撑满而整条丢弃。改模块速率：接 USB-TTL、PWRC 拉低、未连接时发 AT+BAUD8 */
#define BT_BAUDRATE		(9600u)

void	  App_Bluetooth_Init(void);

/* 非阻塞发送。返回 1 = 已入队；返回 0 = 缓冲区满，本次数据被丢弃
 * （调试数据丢一点没关系，绝不能为了等缓冲区而卡控制环） */
uint8_t   App_Bluetooth_SendBytes(const uint8_t *pData, uint16_t Size);
uint16_t  App_Bluetooth_Printf(const char *Format, ...);

/* 接收侧：返回缓冲区里待读取的字节数；读一个字节，无数据时返回 0 */
uint16_t  App_Bluetooth_RxCount(void);
uint8_t   App_Bluetooth_ReadByte(void);

#endif
