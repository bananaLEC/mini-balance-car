/* bsp_usart.h - 串口收发接口（阻塞式）
 *
 * 依据：ST RM0008 第 27 章「Universal synchronous asynchronous receiver transmitter」
 *   - USART_SR 的 TXE / TC / RXNE / ORE 标志语义
 *   - USART_BRR 波特率分频（含小数部分）
 *   - USART_DR 数据寄存器
 * 另参考：Keil MDK microlib 文档（工程链接选项为 --library_type=microlib）
 *
 * 本模块只做「同步阻塞」收发，够用来打日志。
 * 需要不阻塞控制环的收发，请走 user/app_bluetooth.c 的中断 + 环形缓冲方案。
 */

#ifndef _BSP_USART_H_
#define _BSP_USART_H_

#include "stm32f10x.h"

/* 行分隔符，供 My_USART_ReceiveLine 使用 */
#define LINE_SEPERATOR_CR   0x00   /* 回车 \r        */
#define LINE_SEPERATOR_LF   0x01   /* 换行 \n        */
#define LINE_SEPERATOR_CRLF 0x02   /* 回车 + 换行 \r\n */

/* 返回值：见各函数注释 */

/* 发一个字节。阻塞到字节进入发送寄存器（不等发送完成） */
void My_USART_SendByte(USART_TypeDef *USARTx, const uint8_t Data);

/* 发一串字节。阻塞到**最后一个字节真正发完**（等 TC），
 * 这样函数返回后立刻翻转 RS485 方向脚或断电都不会截断最后一个字节 */
void My_USART_SendBytes(USART_TypeDef *USARTx, const uint8_t *pData, uint16_t Size);

/* 发一个字符 */
void My_USART_SendChar(USART_TypeDef *USARTx, const char C);

/* 发一个以 '\0' 结尾的字符串 */
void My_USART_SendString(USART_TypeDef *USARTx, const char *Str);

/* 格式化打印，单次输出上限 127 字节（超出被截断）。
 *
 * ⚠️ 不要在控制环里调用：microlib 下 %f 走软件浮点，一次要几毫秒，
 *    会把控制周期拖垮。需要打印时单独开一个 >= 50ms 的任务。
 *    打印整数（%d）比 %f 快得多，调试输出优先用整数 */
void My_USART_Printf(USART_TypeDef *USARTx, const char *Format, ...);

/* 读一个字节。阻塞等到有数据为止 */
uint8_t My_USART_ReceiveByte(USART_TypeDef *USARTx);

/* 读最多 Size 个字节。Timeout 单位 ms，负数表示无限等待。
 * 返回值：实际读到的字节数（可能小于 Size，说明超时了） */
uint16_t My_USART_ReceiveBytes(USART_TypeDef *USARTx, uint8_t *pDataOut, uint16_t Size, int Timeout);

/* 读到行分隔符为止。LineSeperator 取 LINE_SEPERATOR_CR / LF / CRLF 之一。
 * MaxLength 含结尾的 '\0'。Timeout 单位 ms，负数表示无限等待。
 * 返回值： 0 读到一整行
 *         -1 超时（缓冲区里是已读到的部分，仍然以 '\0' 结尾）
 *         -2 到 MaxLength 还没遇到分隔符，或参数非法 */
int My_USART_ReceiveLine(USART_TypeDef *USARTx, char *pStrOut, uint16_t MaxLength,
                         uint16_t LineSeperator, int Timeout);

#endif
