/* bsp_usart.c - 串口收发实现（阻塞式）
 *
 * 依据：ST RM0008 第 27 章（USART）
 *   TXE  置位 = 发送数据寄存器空，可以写下一个字节
 *   TC   置位 = 最后一个字节已经移出移位寄存器，「真的发完了」
 *   RXNE 置位 = 接收数据寄存器非空，读 DR 会顺带清掉 RXNE 与 ORE
 *
 * 原实现里有两处问题，这里都改掉了：
 *   1. My_USART_ReceiveBytes / ReceiveLine 里 expireTime 在 Timeout < 0 时未初始化，
 *      而 while(Timeout < 0 || GetTick() < expireTime) 因为短路求值恰好没用到它 ——
 *      编译器换个求值顺序就是未定义行为。现在用「剩余时间」表达，不存在未初始化路径。
 *   2. 原来的 __weak 没有实际意义（工程里没有任何重定义），却让每个调用点都走一遍
 *      弱符号跳转。现在改成普通函数。
 */

#include "bsp_usart.h"
#include "bsp_delay.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* 格式化缓冲：放栈上。128 字节在小车这种栈空间充裕（1KB+）的场景是安全的 */
#define USART_PRINTF_BUF_SIZE   (128u)

/* 把「超时」统一表达成剩余毫秒数：
 *   > 0  还剩这么多 ms
 *   == 0 已经超时
 *   < 0  无限等待（由 USART_WaitForever 表示） */
#define USART_WAIT_FOREVER      (-1)

/* 判断「带超时的轮询」是否还应该继续等。
 * 用无符号减法比较时间戳，天然处理 GetTick() 的 49.7 天回绕 */
static int USART_Timeout(uint32_t start_tick, int timeout_ms)
{
	if(timeout_ms < 0)
	{
		return 1;   /* 无限等待，永远继续 */
	}

	return ((uint32_t)(GetTick() - start_tick) < (uint32_t)timeout_ms) ? 1 : 0;
}

void My_USART_SendByte(USART_TypeDef *USARTx, const uint8_t Data)
{
	/* 等发送寄存器空，再写数据 */
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
	{
		/* 空等 */
	}

	USART_SendData(USARTx, (uint16_t)Data);
}

void My_USART_SendBytes(USART_TypeDef *USARTx, const uint8_t *pData, uint16_t Size)
{
	uint16_t i;

	if(Size == 0u)
	{
		return;
	}

	for(i = 0u; i < Size; i++)
	{
		My_USART_SendByte(USARTx, pData[i]);
	}

	/* 等 TC 而不是 TXE：TXE 只说明「寄存器可以收了」，最后一个字节
	 * 可能还在移位寄存器里。等 TC 才能保证整串都上了线 */
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
	{
		/* 空等 */
	}
}

void My_USART_SendChar(USART_TypeDef *USARTx, const char C)
{
	My_USART_SendByte(USARTx, (uint8_t)C);
}

void My_USART_SendString(USART_TypeDef *USARTx, const char *Str)
{
	if(Str == 0)
	{
		return;
	}

	My_USART_SendBytes(USARTx, (const uint8_t *)Str, (uint16_t)strlen(Str));
}

void My_USART_Printf(USART_TypeDef *USARTx, const char *Format, ...)
{
	char    buffer[USART_PRINTF_BUF_SIZE];
	va_list args;
	int     len;

	if(Format == 0)
	{
		return;
	}

	va_start(args, Format);
	len = vsnprintf(buffer, sizeof(buffer), Format, args);
	va_end(args);

	if(len <= 0)
	{
		return;
	}

	/* vsnprintf 返回的是「本该写入的长度」，可能大于缓冲；
	 * 那种情况内容已被截断，长度要按实际容量算 */
	if((size_t)len >= sizeof(buffer))
	{
		len = (int)(sizeof(buffer) - 1u);
	}

	My_USART_SendBytes(USARTx, (const uint8_t *)buffer, (uint16_t)len);
}

uint8_t My_USART_ReceiveByte(USART_TypeDef *USARTx)
{
	while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET)
	{
		/* 空等 */
	}

	/* 读 DR 顺带清掉 RXNE 和 ORE */
	return (uint8_t)USART_ReceiveData(USARTx);
}

uint16_t My_USART_ReceiveBytes(USART_TypeDef *USARTx, uint8_t *pDataOut, uint16_t Size, int Timeout)
{
	uint32_t start_tick;
	uint16_t count = 0u;

	if((pDataOut == 0) || (Size == 0u))
	{
		return 0u;
	}

	start_tick = GetTick();

	while(USART_Timeout(start_tick, Timeout) != 0)
	{
		if(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == SET)
		{
			pDataOut[count] = (uint8_t)USART_ReceiveData(USARTx);
			count++;

			if(count >= Size)
			{
				break;   /* 收满了 */
			}
		}
	}

	return count;
}

int My_USART_ReceiveLine(USART_TypeDef *USARTx, char *pStrOut, uint16_t MaxLength,
                         uint16_t LineSeperator, int Timeout)
{
	uint32_t start_tick;
	uint16_t count = 0u;
	char     prev = '\0';
	int      ret = -1;   /* 默认按「超时」返回，读到分隔符再改成 0 */

	/* MaxLength 要能装下至少一个字符 + '\0'；
	 * CRLF 还会多占一个字节，所以至少要 3 */
	if((pStrOut == 0) || (MaxLength < 2u))
	{
		return -2;
	}

	if((LineSeperator == LINE_SEPERATOR_CRLF) && (MaxLength < 3u))
	{
		return -2;
	}

	start_tick = GetTick();

	while(USART_Timeout(start_tick, Timeout) != 0)
	{
		char c;

		if(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET)
		{
			continue;
		}

		c = (char)USART_ReceiveData(USARTx);

		pStrOut[count] = c;
		count++;

		if(LineSeperator == LINE_SEPERATOR_CR)
		{
			if(c == '\r')
			{
				ret = 0;
				break;
			}
		}
		else if(LineSeperator == LINE_SEPERATOR_LF)
		{
			if(c == '\n')
			{
				ret = 0;
				break;
			}
		}
		else   /* CRLF：必须是 "\r\n" 这个序列，单独的 '\r' 不算 */
		{
			if((prev == '\r') && (c == '\n'))
			{
				ret = 0;
				break;
			}
		}

		prev = c;

		/* 留一个字节给结尾的 '\0'，所以最多收 MaxLength-1 个字符 */
		if(count >= (uint16_t)(MaxLength - 1u))
		{
			ret = -2;   /* 行太长，没等到分隔符 */
			break;
		}
	}

	pStrOut[count] = '\0';

	return ret;
}
