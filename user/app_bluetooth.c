#include "app_bluetooth.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* 核心板 J5 蓝牙座：1-3V3(模块 VCC) 2-GND 3-PA3(USART2_RX <- 模块 TXD) 4-PA2(USART2_TX -> 模块 RXD)。
 * 模块 STAT/ALED/PWRC 没引出，连接状态读不到，需要时用 AT+STAT 查询 */

/* 环形缓冲区大小必须取 2 的幂，取模运算可以换成按位与 */
#define BT_TX_BUF_SIZE	256u
#define BT_TX_BUF_MASK	(BT_TX_BUF_SIZE - 1u)
#define BT_RX_BUF_SIZE	128u
#define BT_RX_BUF_MASK	(BT_RX_BUF_SIZE - 1u)

static volatile uint8_t  tx_buf[BT_TX_BUF_SIZE];
static volatile uint16_t tx_head;	//写入位置（任务里写）
static volatile uint16_t tx_tail;	//读出位置（中断里读）

static volatile uint8_t  rx_buf[BT_RX_BUF_SIZE];
static volatile uint16_t rx_head;	//写入位置（中断里写）
static volatile uint16_t rx_tail;	//读出位置（任务里读）

void App_Bluetooth_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	USART_InitTypeDef USART_InitStruct = {0};
	NVIC_InitTypeDef NVIC_InitStruct = {0};

	//PA2/PA3 在 APB2 的 GPIOA 上，USART2 本身挂在 APB1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	//PA3-RXD：上拉输入，保证模块没插上时是确定电平
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	USART_InitStruct.USART_BaudRate = BT_BAUDRATE;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2, &USART_InitStruct);

	//接收中断常开：字节到了先存进环形缓冲，不丢
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	//发送中断平时关着，只有缓冲区里有货才开，见 App_Bluetooth_SendBytes
	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	//抢占优先级取 3：0/1 留给控制周期中断，2 是编码器 EXTI
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);

	USART_Cmd(USART2, ENABLE);
}

//把一段字节塞进发送缓冲区，真正的发送由 TXE 中断慢慢做
//返回 1 = 入队成功；0 = 剩余空间不够，整条丢弃（宁丢数据不卡控制环）
uint8_t App_Bluetooth_SendBytes(const uint8_t *pData, uint16_t Size)
{
	uint16_t head = tx_head;
	uint16_t used;
	uint16_t free_space;
	uint16_t i;

	if(Size == 0u) return 1u;

	//head/tail 为 uint16_t 单调累加（靠无符号回绕），中断只动 tail，这里读到的 tail
	//最多旧一个字节，宁可少判一点空间也不能多塞
	used = (uint16_t)(tx_head - tx_tail);
	free_space = (uint16_t)(BT_TX_BUF_SIZE - used);

	if(Size > free_space)
	{
		return 0u;
	}

	for(i = 0u; i < Size; i++)
	{
		tx_buf[head & BT_TX_BUF_MASK] = pData[i];
		head++;
	}

	tx_head = head;   //先推进 head，数据对中断可见

	//最后再开中断：若提前开，中断会看到旧 head 以为缓冲区空而关掉 TXE，余下字节就发不出去
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

	return 1u;
}

uint16_t App_Bluetooth_Printf(const char *Format, ...)
{
	char line[128];
	va_list argptr;
	int len;

	__va_start(argptr, Format);
	len = vsnprintf(line, sizeof(line), Format, argptr);
	__va_end(argptr);

	if(len < 0) return 0u;
	if((uint16_t)len >= sizeof(line)) len = sizeof(line) - 1;  //被截断

	return (uint16_t)App_Bluetooth_SendBytes((const uint8_t *)line, (uint16_t)len);
}

uint16_t App_Bluetooth_RxCount(void)
{
	return (uint16_t)(rx_head - rx_tail);
}

//读一个字节，没数据返回 0（BLE 透传里 0x00 不是有效指令字符，够用）
uint8_t App_Bluetooth_ReadByte(void)
{
	uint8_t data;

	if(rx_tail == rx_head)
	{
		return 0u;
	}

	data = rx_buf[rx_tail & BT_RX_BUF_MASK];
	rx_tail++;

	return data;
}

//USART2 中断：TXE 把发送缓冲里的字节排出去，RXNE 收字节进接收缓冲
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		uint8_t data = (uint8_t)USART_ReceiveData(USART2);
		uint16_t next = (uint16_t)(rx_head + 1u);

		//缓冲区满就丢新字节，旧数据保持连续
		if((uint16_t)(next - rx_tail) <= BT_RX_BUF_SIZE)
		{
			rx_buf[rx_head & BT_RX_BUF_MASK] = data;
			rx_head = next;
		}
		//USART_ReceiveData 内部读 DR，顺带清掉 ORE；不清代 RXNE 中断会被卡死
	}

	if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
	{
		while(tx_tail != tx_head)
		{
			if(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
			{
				return;   //移位寄存器又满了，等下一次 TXE 中断接着发
			}
			USART_SendData(USART2, tx_buf[tx_tail & BT_TX_BUF_MASK]);
			tx_tail++;
		}

		//发完了才关中断，否则最后一字节后面的 TXE 会永远等不到
		USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	}
}
