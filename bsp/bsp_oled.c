/* bsp_oled.c - SSD1306 OLED 显示驱动实现
 *
 * 依据：Solomon Systech SSD1306 数据手册 Rev 1.1
 *
 * 显存布局（SSD1306 第 8.7 节「Graphic Display Data RAM」）：
 *   128 列 x 64 行，按页组织：共 8 页，每页 8 行。
 *   页内每个字节对应「某一列上连续的 8 个像素」，bit0 在最上面。
 *   所以帧缓冲下标 = page * 128 + x，共 1024 字节。
 *
 * 分段刷新的实现思路：
 *   整帧 1024 字节在 100kHz 软 I2C 上要 100ms 以上，一次性发会把控制环卡死。
 *   这里把「正在搬运」的状态存在句柄里，每次只发 OLED_CHUNK_BYTES 个字节，
 *   每段之间让出 CPU 给主循环里的任务调度器。
 *   因为每段都要重新发一次「设置页地址」命令，显存写入地址不会串。
 */

#include "bsp_oled.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "bsp_font6x8.h"

/* I2C 控制字节：SSD1306 规定每个 I2C 传输都以一个控制字节开头 */
#define SSD1306_CTRL_CMD        (0x00u)   /* 后面全是命令 */
#define SSD1306_CTRL_DATA       (0x40u)   /* 后面全是显存数据 */

/* 常用命令 */
#define SSD1306_CMD_DISPLAY_OFF         (0xAEu)
#define SSD1306_CMD_DISPLAY_ON          (0xAFu)
#define SSD1306_CMD_SET_CLOCK_DIV       (0xD5u)
#define SSD1306_CMD_SET_MULTIPLEX       (0xA8u)
#define SSD1306_CMD_SET_DISPLAY_OFFSET  (0xD3u)
#define SSD1306_CMD_SET_START_LINE      (0x40u)   /* 低 6 位是行号，0x40 = 第 0 行 */
#define SSD1306_CMD_CHARGE_PUMP         (0x8Du)
#define SSD1306_CMD_SET_MEMORY_MODE     (0x20u)
#define SSD1306_CMD_SET_COL_ADDR        (0x21u)   /* 设置列地址范围（水平寻址模式用） */
#define SSD1306_CMD_SET_PAGE_ADDR       (0x22u)   /* 设置页地址范围（水平寻址模式用） */
#define SSD1306_CMD_SEG_REMAP           (0xA1u)   /* A0=不重映射，A1=左右镜像 */
#define SSD1306_CMD_COM_SCAN_DIR        (0xC8u)   /* C0=正序，C8=反序（上下翻转） */
#define SSD1306_CMD_SET_COM_PINS        (0xDAu)
#define SSD1306_CMD_SET_CONTRAST        (0x81u)
#define SSD1306_CMD_SET_PRECHARGE       (0xD9u)
#define SSD1306_CMD_SET_VCOMH           (0xDBu)
#define SSD1306_CMD_DEACTIVATE_SCROLL   (0x2Eu)
#define SSD1306_CMD_ENTIRE_DISPLAY_ON   (0xA4u)   /* A4=按显存显示，A5=全亮 */
#define SSD1306_CMD_NORMAL_DISPLAY      (0xA6u)   /* A6=正常，A7=反白 */

/* 子参数 */
#define SSD1306_CLOCK_DIV_0x80          (0x80u)
#define SSD1306_MULTIPLEX_64            (0x3Fu)   /* 复用比 = 64 行 - 1 */
#define SSD1306_DISPLAY_OFFSET_0        (0x00u)
#define SSD1306_CHARGE_PUMP_ON          (0x14u)   /* bit2=1 打开内部电荷泵 */
#define SSD1306_MEMORY_MODE_HORIZONTAL  (0x00u)   /* 水平寻址模式 */
#define SSD1306_COM_PINS_ALT            (0x12u)   /* 交替 COM 引脚配置，适配 128x64 */
#define SSD1306_CONTRAST_DEFAULT        (0xCFu)
#define SSD1306_PRECHARGE_DEFAULT       (0xF1u)
#define SSD1306_VCOMH_DEFAULT           (0x20u)

/* 字符画到行尾后的换行处理：光标回到最左，Y 往下走一个字符高度 */
#define OLED_LINE_HEIGHT                ((int16_t)FONT6X8_HEIGHT)

/* 格式化缓冲，放栈上 */
#define OLED_PRINTF_BUF_SIZE            (64u)

/* ---------------------------------------------------------------------------
 * 底层通信
 * ------------------------------------------------------------------------- */

/* 把帧缓冲里 [start, start+chunk) 这一段写进屏幕。
 *
 * 显存处于水平寻址模式（初始化时设的 0x20,0x00），配合 0x21/0x22 指定一个
 * 矩形窗口：写入的数据会在这个窗口里自动按列递增、行尾自动翻页，
 * 不需要逐段手工维护地址指针。
 *
 * 每段都重设一次窗口，是为了让每段都能独立定位 —— 万一某段失败，
 * SSD1306 的地址指针会停在半途，只有重新设窗口才能拉回来。
 *
 * 用固定的 OLED_CHUNK_BYTES 大小的栈缓冲，所以本函数不依赖大栈。
 * 返回 0 成功 / 非 0 失败 */
static int OLED_WriteChunk(OLED_TypeDef *OLED, uint16_t Start, uint16_t Chunk)
{
	uint8_t cmd[7];
	uint8_t buffer[OLED_CHUNK_BYTES + 1u];

	cmd[0] = SSD1306_CTRL_CMD;
	cmd[1] = SSD1306_CMD_SET_COL_ADDR;                        /* 0x21 列地址范围 */
	cmd[2] = (uint8_t)(Start % OLED_WIDTH);                   /* 起始列 */
	cmd[3] = (uint8_t)(OLED_WIDTH - 1u);                      /* 结束列固定到行尾 */
	cmd[4] = SSD1306_CMD_SET_PAGE_ADDR;                       /* 0x22 页地址范围 */
	cmd[5] = (uint8_t)(Start / OLED_WIDTH);                   /* 起始页 */
	cmd[6] = (uint8_t)((Start + Chunk - 1u) / OLED_WIDTH);    /* 结束页 */

	if(OLED->Write(OLED->Addr, cmd, sizeof(cmd)) != 0)
	{
		return OLED_ERR_I2C;
	}

	buffer[0] = SSD1306_CTRL_DATA;
	memcpy(&buffer[1], &OLED->Frame[Start], Chunk);

	return (OLED->Write(OLED->Addr, buffer, (uint16_t)(Chunk + 1u)) != 0)
	       ? OLED_ERR_I2C : OLED_OK;
}

/* 把「一整个帧的显存」同步写进屏幕。
 * 分段发送而不是拼一个 1024 字节的大缓冲：栈占用固定在
 * OLED_CHUNK_BYTES 级别，不依赖启动文件给的 Stack_Size。
 * 上电初始化时用（此时控制环还没跑起来），慢一点无所谓 */
static int OLED_FlushAll(OLED_TypeDef *OLED)
{
	uint16_t remaining = OLED_FRAME_BYTES;
	uint16_t offset = 0u;

	while(remaining > 0u)
	{
		uint16_t chunk = (remaining > OLED_CHUNK_BYTES) ? OLED_CHUNK_BYTES : remaining;

		if(OLED_WriteChunk(OLED, offset, chunk) != OLED_OK)
		{
			return OLED_ERR_I2C;
		}

		offset    = (uint16_t)(offset + chunk);
		remaining = (uint16_t)(remaining - chunk);
	}

	return OLED_OK;
}

/* ---------------------------------------------------------------------------
 * 画点
 * ------------------------------------------------------------------------- */

static void OLED_SetPixel(OLED_TypeDef *OLED, int16_t X, int16_t Y, uint8_t On)
{
	uint16_t index;
	uint8_t  mask;

	if((X < 0) || (X >= (int16_t)OLED_WIDTH) || (Y < 0) || (Y >= (int16_t)OLED_HEIGHT))
	{
		return;   /* 屏幕外直接丢弃，调用方不必自己判边界 */
	}

	index = (uint16_t)(((uint16_t)Y / 8u) * OLED_WIDTH + (uint16_t)X);
	mask  = (uint8_t)(1u << ((uint8_t)Y & 0x07u));

	if(On != 0u)
	{
		OLED->Frame[index] |= mask;
	}
	else
	{
		OLED->Frame[index] = (uint8_t)(OLED->Frame[index] & (uint8_t)(~mask));
	}
}

/* ---------------------------------------------------------------------------
 * 画字符
 * 字模是「按列取模、bit0 在最上面」，与屏幕显存的纵向排列一致 */
static int16_t OLED_DrawChar(OLED_TypeDef *OLED, char C)
{
	const uint8_t *glyph = Font6x8_GetGlyph(C);
	int16_t        base_x = OLED->CursorX;
	int16_t        base_y = OLED->CursorY;
	uint8_t        col;
	uint8_t        row;
	int16_t        used = 0;

	/* 先算出这个字符实际占了几个非空列，用于「比例前进」 */
	for(col = 0u; col < FONT6X8_WIDTH; col++)
	{
		if(glyph[col] != 0u)
		{
			used = (int16_t)(col + 1u);
		}
	}

	for(col = 0u; col < FONT6X8_WIDTH; col++)
	{
		for(row = 0u; row < FONT6X8_HEIGHT; row++)
		{
			uint8_t on = (uint8_t)((glyph[col] >> row) & 0x01u);

			/* 字模里没点亮的位置也显式清掉：
			 * 状态屏是整屏重画的，覆盖式绘制能保证不留上一帧的残迹 */
			OLED_SetPixel(OLED, (int16_t)(base_x + col), (int16_t)(base_y + row), on);
		}
	}

	/* 前进量 = 实际用到的列数 + 1 列字间距（空格无任何列，前进 1+1 列） */
	return (int16_t)(used + 1);
}

/* 光标前进，必要时换行 / 停止 */
static void OLED_AdvanceCursor(OLED_TypeDef *OLED, int16_t Step)
{
	OLED->CursorX = (int16_t)(OLED->CursorX + Step);

	if(OLED->CursorX >= (int16_t)OLED_WIDTH)
	{
		OLED->CursorX = 0;
		OLED->CursorY = (int16_t)(OLED->CursorY + OLED_LINE_HEIGHT);
	}
}

/* ---------------------------------------------------------------------------
 * 对外接口
 * ------------------------------------------------------------------------- */

int OLED_Bind(OLED_TypeDef *OLED, OLED_WriteCallback_t Write, uint8_t Addr)
{
	if((OLED == 0) || (Write == 0))
	{
		return OLED_ERR_PARAM;
	}

	memset(OLED, 0, sizeof(OLED_TypeDef));

	OLED->Write = Write;
	OLED->Addr  = (Addr != 0u) ? Addr : (uint8_t)OLED_I2C_ADDR;
	OLED->Bound = 1u;

	return OLED_OK;
}

void OLED_Clear(OLED_TypeDef *OLED)
{
	if(OLED == 0)
	{
		return;
	}

	memset(OLED->Frame, 0, OLED_FRAME_BYTES);
}

void OLED_SetCursor(OLED_TypeDef *OLED, int16_t X, int16_t Y)
{
	if(OLED == 0)
	{
		return;
	}

	/* 夹到屏幕范围内，避免调用方算出越界坐标。
	 * 上限取 WIDTH-1 / HEIGHT-1：坐标本身必须落在屏内，
	 * 夹到 WIDTH / HEIGHT 会让光标停在屏外，后续画点全被丢弃 */
	if(X < 0)
	{
		X = 0;
	}
	else if(X > (int16_t)(OLED_WIDTH - 1u))
	{
		X = (int16_t)(OLED_WIDTH - 1u);
	}

	if(Y < 0)
	{
		Y = 0;
	}
	else if(Y > (int16_t)(OLED_HEIGHT - 1u))
	{
		Y = (int16_t)(OLED_HEIGHT - 1u);
	}

	OLED->CursorX = X;
	OLED->CursorY = Y;
}

void OLED_DrawString(OLED_TypeDef *OLED, const char *Str)
{
	if((OLED == 0) || (Str == 0))
	{
		return;
	}

	while(*Str != '\0')
	{
		/* 画到底部就不再画了：继续画只会白白耗时间，屏幕上也看不到 */
		if(OLED->CursorY >= (int16_t)OLED_HEIGHT)
		{
			return;
		}

		OLED_AdvanceCursor(OLED, OLED_DrawChar(OLED, *Str));
		Str++;
	}
}

void OLED_Printf(OLED_TypeDef *OLED, const char *Format, ...)
{
	char    buffer[OLED_PRINTF_BUF_SIZE];
	va_list args;
	int     len;

	if((OLED == 0) || (Format == 0))
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

	OLED_DrawString(OLED, buffer);
}

int OLED_StartSendBuffer(OLED_TypeDef *OLED)
{
	if((OLED == 0) || (OLED->Bound == 0u) || (OLED->Write == 0))
	{
		return OLED_ERR_PARAM;
	}

	if(OLED->Busy != 0u)
	{
		return OLED_ERR_PARAM;   /* 上一帧还没搬完 */
	}

	OLED->Progress = 0u;
	OLED->Busy     = 1u;

	return OLED_OK;
}

int OLED_Pump(OLED_TypeDef *OLED, uint8_t *pMoreOut)
{
	uint16_t remaining;
	uint16_t chunk;

	if(pMoreOut != 0)
	{
		*pMoreOut = 0u;
	}

	if((OLED == 0) || (pMoreOut == 0) || (OLED->Bound == 0u) || (OLED->Write == 0))
	{
		return OLED_ERR_PARAM;
	}

	if(OLED->Busy == 0u)
	{
		return OLED_OK;   /* 没有正在进行的传输，什么都不用做 */
	}

	remaining = (uint16_t)(OLED_FRAME_BYTES - OLED->Progress);
	chunk     = (remaining > OLED_CHUNK_BYTES) ? OLED_CHUNK_BYTES : remaining;

	if(OLED_WriteChunk(OLED, OLED->Progress, chunk) != OLED_OK)
	{
		/* 传输失败就放弃这一帧，把状态清干净，免得永远卡在 Busy 里。
		 * 下一帧开始时每段都会重设窗口，所以不会留下错位 */
		OLED->Busy     = 0u;
		OLED->Progress = 0u;
		return OLED_ERR_I2C;
	}

	OLED->Progress = (uint16_t)(OLED->Progress + chunk);

	if(OLED->Progress >= OLED_FRAME_BYTES)
	{
		OLED->Busy = 0u;
		*pMoreOut  = 0u;
	}
	else
	{
		*pMoreOut = 1u;
	}

	return OLED_OK;
}

uint8_t OLED_IsBusy(OLED_TypeDef *OLED)
{
	if(OLED == 0)
	{
		return 0u;
	}

	return OLED->Busy;
}

int OLED_SendBuffer(OLED_TypeDef *OLED)
{
	if((OLED == 0) || (OLED->Bound == 0u) || (OLED->Write == 0))
	{
		return OLED_ERR_PARAM;
	}

	return OLED_FlushAll(OLED);
}

int OLED_Init(OLED_TypeDef *OLED)
{
	/* 上电初始化序列，逐条对应 SSD1306 数据手册第 8 章的命令。
	 * 顺序按手册推荐流程：先关显示，配好参数，最后开显示 */
	static const uint8_t init_cmds[] =
	{
		SSD1306_CMD_DISPLAY_OFF,                              /* 0xAE 先关显示 */
		SSD1306_CMD_SET_CLOCK_DIV, SSD1306_CLOCK_DIV_0x80,    /* 0xD5 时钟分频/振荡频率 */
		SSD1306_CMD_SET_MULTIPLEX, SSD1306_MULTIPLEX_64,      /* 0xA8 复用比 = 64 行 */
		SSD1306_CMD_SET_DISPLAY_OFFSET, SSD1306_DISPLAY_OFFSET_0, /* 0xD3 垂直偏移 0 */
		SSD1306_CMD_SET_START_LINE,                           /* 0x40 显示起始行 = 0 */
		SSD1306_CMD_CHARGE_PUMP, SSD1306_CHARGE_PUMP_ON,      /* 0x8D 打开内部电荷泵 */
		SSD1306_CMD_SET_MEMORY_MODE, SSD1306_MEMORY_MODE_HORIZONTAL, /* 0x20 水平寻址 */
		SSD1306_CMD_SEG_REMAP,                                /* 0xA1 段重映射：左右镜像 */
		SSD1306_CMD_COM_SCAN_DIR,                             /* 0xC8 COM 反序扫描：上下翻转 */
		SSD1306_CMD_SET_COM_PINS, SSD1306_COM_PINS_ALT,       /* 0xDA COM 引脚配置（128x64） */
		SSD1306_CMD_SET_CONTRAST, SSD1306_CONTRAST_DEFAULT,   /* 0x81 对比度 */
		SSD1306_CMD_SET_PRECHARGE, SSD1306_PRECHARGE_DEFAULT, /* 0xD9 预充电周期 */
		SSD1306_CMD_SET_VCOMH, SSD1306_VCOMH_DEFAULT,         /* 0xDB VCOMH 取消电平 */
		SSD1306_CMD_DEACTIVATE_SCROLL,                        /* 0x2E 关闭滚动 */
		SSD1306_CMD_ENTIRE_DISPLAY_ON,                        /* 0xA4 按显存内容显示 */
		SSD1306_CMD_NORMAL_DISPLAY,                           /* 0xA6 正常（非反白） */
		SSD1306_CMD_DISPLAY_ON                                /* 0xAF 开显示 */
	};

	uint8_t buffer[1u + sizeof(init_cmds)];
	uint16_t i;

	if((OLED == 0) || (OLED->Bound == 0u) || (OLED->Write == 0))
	{
		return OLED_ERR_PARAM;
	}

	/* 整条序列一次发完：控制字节（命令）+ 全部命令字节。
	 * SSD1306 允许把连续命令拼在同一个 I2C 传输里 */
	buffer[0] = SSD1306_CTRL_CMD;
	for(i = 0u; i < (uint16_t)sizeof(init_cmds); i++)
	{
		buffer[i + 1u] = init_cmds[i];
	}

	if(OLED->Write(OLED->Addr, buffer, (uint16_t)(sizeof(init_cmds) + 1u)) != 0)
	{
		return OLED_ERR_I2C;   /* 屏幕没应答：多半是接线或地址不对 */
	}

	/* 清屏并把这一帧推到屏幕上，避免上电时显存里是随机内容 */
	OLED_Clear(OLED);
	OLED->CursorX = 0;
	OLED->CursorY = 0;
	OLED->Busy    = 0u;
	OLED->Progress = 0u;

	return OLED_FlushAll(OLED);
}
