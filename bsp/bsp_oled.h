/* bsp_oled.h - SSD1306 OLED 显示驱动（128x64，I2C）
 *
 * 依据：Solomon Systech **SSD1306** 数据手册 Rev 1.1
 *   - 第 8 章 命令表：0xAE/0xAF 显示开关、0xD5 时钟分频、0xA8 复用比、
 *     0xD3 垂直显示偏移、0x40 显示起始行、0x8D 电荷泵、0x20 寻址模式、
 *     0xA1/0xC8 段/行重映射、0xDA COM 引脚配置、0x81 对比度、
 *     0xD9 预充电周期、0xDB VCOMH 取消电平、0x2E 关闭滚动、0xA4/0xA6
 *   - I2C 写时序：控制字节 0x00 = 后面全是命令；0x40 = 后面全是显存数据
 *   - 上电后 VCC 稳定到发第一条命令之间需要 >100ms（本模块由调用方保证）
 *
 * 显示存储：SSD1306 的显存按「页」组织，一页 8 行像素，共 8 页。
 *   每页 128 个字节，一个字节代表某一列上连续的 8 个像素，
 *   **bit0 在最上面**。这与 bsp_font6x8 的取模方向一致，可以整字节搬运。
 *
 * ⚠️ 本驱动只支持 SSD1306。
 *    常见的 0.96 寸 I2C 模块还有一种控制器是 SH1106，两者不兼容：
 *      · SH1106 不支持 0x20（寻址模式）命令；
 *      · SH1106 每页有 132 列，显存写入要从第 2 列开始（需要 +2 偏移）。
 *    接上 SH1106 会出现花屏或整体右移 2 像素。
 *    确认芯片型号请查模块 PCB 丝印或问卖家；若确实要用 SH1106，
 *    需要另写一份初始化序列和列偏移，不要靠改几个宏凑合。
 *
 * 本模块只提供状态屏真正用到的能力（初始化/清屏/光标/画字符/输出缓冲），
 * 没有画圆、画线、位图、剪切区、文本框、多字体那一套。
 */

#ifndef _BSP_OLED_H_
#define _BSP_OLED_H_

#include "stm32f10x.h"

/* 模块默认 I2C 从机地址（7 位地址 0x3C 左对齐） */
#define OLED_I2C_ADDR           (0x78u)

#define OLED_WIDTH              (128u)
#define OLED_HEIGHT             (64u)
#define OLED_PAGE_COUNT         (8u)          /* 64 / 8 */
#define OLED_FRAME_BYTES        (OLED_WIDTH * OLED_PAGE_COUNT)   /* 1024 */

/* 分段刷新的单次长度上限（字节）。
 * 128x64 整帧 1024 字节，在 100kHz 软 I2C 上要 100ms 以上。
 * 拆成 8 字节一段，每段只占主循环约 1ms，控制环不会被拖住 */
#define OLED_CHUNK_BYTES        (8u)

/* 各接口的返回值：0 成功，负数失败 */
#define OLED_OK                 (0)
#define OLED_ERR_PARAM          (-1)
#define OLED_ERR_I2C            (-2)

/* 底层 I2C 写回调：把 size 个字节写到从机 addr。
 * 返回 0 表示成功，非 0 表示失败（直接透传 bsp_si2c 的返回值即可） */
typedef int (*OLED_WriteCallback_t)(uint8_t addr, const uint8_t *pdata, uint16_t size);

typedef struct
{
	/* --- 由绑定接口填写 --- */
	OLED_WriteCallback_t Write;   /* 必填：底层 I2C 写 */
	uint8_t              Addr;    /* 选填：从机地址，填 0 则用 OLED_I2C_ADDR */

	/* --- 内部状态，调用方不要直接改 --- */
	uint8_t  Frame[OLED_FRAME_BYTES];  /* 帧缓冲，1024 字节 */
	int16_t  CursorX;
	int16_t  CursorY;
	uint16_t Progress;                 /* 分段刷新进度，单位字节 */
	uint8_t  Busy;                     /* 1 = 有一帧正在分段往屏幕搬 */
	uint8_t  Bound;                    /* 1 = 已经绑定过回调 */
} OLED_TypeDef;

/* 绑定底层 I2C 写函数。
 * 必须在 OLED_Init 之前调用；回调不能为空 */
int OLED_Bind(OLED_TypeDef *OLED, OLED_WriteCallback_t Write, uint8_t Addr);

/* 初始化屏幕：发一整套 SSD1306 上电配置，并把帧缓冲清空推到屏幕上。
 * 返回值：OLED_OK / OLED_ERR_PARAM（没绑定）/ OLED_ERR_I2C（屏幕没应答） */
int OLED_Init(OLED_TypeDef *OLED);

/* 只清帧缓冲，不动屏幕。屏幕内容要等下一次 SendBuffer 才更新 */
void OLED_Clear(OLED_TypeDef *OLED);

/* 设置光标（字符左上角），单位像素。超出屏幕会被夹到边界内 */
void OLED_SetCursor(OLED_TypeDef *OLED, int16_t X, int16_t Y);

/* 在光标处画一个以 '\0' 结尾的字符串，画完光标自动右移。
 * 只支持 0x20~0x7E 的 ASCII；其它字符（含中文）按空格处理。
 * 画到右边界会自动换到下一行；画到底部就停下 */
void OLED_DrawString(OLED_TypeDef *OLED, const char *Str);

/* 格式化输出到光标处。单次上限 63 字节，超出截断。
 * ⚠️ 与 My_USART_Printf 一样：microlib 下 %f 要几毫秒，
 *    不要放在控制环里，整数（%d）优先 */
void OLED_Printf(OLED_TypeDef *OLED, const char *Format, ...);

/* 把整个帧缓冲一次性写进屏幕。
 * 会阻塞到 1024 字节发完（100kHz 软 I2C 上约 100ms），
 * **只能在控制环还没跑起来的时候用**（比如上电初始化）。
 * 运行中请改用下面的分段接口 */
int OLED_SendBuffer(OLED_TypeDef *OLED);

/* 开始分段刷新：只记录状态，不搬数据。之后由主循环反复调 OLED_Pump */
int OLED_StartSendBuffer(OLED_TypeDef *OLED);

/* 分段刷新的一步：搬最多 OLED_CHUNK_BYTES 个字节。
 * 返回 OLED_OK 表示这一小段成功；
 * 返回 OLED_ERR_I2C 表示传输失败（已自动取消本次刷新）；
 * 通过 *pMoreOut 告知是否还有剩余：
 *   1 = 还没搬完，下一圈继续调
 *   0 = 已经搬完（此时 Busy 已经清掉，可以开始下一次 StartSendBuffer） */
int OLED_Pump(OLED_TypeDef *OLED, uint8_t *pMoreOut);

/* 屏幕是否正有一帧在分段搬运中 */
uint8_t OLED_IsBusy(OLED_TypeDef *OLED);

#endif
