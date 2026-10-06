/* bsp_font6x8.h - 6x8 点阵 ASCII 字体（本项目自行制作）
 *
 * 字形范围：0x20(空格) ~ 0x7E(~)，共 95 个可见字符。
 *
 * 点阵格式：每个字符 8 个字节，**纵向取模、低位在上**。
 *   字节 n 对应字符的第 n 列（从左到右，共 6 列有效），
 *   bit0 = 该列最上面那一点（y=0），bit6 = 最下面那一点（y=6）。
 *   第 7 列（第 7 个 bit）留空，作为字符之间的间隔列。
 *
 * 因此 OLED 上画一个字符的流程是：
 *   对 col = 0..5：取出 Font6x8[c][col]，对 bit = 0..6：
 *       若置位则在 (x+col, y+bit) 画点
 *
 * 一个字符占 6 列 x 8 行，所以 128x64 的屏一屏 21 列 x 8 行。
 * 这是本项目自己整理的字模，不含任何第三方字体数据。
 */

#ifndef _BSP_FONT6X8_H_
#define _BSP_FONT6X8_H_

#include <stdint.h>

#define FONT6X8_WIDTH          (6u)   /* 每个字符占 6 列 */
#define FONT6X8_HEIGHT         (8u)   /* 每个字符占 8 行 */

#define FONT6X8_FIRST_CHAR     (0x20u)
#define FONT6X8_LAST_CHAR      (0x7Eu)
#define FONT6X8_CHAR_COUNT     (FONT6X8_LAST_CHAR - FONT6X8_FIRST_CHAR + 1u)   /* 95 */

/* 95 个字符 x 8 字节的字模表 */
extern const uint8_t Font6x8[FONT6X8_CHAR_COUNT][8];

/* 取某个字符的字模。
 * 超出 0x20~0x7E 范围的字符（含中文、控制字符）统一返回空格的字模，
 * 这样调用方不需要自己判范围，不会画出乱七八糟的东西。
 * 返回值指向 8 字节的列点阵，可直接索引 0..7 */
const uint8_t *Font6x8_GetGlyph(char c);

#endif
