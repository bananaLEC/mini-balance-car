/* bsp_si2c.h - 软件（位拆）I2C 主机接口
 *
 * 依据：NXP UM10204《I2C-bus specification and user manual》
 *   - 起始条件：SCL 为高时 SDA 由高变低
 *   - 停止条件：SCL 为高时 SDA 由低变高
 *   - 数据位：SCL 为低时摆 SDA，SCL 为高时由从机采样
 *   - 应答位：主机发完 8 位后释放 SDA，第 9 个时钟读 SDA 电平，低 = ACK
 *   - 两线都是开漏，高电平靠外部上拉电阻
 *
 * 本模块用于 OLED（PB8/PB9）。OLED 只写不读，但四个接口都给全，
 * 以后接 EEPROM 之类的软 I2C 器件不用返工。
 */

#ifndef _BSP_SI2C_H_
#define _BSP_SI2C_H_

#include "stm32f10x.h"

#define SI2C_ACK    1u   /* 主机拉低 SDA：应答，表示「还要继续读」 */
#define SI2C_NACK   0u   /* 主机释放 SDA：非应答，表示「读完了」   */

typedef struct
{
	GPIO_TypeDef *SCL_GPIOx;   /* SCL 所在端口，取 GPIOA..GPIOE */
	uint16_t      SCL_GPIO_Pin;/* SCL 引脚，取 GPIO_Pin_0..15   */

	GPIO_TypeDef *SDA_GPIOx;   /* SDA 所在端口 */
	uint16_t      SDA_GPIO_Pin;/* SDA 引脚   */

	/* 半周期延时，单位 us。I2C 标准模式下总线速率约 100kHz，
	 * 即半周期 5us；取 2~3us 对应约 150~250kHz，
	 * 是本板 OLED 模块实测能稳定工作的范围。
	 * 换更长的排线或换器件时把它调大即可，不用改代码逻辑 */
	uint16_t      HalfPeriodUs;
} SI2C_TypeDef;

/* 把 SCL / SDA 配成开漏输出，并把两根线放高（进入空闲态）。
 * 调用前需自行保证 SI2C_TypeDef 里的字段已填好 */
void My_SI2C_Init(SI2C_TypeDef *SI2C);

/* 向从机写多个字节。
 * Addr 为 7 位地址左对齐（即 A6..A0 后面跟 0，写操作），例如 OLED 的 0x78。
 * 返回值： 0 成功 / -1 寻址失败（从机没应答地址）/ -2 数据被拒收（从机没应答数据） */
int My_SI2C_SendBytes(SI2C_TypeDef *SI2C, uint8_t Addr, const uint8_t *pData, uint16_t Size);

/* 从从机读多个字节。最后一个字节回 NACK，其余回 ACK（I2C 规范要求）。
 * 返回值： 0 成功 / -1 寻址失败 */
int My_SI2C_ReceiveBytes(SI2C_TypeDef *SI2C, uint8_t Addr, uint8_t *pBuffer, uint16_t Size);

/* 读从机某个寄存器的值：起始 → 地址+写 → 寄存器号 → 重复起始 → 地址+读 → 数据 → 停止
 * 返回值： 0 成功 / -1 寻址失败 / -2 寄存器号被拒收 */
int My_SI2C_RegReadBytes(SI2C_TypeDef *SI2C, uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Size);

/* 写从机某个寄存器的值
 * 返回值： 0 成功 / -1 寻址失败 / -2 寄存器号或数据被拒收 */
int My_SI2C_RegWriteBytes(SI2C_TypeDef *SI2C, uint8_t Addr, uint8_t Reg, const uint8_t *pData, uint16_t Size);

#endif
