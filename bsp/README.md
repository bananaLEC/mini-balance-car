# bsp/ — 板级驱动层（Board Support Package）

本目录下全部代码为**本项目自研**，依据公开数据手册与官方参考手册编写，
不含任何第三方驱动源码。

## 与旧实现的关系

重构前 `my_lib/` 下的底层驱动（`delay` / `usart` / `i2c` / `si2c` / `oled` / `qmath` / `spi` / `str_cmd`）
来源为 B 站「铁头山羊」，未附带任何开源许可证。本目录是它们的**重写替代**，
接口名与签名保持兼容，因此 `user/` 与 `test/` 无需改动。

`my_lib/` 现在只保留本项目自己写的三个模块：`task.*`（周期任务调度）、
`pid.*`（PID 控制器）、`button.*`（按键驱动）。

## 依据的文档

| 模块 | 依据 |
|---|---|
| `delay.*` | ARM **DDI 0337**（Cortex-M3 技术参考手册）— SysTick、DWT_CYCCNT、`SCB->SHP[]` |
| `usart.*` | ST **RM0008** 第 27 章（USART）；Keil MDK microlib 文档 |
| `i2c.*` | ST **RM0008** 第 26 章；ST **AN2824**；**ES096**（STM32F103 勘误手册） |
| `si2c.*` | NXP **UM10204**（I2C-bus specification） |
| `oled.*` | Solomon Systech **SSD1306** 数据手册 Rev 1.1 |
| `font6x8.*` | 本项目自行制作的点阵字模 |

## 重构中修掉的缺陷

详见 [`docs/驱动重构方案.md`](../docs/驱动重构方案.md)。
