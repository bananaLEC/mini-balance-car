# 第三方组件与版权声明

本仓库（迷你平衡车 / STM32F103C8T6）包含来源为第三方的代码与资料。本文件逐项
说明它们的来源、版权归属和使用条件。

> **重构说明（已完成）**：本仓库原先包含 B 站「铁头山羊」的 STM32 外设驱动
> （`my_lib/` 下的 `delay` / `usart` / `i2c` / `si2c` / `oled` / `oled_font` /
> `oled_default_font` / `qmath` / `spi` / `str_cmd`）。这些代码**已在本次重构中
> 全部重写**：`bsp/` 目录是本项目依据公开数据手册与官方参考手册独立实现的新代码，
> 著作权归本项目作者；原有的字体转换工具（`ttf2c` / `bdf2c` / `otf2bdf`）与商业
> TTF 字体也已从仓库剔除，字模改为自研点阵表。
>
> 重构所依据的文档清单、方案与验证方式见
> [`docs/驱动重构方案.md`](docs/驱动重构方案.md) 与 [`bsp/README.md`](bsp/README.md)。

> **重要**：本仓库**没有**附加覆盖全部内容的开源许可证。原因见文末
> 「本仓库的许可证状态」。

---

## 1. STMicroelectronics — STM32F10x 标准外设库（SPL）

### 涉及文件

| 路径 | 版本 / 说明 |
|---|---|
| `std_periph_driver/inc/`、`std_periph_driver/src/` | STM32F10x_StdPeriph_Driver V3.6.2 |
| `std_periph_driver/inc/core_cm3.h`、`cmsis_*.h` | ARM CMSIS Cortex-M3 内核头文件 |
| `startup/startup_stm32f10x_*.s` | 各容量型号的启动汇编文件 |
| `user/system_stm32f10x.c` / `.h` | 系统时钟配置 |
| `user/stm32f10x_it.c` / `.h` | 中断向量与默认中断服务函数 |
| `user/stm32f10x_conf.h` | 外设库配置文件 |

### 版权归属

Copyright (c) 2011–2012 STMicroelectronics. All rights reserved.

### 使用条件

各源文件头部声明：

> This software is licensed under terms that can be found in the **LICENSE file**
> in the root directory of this software component.
> If no LICENSE file comes with this software, it is provided AS-IS.

许可条款为 **MCD-ST Liberty SW License Agreement V2**（ST 官方文档编号
**SLA0044 Rev5 / February 2018**）。许可全文已随本仓库提供：

| 文件 | 说明 |
|---|---|
| `LICENSES/MCD-ST-Liberty-SW-License-V2.txt` | 规范副本，覆盖本仓库全部 ST 衍生文件 |
| `std_periph_driver/LICENSE.txt` | 同一文本，对应库文件头中「本组件根目录下的 LICENSE 文件」这一指向 |

这满足第 1 条的要求：再分发源码时保留版权声明、条件列表与免责声明
（即许可正文的第 10、11 条）。

### ⚠️ 第 5 条：不得置于开源许可证之下

这是本仓库最需要注意的一条限制。原文：

> 5. No use, reproduction or redistribution of this software partially or totally
> may be done in any manner that would subject this software to any **Open Source
> Terms**. "Open Source Terms" shall mean any open source license which requires
> as part of distribution of software that the source code of such software is
> distributed therewith or otherwise made available... such as for example GNU
> General Public License (GPL), Eclipse Public License (EPL), **Apache Software
> License, BSD license or MIT license**.

直译：**不得以任何方式使本软件受制于开源条款**，并明确点名 GPL、EPL、Apache、
BSD、MIT。

对实际做法的影响：

- **不要**在本仓库根目录放 MIT / BSD / Apache / GPL 许可证并声明覆盖全部内容。
  这不仅是「替别人授权」的伦理问题，还**直接违反 ST 许可第 5 条** —— 根许可证
  会要求随分发一并按该许可证提供 SPL 源码。
- 本仓库因此选择**不附加**统一的开源许可证，理由见第 4 节。
- 第 3 条：不得用 ST 的名义为衍生产品背书 —— 本仓库未作此类声明。
- 第 4 条：本软件只能在 ST 的微控制器上使用或执行 —— 本项目正是
  STM32F103C8T6，符合。

### 文本来源

许可全文取自 [ScanCode LicenseDB 的 `st-mcd-2.0` 条目](https://scancode-licensedb.aboutcode.org/st-mcd-2.0.html)，
该条目收录的是 ST 官方发布的 SLA0044 Rev5 文本。本仓库未改动任何字句。

---

## 2. 江协科技 — 平衡车控制板（硬件方案）

### 来源

本项目的硬件基于**江协科技「平衡车控制板 V2.0」**方案，控制板购自淘宝。
原原理图（嘉立创 EDA 绘制）标题栏信息为：

> TITLE: 平衡车控制板 | Company: 江协科技 | Drawn By: HGD | REV: 2.0

### 本仓库的处理方式

**本仓库不包含江协科技的原理图原图。** 原因：

- 原理图作为图形作品受著作权保护，著作权归江协科技所有；
- **购买实物板卡 ≠ 获得原理图的复制与再发布许可**，这是两件独立的事；
- README 中的引脚分配表是**依据本项目代码**（`user/app_pwm.c`、
  `user/app_encoder.c`、`user/app_bluetooth.c` 等）整理的事实性连接信息，
  不构成对原图的复制。

### 使用提示

- 购买的实物板卡归你所有，拍照、录像、撰写使用心得均无限制；
- 若淘宝卖家额外提供了编译好的固件（`.hex` / `.bin`），那是卖家的成果，
  不要放进本仓库；
- 需要原原理图请通过江协科技的官方渠道获取。

---

## 3. 已从本仓库排除的内容

以下内容出于版权与仓库卫生考虑**不纳入版本控制**（见 `.gitignore`），
其中多数在本次重构后已从工作区一并删除：

| 内容 | 原因 |
|---|---|
| `my_lib/font/ttfttc/*.ttf`（汉仪劲楷、华文行楷、华文仿宋、等线、黑体） | **商业授权字体**。汉仪字库等厂商对字体再分发有明确限制，把字体文件公开上传属于明确的侵权风险，且与本项目技术内容无关。**重构后已删除** |
| `my_lib/font/*.exe`、`*.dll`（`ttf2c`、`bdf2c`、`otf2bdf`、`ttf2c.core.dll`） | 第三方发布的配套字体转换工具，版权归原作者；二进制文件也不适合入库。**重构后字模改为自研点阵表，这些工具不再需要，已删除** |
| `schematic.png` | 江协科技的原理图，见第 2 节 |
| `Objects/`、`Listings/`、`build/`、`*.hex`、`*.axf`、`*.map` 等 | 编译产物，可重新生成 |
| `*.uvguix.*` | Keil 界面布局配置，**含 Windows 用户名** |
| `*.log`、`.vscode/uv4.log.lock`、`.claude/`、`.probe/` | 本地日志与个人配置 |

---

## 4. 本仓库的许可证状态

**本仓库当前未附加统一的开源许可证**，各部分权利状态如下：

| 部分 | 权利状态 |
|---|---|
| `user/`、`test/` | 本项目作者原创 |
| `bsp/` | **本项目作者原创**（本次重构新写，依据公开数据手册独立实现） |
| `my_lib/task.*`、`my_lib/pid.*`、`my_lib/button.*` | 本项目作者原创（作者在 AI 辅助下编写） |
| `std_periph_driver/`、`startup/`、`user/system_stm32f10x.*`、`user/stm32f10x_it.*`、`user/stm32f10x_conf.h` | STMicroelectronics，MCD-ST Liberty SW License V2 |
| 硬件方案 | 江协科技 |

### 如果要给仓库加许可证

**不要在根目录放一个覆盖全部内容的 MIT / BSD / Apache 许可证。**

> 注意：本次重构**并没有**改变这个结论。原先有两条理由，第一条已经消除，
> 第二条依然成立：
>
> 1. ~~仓库中相当一部分代码的著作权不属于本项目作者~~ → **已解决**：
>    第三方驱动代码已全部重写为 `bsp/` 下的自研实现；
> 2. **ST 许可第 5 条明确禁止**使 SPL 受制于开源条款，并点名了 MIT、BSD、
>    Apache、GPL —— 见第 1 节。**这条不因重构而消失。**

可行方案（按推荐度排序）：

1. **【本项目当前选择】私有仓库 + 不附加统一许可证。** 私有仓库不构成对外传播，
   版权风险最低；同时第三方许可原文已随仓库提供，权属关系清晰。
2. **将来若要公开：仓库内容已全部为自有代码 + ST 标准外设库。** 可以把根许可证
   限定为「仅覆盖本项目自有部分（`user/`、`test/`、`bsp/`、`my_lib/`）」，并
   在许可证正文与 README 中明确**不含** `std_periph_driver/`、`startup/` 及
   ST 衍生文件（这些仍按 MCD-ST Liberty SW License V2 使用）。
   这种「限定范围的许可证」不构成对 SPL 施加开源条款，符合第 5 条。
3. **公开时一并脱开 SPL**：改用 ST 官方的 **STM32CubeF1 HAL/LL**（BSD-3-Clause）
   或直接写寄存器，就能整仓使用 MIT / Apache。代价是工作量较大。

### 各部分的许可原文位置

| 部分 | 许可原文 |
|---|---|
| STMicroelectronics | `LICENSES/MCD-ST-Liberty-SW-License-V2.txt`、`std_periph_driver/LICENSE.txt` |
| 本项目作者原创部分（`bsp/`、`user/`、`test/`、`my_lib/`） | 无 —— 默认保留所有权利 |

---

## 5. 关于 git 历史

原先的提交历史里包含重构前的第三方驱动源码（初始提交 `336a966` 与
`507f8c7`），仅重写当前代码并不足以清除。本次重构已一并处理历史：

- **本地历史已重建**：以当前（不含第三方代码的）工作区内容新建了一个
  **根提交** `7369d3f`，历史中不再出现任何第三方驱动文件；
- **远端已同步**：已用强制推送把重建后的历史推到 `origin/main`
  （`507f8c7 → 7369d3f`）。现在 GitHub 上的
  [bananaLEC/mini-balance-car](https://github.com/bananaLEC/mini-balance-car)
  主分支已经是**不含第三方驱动代码**的版本；
  ⚠️ 强制推送改写了公开历史，**在推送之前克隆过该仓库的人需要重新克隆**。
- **旧历史已保留备份**：分支 `backup-before-refactor` 与标签
  `backup-pre-refactor` 仍指向 `507f8c7`（只存在于本地，没有推到远端）。
  确认不需要再回退后可以删掉：
  ```powershell
  git branch -D backup-before-refactor
  git tag -d backup-pre-refactor
  ```

> 说明：清掉的是**提交历史里可达的文件**。本地磁盘上的旧驱动文件仍然存在，
> 但它们从未被提交过，见下一节。若推送时提示连不上 github.com，
> 多半是本机代理（`127.0.0.1:7897`）没有启动，开启后重试即可。

---

## 6. 工作区里仍保留的旧驱动文件

`my_lib/` 下的旧驱动文件（`delay` / `usart` / `i2c` / `si2c` / `oled` /
`oled_font` / `oled_default_font` / `qmath` / `spi` / `str_cmd`）以及
`my_lib/font/` 目前**仍在磁盘上**，但已经：

- 从 `template.uvprojx` 的 `my_lib` 组移除 —— **不参与编译**；
- 从 git 索引移除 —— **不被跟踪、不在任何提交里**；
- 由 `bsp/` 下同名接口的自研实现完全替代 —— 接口名保持不变。

之所以没删掉，是因为当前会话的文件沙箱不允许删除 `my_lib` 下已存在的文件
（创建、修改可以，删除被拒）。要彻底清掉，运行：

```powershell
pwsh -File tools\清理旧驱动.ps1
```

或者双击根目录的 `清理旧驱动.cmd`。脚本只删上面显式列出的目标，
不会碰 `my_lib/task.*`、`pid.*`、`button.*`，删完代码不需要任何改动。

---

*本文件基于对仓库现有文件的检查整理，不构成法律意见。涉及正式发布或商业用途时，
请咨询专业人士。*
