# bsp/ 自研驱动独立复核报告

- 复核对象：`bsp/bsp_i2c.*`、`bsp/bsp_si2c.*`、`bsp/bsp_delay.*`、`bsp/bsp_oled.*`、`bsp/bsp_usart.*`（含 `bsp_font6x8.*` 的位序核对）
- 复核方式：**只读静态审查**（未修改 `bsp/` 下任何文件）+ 编译/链接验证 + SPL 头文件与源码交叉核对 + 字模独立解码
- 复核工具记录：
  - `pwsh -File tools\compile.ps1` → **51 / 51 通过，0 warning 0 error**
  - `pwsh -File tools\link.ps1` → **链接成功**，`Total ROM Size = 27620 B (26.97 kB)`、`Total RW Size = 6296 B (6.15 kB)`
    （重构前基线 ROM ≈ 23516+14444+196 = 38.2 kB / RAM ≈ 9.4 kB，当前实现更小，预算内）
- 依据文档：ST **RM0008** §26.3.1/§26.3.2/§26.6（I2C）、ST **AN2824**、**UM10204** §3.1、ARM **DDI0337**（SysTick/DWT）、Solomon **SSD1306** Rev1.1 §8/§10.1.2-10.1.3、工程内 SPL 源码与原实现对照
- 结论一句话：**接口与数值计算（下标、位运算、字模、串口边界）写得干净，但计时地基（`GetUs`）实现错误，直接连锁出「临界区超时失效→死锁」「软 I2C 慢 500 倍→控制环被拖垮」，另有 2 处硬件 I2C 时序/状态缺陷必须先修再上板。**

> 说明：本环境无法直接检索 RM0008 原始 PDF（PDF 抓取被拒），凡涉及手册措辞的结论均以「工程内 ST SPL 官方注释（`std_periph_driver/`）+ 原实现可工作代码 + 社区实证」三方交叉为据，并在第 6 节列出需要上板确认的项。

---

## 0. 严重问题摘要（先看这 4 条）

| # | 位置 | 一句话后果 |
|---|---|---|
| **P0-1** | `bsp_delay.c:128-170` | `GetUs()` 的微秒部分恒为 0（`elapsed` 量的是「读时间戳本身」的几十个周期），`GetUs` 实际只有 **1 ms 分辨率**。`DelayUs(2)` 退化成「等到下一个毫秒边界」→ 软 I2C 半周期 0~1000 µs、`OLED_Pump` 单段阻塞 **≈100 ms**、整屏 **≈15 s**，主循环（`main.c:217-221` 每圈搬一段）被拖到 **≈8 Hz**，5 ms 平衡环根本无法运行；编码器边沿时间戳也全部量化到 1 ms，`ENC_MIN_EDGE_US=350` 的滤波判据失效。 |
| **P0-2** | `bsp_i2c.c:141-154` + `240` + `264` | 临界区内的超时**永远不会到期**：`__disable_irq()` 期间 SysTick 中断进不来 → `delay_ticks` 不增 → `GetUs()` 被钉死在同一个值 → `GetUs()-start` 恒为 0。任何一次「最后字节没收全」（包括 P0-3 必然触发的情况）都会**永久死循环且中断一直关着**，电机保持上一拍 PWM；工程还没做看门狗（`main.c:215`），只能断电。这正是 `bsp_i2c.c:12-13` 注释声称要避免的场景。 |
| **P0-3** | `bsp_i2c.c:296-309` + `258-291` + `419-480` | `My_I2C_RegReadBytes(..., Size == 2)` 这条路径**从不清 ADDR 标志**（重复起始后 `ADDR=1`，只有 `I2C_ReadSingleByte`/`Size>=3`/地址阶段三处会清）。主接收模式下 `ADDR=1` 会拉住 SCL 不放（RM0008 §26.3.2），RXNE 永不置位 → 配合 P0-2 直接**死机**。当前 `user/` 只用到 Size=14 和 Size=1，所以是**潜伏缺陷**，但属于公开 API 的必然死锁。 |
| **P0-4** | `bsp_i2c.c:275`、`311-313`、`321-323` | 每次读成功后 ACK 位都**残留在 DISABLE**（`I2C_ReadLastTwoBytes`/`I2C_ReadSingleByte` 关掉后再没恢复）；而 `Size>=3` 路径清 ADDR 之前**不先把 ACK 置 1**，只在读到第 0 个字节之后（`line 323`）才置 1。按 RM0008，ACK 位管的是「下一个被接收字节」的应答，第 0 字节的应答时隙就在 RXNE 之后约 1 个 SCL 周期（400 kHz 下 ≈1.25~1.7 µs），而轮询循环每次迭代都调用带 64 位除法的 `GetUs()`（百纳秒~µs 级），**这是一个会输的竞态**：首字节被 NACK → 从机停发 → 等第 1 字节 10 ms 超时返回 -3。原实现 `my_lib/i2c.c:247` 恰恰是先 `ACK=ENABLE` 再清 ADDR，重写时丢掉了这一步。 |

---

## 1. 逐项核查结论（对应任务书的 8 条要求）

### 1.1 `bsp_i2c.c` 的 ADDR 清除时机（Size==1 / ==2 / >=3 三条路径）

- **清标志的写法本身正确**：`I2C_ClearAddrFlag()`（`bsp_i2c.c:165-169`）先读 SR1 再读 SR2，符合 RM0008；与本仓库 SPL 自带注释一致（`std_periph_driver/src/stm32f10x_i2c.c:1203-1205`：「ADDR 由 SR1 读 + SR2 读的序列清除」）。另核对 `I2C_GetFlagStatus`（`stm32f10x_i2c.c:1132-1176`）只读「标志所在的那一个寄存器」（BUSY 在 SR2，其余在 SR1），因此等待标志不会顺手清掉 ADDR，也不会误清。
- **但三条路径的「清 ADDR / 关 ACK / 发 STOP」相对顺序不一致，且与手册顺序有出入**：
  - `Size == 1`（`bsp_i2c.c:230-252`）：代码写的是 `ACK=0 → 清 ADDR → STOP`，与手册 EV6_1 顺序一致 —— **但前提是 ADDR 此时还在**。`My_I2C_ReceiveBytes` 走的是 `I2C_AddressPhase()`，其末尾 `line 218` 已经把 ADDR 清了，所以实际执行顺序是 `地址阶段清 ADDR → …… → ACK=0 → (空操作)清 ADDR → STOP`。`bsp_i2c.c:227-229` 的注释「地址阶段之后 ADDR 还没清」**与事实不符**（只有 `My_I2C_RegReadBytes` 的 Size==1 分支才成立）。实际后果：1 字节的应答时隙在清 ADDR 之后约 8 个位时间（400 kHz ≈ 20 µs）才到，ACK=0 早就写好，**功能上无害**（详见 L6），但注释误导、易被后人「顺手改坏」。
  - `Size == 2`（`bsp_i2c.c:305-309` → `I2C_ReadLastTwoBytes`）：**清 ADDR 完全缺失**（P0-3，严重）。
  - `Size >= 3`（`bsp_i2c.c:311-323`）：清 ADDR 之前没有把 ACK 置 1（P0-4，严重）；代码注释「此时应答位是开的」在「上一笔是读操作」时不成立。
- **结论**：清 ADDR 的动作写法对，**调用它的时机与配套的 ACK 位状态不对**；`ReceiveBytes` 与 `RegReadBytes` 两条入口共用同一套函数，但两条入口进入 `I2C_ReadData` 时 ADDR / ACK 的初态不同（一个已清 ADDR、一个未清），这是缺陷的结构性来源。

### 1.2 多字节接收「读 DR 同时清 RXNE 和 BTF」那一步

- 注释（`bsp_i2c.c:254-256`）表述不准确：按 SPL 官方注释（`stm32f10x_i2c.c:1200-1202`），BTF 的清除序列是「**先读 SR1，再读/写 DR**」，单读 DR 并不清 BTF。由于代码每次等 RXNE 的轮询本身就是读 SR1，紧接的 `I2C_ReceiveData()` 确实会顺带清掉 BTF，**实际结果正确**，属注释不严谨（L7）。
- **不会死等、不会多读一个字节**：`I2C_ReadLastTwoBytes()` 每个 `I2C_ReceiveData()` 对应一个字节，恰好读 2 个；`Size>=3` 的循环边界 `for(i=1; (i+2)<Size; i++)` 与末尾 `Size-2 / Size-1` 组合覆盖完整（Size=3/4/14 逐一推演通过），无越界、无重复读、无漏读。
- **时间窗比手册窄但没有错**：手册 2 字节口径是「BTF 置位后关 ACK、发 STOP」，本实现是「倒数第二个字节 RXNE 就绪即关 ACK+STOP」，早于手册时机；由于 ACK 位只作用于「下一个被接收字节」，最后一个字节仍会被正确 NACK，读 DR 也仍能拿到它 —— **功能成立**，仅在第 0 个字节异常迟到时才可能多 ACK 一个字节（对照从机无影响，低风险）。
- 唯一真实的等不到场景来自 P0-3（ADDR 未清）。

### 1.3 `bsp_si2c.c` 的 ACK/NAK 语义 —— **已核查，无误（旧 bug 未重犯）**

- `SI2C_ACK=1` 含义为「拉低 SDA」（`bsp_si2c.h:19-20`）；`SI2C_ReadByte()` 里 `SDA_Write(Ack ? 0 : 1)`（`bsp_si2c.c:164`）与定义一致：传 `SI2C_ACK` → 拉低 → 应答。
- 两个入口都正确：`My_SI2C_ReceiveBytes`（`bsp_si2c.c:308`）与 `My_SI2C_RegReadBytes`（`bsp_si2c.c:353`）都用 `(i == Size-1) ? SI2C_NACK : SI2C_ACK`，即**中间字节 ACK、最后一个字节 NACK**，未出现原实现「两套相反逻辑」的问题。
- 主机侧读第 9 个时钟前的处理也正确：`SDA_Write(1)` 释放（`bsp_si2c.c:146`、`173`），不会读到自己。

### 1.4 `bsp_si2c.c` 的信号时序（UM10204）

- **电平先后顺序全部正确**：起始 = SCL 高时 SDA 高→低（`60-71`）；停止 = SCL 高时 SDA 低→高（`74-85`）；重复起始 = 先释放 SDA、再拉高 SCL、再拉低 SDA（`89-103`，入口处 SCL 已为低，不会误产生 STOP）；数据位 = SCL 低时改变 SDA、SCL 高时采样（`115-121`）；读位 = 先释放 SDA 再拉高 SCL（`145-161`，MSB first）；ACK 位 = SCL 低时驱动 SDA、SCL 高时输出、再拉低 SCL 后释放（`164-173`）。符合 UM10204 §3.1「SDA 只能在 SCL 为低时改变」及 START/STOP 定义。
- **每个边沿都有明确延时**（`SI2C_HalfDelay`，`bsp_si2c.c:50-53`），旧实现「低电平半周期靠指令时间凑」的问题确实修掉了。
- **但实际电平宽度不达标（受 P0-1 连累）**：`HalfPeriodUs = 2`（`user/app_oled.c:13`），而 `DelayUs(2)` 在 `GetUs` 只有 1 ms 台阶时是「等到下一个毫秒边界」，实际 0~1000 µs **且随机**，最小可到亚微秒 → 快速模式的 t<sub>LOW</sub>≥1.3 µs、t<sub>SU;STA</sub>/t<sub>SU;STO</sub>≥0.6 µs 都可能被违反，同时总线频率从设计的 ~250 kHz 掉到 ~1 kHz。
- START/STOP 的建立/保持时间在半周期为 2~5 µs 时满足快速模式（≥0.6 µs）与标准模式（≥4.7 µs，仅 5 µs 配置下勉强）要求，**属边缘但合规**；不检查从机时钟延展（SI2C 从不采样 SCL 是否真的变高）——OLED 不延展，今天无影响，属 L5。

### 1.5 `bsp_delay.c`

- **可重入性**：`GetUs()` 内只读全局量与硬件寄存器，不写共享状态，不关中断，**可重入、且不破坏调用方中断状态**（这一点比原实现好，`bsp_delay.h:6-7` 的主张成立）。
- **毫秒 tick 写入者唯一**：`delay_ticks` 只由 `Delay_TickInc()` 写（`bsp_delay.c:116-119`），唯一调用点是 `SysTick_Handler`（`user/stm32f10x_it.c:164-169`），全工程无第二处自增 —— **已核查，无误**。
- **回绕处理**：`Delay()` 用无符号差值（`bsp_delay.c:181`），跨 49.7 天回绕正确；`bsp_usart.c` 的超时也用无符号差值（`bsp_usart.c:41`），正确。
- **Q8 换算无累积误差**：`cycles_per_us_q8 = (HCLK<<8)/1e6`（`bsp_delay.c:85`）。整数 MHz 时 `HCLK = MHz*1e6`，该式恒等于 `MHz*256`（72 MHz → 18432），`elapsed*256/18432` 恰好等于 `elapsed/72`，**没有截断误差**；`(uint64_t)elapsed << 8` 先转 64 位，无溢出。**已核查，无误**。
- **致命问题见 P0-1/P0-2**：微秒分量算法取错了参考点；而「tick 只由中断推进」这条正确设计与「临界区里要用它做超时」互相矛盾。
- `Delay_Init()` 的 SysTick 配置顺序正确（先 `CTRL &= ~ENABLE`、清 VAL、写 LOAD、设优先级、最后一次性写 CTRL，`bsp_delay.c:88-112`），DWT 使能序列（`DEMCR.TRCENA` → `CYCCNT=0` → `CTRL.CYCCNTENA`，`bsp_delay.c:51-59`）符合 DDI0337，`NVIC_SetPriority` 在 `NVIC_PriorityGroupConfig` 之后调用（`main.c:173` → `175`）也正确。
- **DWT 探测失败的备用路径反而是唯一正确的路径**：`dwt_ready==0` 时用 `SysTick->LOAD - VAL` 补亚毫秒（`bsp_delay.c:160-167`），分辨率与控制区可推进性都对 —— 这是本模块最讽刺的一处（详见 L4/M3）。

### 1.6 `bsp_oled.c`

- **帧缓冲下标与位运算正确**：`index = (Y/8)*128 + X`、`mask = 1 << (Y&7)`（`bsp_oled.c:127-128`），与 SSD1306 §8.7 页结构（每页 8 行、bit0 在最上）以及字模「按列取模、bit0 在上」一致；已独立解码 `bsp_font6x8.c` 验证：95 个字形、无 bit7 置位，'A'/'5'/'g'/'.' 渲染正确。
- **页地址设置有问题（M1，中）**：`OLED_Init` 把寻址模式设成**水平寻址**（`bsp_oled.c:414`：`0x20, 0x00`），而分段/整帧刷新用的是**页寻址专用命令** `0xB0+page`（`bsp_oled.c:85`、`339`）和单字节列地址（`bsp_oled.c:92`、`346`）。SSD1306 手册把 `00h~0Fh / 10h~1Fh / B0h~B7h` 明确标注为 Page Addressing Mode 专用；水平模式下它们被忽略，定位靠「写入指针恰好连续」侥幸成立。另外 `header[1] = offset % 128` 在列 ≥16 时**根本不是合法的列地址命令**（页寻址要分两字节：`00h~0Fh` 低半字节 + `10h~1Fh` 高半字节）。
- **分段刷新的「串页」风险**：当前 `OLED_CHUNK_BYTES=8`（`bsp_oled.h:43`）整除 128，每段不跨页，加上水平模式自增，所以显示正常；一旦改块长导致跨页、或在页寻址模式下，就会串页。更现实的风险是**写入指针错位**：`OLED_Pump` 失败时清 `Progress/Busy`（`bsp_oled.c:359-365`）但 SSD1306 内部指针停在半途，下一次 `OLED_SendBuffer`/新一帧写入就会**整体错位**（最大 4 页），因为「每段重设页地址」这件事实际没生效。
- **初始化命令序列逐条合法**：`0xAE / 0xD5,0x80 / 0xA8,0x3F / 0xD3,0x00 / 0x40 / 0x8D,0x14 / 0x20,0x00 / 0xA1 / 0xC8 / 0xDA,0x12 / 0x81,0xCF / 0xD9,0xF1 / 0xDB,0x20 / 0x2E / 0xA4 / 0xA6 / 0xAF`（`bsp_oled.c:406-425`）取值均与 SSD1306 命令表相符；把控制字节 `0x00` + 多条命令拼在**同一个 I2C 传输**里是手册允许的。上电 >100 ms 的等待由调用方保证（`bsp_oled.h:9`、`App_OLED_Init` 远晚于上电），可接受。
- 缓冲区边界、`vsnprintf` 截断、无 `malloc`、`SetPixel` 边界裁剪均正确（见第 3 节）。

### 1.7 `bsp_usart.c`

- **超时路径已修好**：`USART_Timeout()`（`bsp_usart.c:34-42`）用「剩余时间 + 无符号差值」表达，`Timeout<0` 表示无限等待，**不存在未初始化变量路径**，旧实现 `expireTime` 未初始化 + 依赖短路求值的问题确实消除了。`Timeout==0` → 立刻返回 0，行为明确。
- **`ReceiveLine` 边界正确**（详见第 3 节第 15 条）：写入下标最大 `MaxLength-1`，末尾一定补 `'\0'`；CRLF 只在 `"\r\n"` 序列上返回 0；`MaxLength<2`、CRLF 且 `<3` 都被拒绝（旧实现这里判据是错的，新的对）。
- 遗留小项：发送通路 `TXE/TC` 裸等无超时（`bsp_usart.c:47-51`、`71-74`），USART 未使能时钟时会永久阻塞（L8）；`LineSeperator` 非法值落到 CRLF 分支（可接受）。

### 1.8 通用：溢出 / 回绕 / 越界 / 空指针 / 中断安全

- 未发现数组越界、整数溢出、位域错用；`OLED_FlushAll` 的 `buffer[10]` 只用 9 字节、`OLED_Pump` 的 `buffer[9]` 只用 9 字节，均在界内（多留 1 字节无害）。
- 空指针处理不一致：`My_I2C_ReceiveBytes(NULL 缓冲, Size>0)`、`My_SI2C_ReceiveBytes/RegReadBytes(NULL, Size>0)` 都**静默返回 0（成功）**（`bsp_i2c.c:404-407`、`bsp_si2c.c:290-293`、`320-323`），调用方无法区分「读到 0 字节」和「参数错误」（L3）；传输函数不校验 `I2Cx/SI2C` 句柄为空（`My_SI2C_Init` 校验了，`bsp_si2c.c:214`）。
- 中断安全：`bsp_i2c.c` 的两处临界区用 `__disable_irq()/__enable_irq()`，**不保存/恢复 PRIMASK**（M2），与本模块「可在临界区里调用」的设计主张冲突；`bsp_si2c.c`/`bsp_oled.c`/`bsp_usart.c` 不使用临界区，无此问题。

---

## 2. 发现清单（按格式逐条）

### P0-1 【高】`GetUs()` 的微秒分量恒为 0，整条时间地基只有 1 ms 分辨率

- 【文件:行号】`bsp_delay.c:128-170`（核心 `144-158`），下游 `bsp_delay.c:187-200`、`bsp_si2c.c:50-53`、`bsp_oled.c:310-380`、`bsp_oled.h:43`、`user/app_oled.c:13`、`user/main.c:217-221`、`user/app_encoder.c:47,73-79,157-166`
- 【问题描述】
  ```c
  base  = DWT->CYCCNT;      /* 144 */
  ticks = delay_ticks;      /* 145 */
  now   = DWT->CYCCNT;      /* 146 */
  elapsed = now - base;     /* 149：量的是「读 ticks 这一步本身」的耗时（几个~几十个周期） */
  sub_us  = (elapsed << 8) / cycles_per_us_q8;   /* 158：72MHz 下 <72 周期即截断为 0 */
  return ticks*1000 + sub_us;                    /* 169 */
  ```
  `base/now` 之间只隔着一次 `delay_ticks` 读取，`elapsed` 不是「距上一个毫秒边界的周期数」，而是这次函数调用内部的几十个周期，除以 72 后**恒为 0**（只有恰好被中断插进来时才可能凑出 1~几 µs）。于是 `GetUs()` 退化为 `delay_ticks*1000`，**分辨率 1 ms**，与 `bsp_delay.h:32-36` 声称的 `1/72 µs` 完全不符。
- 【为什么是问题】
  - 手册/设计依据：`docs/驱动重构方案.md` §3.3(a) 方案 A 要求「DWT->CYCCNT 配合 SysTick 溢出计数」得到绝对微秒时间；正确写法应取 `(CYCCNT - 该毫秒拍起点周期数)/72` 或 `(SysTick->LOAD - SysTick->VAL)/72`（原实现 `my_lib/delay.c:101` 就是后者）。当前写法把「函数自身的执行时间」当成了亚毫秒余量。
  - 实测后果链（全部可由代码推算，不依赖硬件）：
    1. `DelayUs(2)`（`bsp_delay.c:194-198`）→ `start = GetUs()+3`，而 `GetUs` 只以 1000 为步进 → **实际等待「到下一个 ms 边界」，0~1000 µs 且随机**，最小值可到亚微秒。
    2. 软 I2C 每个位 = 2 次半周期（`bsp_si2c.c:115-121`）→ 平均 ~1 ms/位、~9 ms/字节 → 总线频率 ~1 kHz（设计值 ~250 kHz，慢约 250 倍），且电平宽度违反 UM10204 最小值。
    3. `OLED_Pump` 一段 = 3 次 I2C 传输、13 个数据字节 + 3 个地址字节（`bsp_oled.c:338-357`）→ 每字节 ~9 ms → **单段阻塞 ≈100~150 ms**（`main.c:217-221` 每圈只搬一段 → 主循环 ~8 Hz）；整屏 128 段 ≈ **15 s**；`App_OLED_Init` 里的一次性整屏（`user/app_oled.c:102`，1024+ 字节）也 ≈15 s。5 ms/10 ms 的任务（MPU6050、姿态、平衡、电机、按键）全部被拖到 8 Hz 附近，**平衡环无法工作**（`app_oled.c:128-129` 注释「一次只搬 8 字节（软 I2C 上约 1ms）」的估计因此差两个数量级）。
    4. 编码器：边沿时间戳全部是 1000 的倍数（`app_encoder.c:73`），而假边沿判据 `ENC_MIN_EDGE_US=350`（`app_encoder.c:47`、判据在 `76`）→ **同一毫秒内的真实边沿被当噪声丢弃**（满速时真实边沿间隔约 770 µs，约每两个边沿就有一个落在同一毫秒而被丢）；`us_per_edge` 只能是 1000/2000/… 的倍数（`app_encoder.c:157-158`）→ 轮速反馈系统性失真（真实 770 µs 间隔被记成 1000 µs 时偏低约 23%，丢边沿后跳到 2000 µs 时读数直接腰斩），速度环与平衡环的 x_dot 重建随之失真。
    5. PID：`deltaT = (t_k - t_k_1)*1e-6`（`my_lib/pid.c:56-57`）被量化到 1 ms；`deltaT < 1e-6` 时才夹到 1 µs（`pid.c:64-67`），一旦同一毫秒内两次调用，微分项 `err_dev = Δerr/1e-6` 被放大 1e6 倍。
    6. 附带：`I2C_WaitFlag` 等轮询循环每圈都调 `GetUs()`（`bsp_i2c.c:104`、`133`、`147`），而 `GetUs` 里有 64 位除法（`__aeabi_uldivmod`，百纳秒~µs 级），把 400 kHz 下的应答位窗口（约 1 个 SCL 周期）挤掉 —— 它是 P0-4 竞态的直接帮凶。
- 【严重程度】**高**（会让硬件「能通但跑不动」：屏幕 15 s 一帧、控制环 8 Hz、测速失真；且掩盖了 P0-2 的死锁）
- 【建议修法】改成绝对时间拼接，二选一：
  1. 首选（与原实现同构、且临界区里也推进）：`us = delay_ticks*1000 + (uint64_t)((SysTick->LOAD - SysTick->VAL) * 256 / cycles_per_us_q8);` —— 注意 `GetUs` 不再需要 DWT（DWT 可作为可选的更高分辨率源）；`delay_ticks` 读 + `VAL` 读仍需容错（见 M3）。
  2. 保留 DWT：在 SysTick 中断里记录本拍起点 `cyccnt_at_tick = DWT->CYCCNT;`（与 `delay_ticks++` 一起），`GetUs = delay_ticks*1000 + (DWT->CYCCNT - cyccnt_at_tick)/72`，并处理 59.6 s 回绕。
  3. 无论哪种，都把「超时必须能在关中断期间推进」作为硬约束（见 P0-2）。
  4. 顺带把轮询循环里的 `GetUs()` 降频（例如每 32 圈检查一次超时，或用纯计数器），把 400 kHz 下的应答窗口留给 ACK 位。

### P0-2 【高】临界区内的超时永远不会到期 → 死锁且中断永久关闭

- 【文件:行号】`bsp_i2c.c:141-154`（`I2C_WaitFlagCritical`）、调用点 `bsp_i2c.c:240`、`264`；成因 `bsp_delay.c:116-128`
- 【问题描述】`I2C_WaitFlagCritical` 在 `__disable_irq()` 之后用 `GetUs()` 做 1 ms 超时。由于 `GetUs` 的值 = `delay_ticks*1000 + 0`（P0-1），而 `delay_ticks` **只由 SysTick 中断推进**（`bsp_delay.c:116-119`，`stm32f10x_it.c:166-169`），关中断期间 SysTick 中断无法执行 → `GetUs()` 在临界区内是**严格常量** → `(GetUs() - start) > 1000` 永远为假 → `while` 永不退出。
- 【为什么是问题】
  - 这直接推翻了模块自己的安全承诺：`bsp_i2c.c:11-13` 写明「每一个等标志位的地方都有超时，函数不会无限阻塞……裸 while 一旦遇到从机拉死总线就永久卡死，此时电机保持上一拍 PWM，车会冲出去」；`bsp_i2c.c:28-31` 也说明临界区等待「不能等太久，因为全程关着中断」。而实际是**临界区里的两处等待一旦落空就是无限期关中断的死循环**，比「裸 while」更糟（中断也关了，串口、编码器、SysTick 全部停摆）。
  - `main.c:215` 明确「看门狗还没做」，所以没有 IWDG 兜底，现场只能断电。
  - 触发条件并不苛刻：P0-3（Size==2）必然进入；从机掉电/线松/时钟延展过长等任何「最后一个字节没来」的情形也都会进入。
  - 对比：原实现在临界区内用**纯循环计数**限时（`my_lib/i2c.c:60-77`，10 万次≈10 ms），虽然与主频耦合，但不会死锁 —— 重写时把这条经验丢掉了。
- 【严重程度】**高**（确定性死锁 + 中断永久关闭 + 无看门狗）
- 【建议修法】
  1. 让时间源在关中断期间也能推进（P0-1 的修法 1/2 即可满足；`SysTick->VAL`/`DWT->CYCCNT` 都是硬件计数器，与中断无关）。
  2. 或者在临界区内改用**有界循环次数**（保留原方案）并在超时后立刻 `__enable_irq()`。
  3. 无论哪种，都不要在临界区内依赖「由中断推进的软件计数」。
  4. 加 IWDG（`改正方案.md` 第 6 步已有方案）作为最后一道兜底。

### P0-3 【高】`My_I2C_RegReadBytes(..., Size == 2)` 从不清 ADDR → 必然死等

- 【文件:行号】`bsp_i2c.c:419-480`（`468` 等待 ADDR、`479` 调用读数据）、`bsp_i2c.c:296-309`（Size==2 分支）、`bsp_i2c.c:258-291`（`I2C_ReadLastTwoBytes` 全程无 `I2C_ClearAddrFlag`）
- 【问题描述】`RegReadBytes` 在读方向地址阶段只「等」ADDR（`line 468`），**没有清**；随后 `I2C_ReadData(Size==2)` 直接进 `I2C_ReadLastTwoBytes`，该函数只在 `__disable_irq()` 后等 RXNE（`line 264`）。全工程清 ADDR 的位置只有三处：地址阶段末尾（`218`）、`I2C_ReadSingleByte`（`237`）、`Size>=3` 分支（`313`），**Size==2 这条路径一处都不覆盖**。
- 【为什么是问题】RM0008 §26.3.2 主接收流程要求 EV6（ADDR=1）用「读 SR1 + 读 SR2」清除；ADDR 未清时主机在地址字节与第一个数据字节之间拉住 SCL（这正是「1 字节接收必须把清 ADDR 和发 STOP 放一起」的原因）。因此 RXNE 永远不会置位 → 走到 P0-2 的无限死循环（中断全关）；即便假设硬件不拉 SCL，ADDR 残留也会让**后续每一笔事务**在 `I2C_WaitEither(AF, ADDR)`（`203`/`468`）处立刻误判「地址已被应答」，状态机彻底错乱。
- 【严重程度】**高**（公开 API 的必然死锁；当前 `user/` 只调 Size=14/1，属潜伏）
- 【建议修法】把 ADDR 的清除统一放到 `I2C_ReadData` 里按 SIZE 分支处理，让 `ReceiveBytes`/`RegReadBytes` 两条入口的 ADDR/ACK 初态一致，例如：
  ```c
  if (Size == 1) { ACK=0; ClearAddr(); STOP; wait RXNE; *pBuffer = DR; }
  else if (Size == 2) { ACK=1; ClearAddr(); /* 再 ACK=0 + STOP + 读两字节 */ }
  else { ACK=1; ClearAddr(); ... }
  ```
  并在 `RegReadBytes` 中把「地址阶段」封装成显式参数（`ClearAddrAfterAddr`），不要靠「调用者已经清过」的隐式约定。

### P0-4 【高】读操作结束后 ACK 位残留 `DISABLE`，`Size>=3` 清 ADDR 前又不先置 1 → 首字节被 NACK 的竞态

- 【文件:行号】`bsp_i2c.c:275`（关 ACK 后再无恢复）、`236`（同）、`290`（直接 return 0）、`311-313`（清 ADDR 前未置 ACK）、`321-323`（读到第 0 字节之后才置 ACK）
- 【问题描述】每次成功的读（`I2C_ReadLastTwoBytes` / `I2C_ReadSingleByte`）都会把 `CR1.ACK` 置 0 后返回，模块没有在收尾时恢复「空闲态 ACK=1」。下一次 `Size>=3` 的读进入时：地址阶段末尾以 `ACK=0` 清 ADDR（`218`）→ 主接收开始接收第 0 个数据字节 → 只有等第 0 字节的 RXNE 被轮询到、读走 DR 之后才在 `line 323` 把 ACK 置 1。而 ACK 位决定的是「下一个被接收字节」的应答（RM0008 §26.6.1 CR1.ACK/POS；SPL 亦以 `I2C_AcknowledgeConfig` 控制该位）：第 0 字节的应答时隙就在它自己的第 9 个时钟，即 RXNE 置位后约一个 SCL 周期（400 kHz 下 ≈1.25~1.7 µs）；本代码从「轮询发现 RXNE」到写完 CR1 要经过 `I2C_GetFlagStatus` + `I2C_ReceiveData` + `I2C_AcknowledgeConfig`，而轮询循环每圈还夹着带 64 位除法的 `GetUs()`（P0-1 第 6 点），延迟可达数微秒 → **很可能赶不上应答时隙**。
- 【为什么是问题】
  - 手册依据：RM0008 主接收 EV6 的顺序是「ACK=1（保持应答）→ 清 ADDR → 依次接收」；单字节接收（EV6_1）更是明确要求「**先关 ACK**，再清 ADDR，再发 STOP」，说明 ACK 位必须在对应字节被接收**之前**写好。当前代码在 `Size>=3` 分支把顺序做反了（先清 ADDR、后置 ACK），并且依赖「上一次遗留的 ACK 状态」。
  - 代码自证：`bsp_i2c.c:311` 的注释「此时应答位是开的」在「上一笔是读操作」时不成立 —— `I2C_ReadLastTwoBytes` 在 `275` 行关掉 ACK 后直接返回成功，`I2C_ReadSingleByte` 同理。也就是说，**除开机第一笔读以外，每一笔多字节读都是在 ACK=0 的状态下清 ADDR 的**。
  - 后果：若竞态失败，从机（MPU6050）在第 0 个字节看到 NACK 即释放 SDA 停发 → 代码在 `326-332` 等第 1 个字节 → **10 ms 超时**（`I2C_TIMEOUT_US`）→ 返回 -3。`user/app_mpu6050.c:109-130` 会把它计成一次失败（并可能触发 `My_I2C_ResetBus`），而 10 ms 的阻塞落在 5 ms 的控制任务里。若竞态总是失败（若 ACK 位在字节装载时就被锁存），则表现为「隔一次读失败一次」——无论哪种都属于必须修的真实缺陷。
  - 对照原实现：`my_lib/i2c.c:247` 在清 ADDR **之前**先 `I2C_AcknowledgeConfig(ENABLE)`，且失败路径会恢复 ACK（`271`、`404`）；重写版把「先置 1」丢了，只保留了「读完后关 0」。
- 【严重程度】**高**（真实时序竞态，直接命中当前唯一的传感器读路径；建议上板用逻辑分析仪确认首字节 ACK/NACK 后再判定实际发生率）
- 【建议修法】
  1. 每笔传输开始时（清 ADDR 之前）显式 `I2C_AcknowledgeConfig(I2Cx, ENABLE)`；`Size==1` 分支保持「先 ACK=0 再清 ADDR 再 STOP」。
  2. 每笔传输收尾（无论成功/失败/超时）都把 ACK 位恢复成模块空闲态（ENABLE），不要让状态跨调用泄漏；错误路径同理。
  3. 把「当前处于哪个 SIZE 分支、ADDR 是否已清、ACK 当前值」写成显式状态（或函数参数），避免隐式约定。
  4. 配合 P0-1 的修法 4 降低轮询延迟。

### M1 【中】SSD1306 寻址模式与页地址命令不一致：定位命令被忽略，靠「顺序恰好连续」侥幸正确

- 【文件:行号】`bsp_oled.c:414`（`0x20, 0x00` 水平寻址）↔ `bsp_oled.c:84-96`、`336-349`（`0xB0+page` 与单字节列地址）；相关 `bsp_oled.h:43`、`bsp_oled.c:359-365`
- 【问题描述】初始化把内存寻址模式设为**水平寻址**，而分段/整帧刷新使用**页寻址专用**命令：`0xB0 + (offset/128)` 与 `header[1] = offset % 128`。后者在水平模式下按手册不生效（`00h~0Fh`、`10h~1Fh`、`B0h~B7h` 均标注为 Page Addressing Mode），因此「每段重设页地址、显存写入地址不会串」（`bsp_oled.c:14` 的注释）这件事**实际上没有发生**；显示之所以看起来正确，只是因为数据按 `offset` 单调递增写入，而水平模式下指针恰好也线性自增（社区同源案例：u8g2 issue #1362 —— 「the set low/high column and set start page commands are ignored, it just overflows into the next page and by accident continues drawing in just the right spot」，见文末链接）。另外 `offset % 128` 作为「低列地址」命令在列 ≥16 时**根本不是合法命令**（页寻址需要 `00h~0Fh` + `10h~1Fh` 两字节）。
- 【为什么是问题】
  - 手册依据：SSD1306 Rev1.1 §8.5/§8.6/§8.7 把列/页起始地址命令限定在 Page Addressing Mode；§10.1.2/§10.1.3 分别定义水平/页寻址下指针的自增与回绕规则（水平模式下回绕点由 `21h/22h` 决定）。
  - 具体故障面：`OLED_Pump` 失败后清 `Progress/Busy`（`359-365`），但 SSD1306 内部指针停在半途；下一次 `OLED_SendBuffer`（`user/app_oled.c:102` 上电用、`bsp_oled.c:392-400`）或新一帧的第 0 段会从**残留指针**开始写 → 画面整体错位（8 字节一段、最大可错到 4 页 = 半屏），且现象难与「接线问题」区分。当前 `OLED_CHUNK_BYTES=8` 整除 128 才没暴露串页问题（改块长即暴露）。
  - 参考：原实现用的是水平模式的正确写法 `0x21`（列 0~127）+ `0x22`（页 0~7），见 `my_lib/oled.c:935-945`。
- 【严重程度】**中**（常见路径侥幸正确，异常/改动后出错；且属于「按手册就不该这么写」）
- 【建议修法】二选一，不要混用：
  - A. 改成页寻址：`0x20, 0x02`，每段发 `0x00|(col&0x0F)`、`0x10|((col>>4)&0x0F)`、`0xB0|page`（且段长必须整除 128 或每段自己按页边界切分）。
  - B. 保持水平寻址：用 `0x21`（起始列、结束列）与 `0x22`（起始页、结束页）命令设置窗口，每段显式重设；这也是原实现的做法。
  - 并在每次 `OLED_StartSendBuffer()` 时先发一次「整帧窗口」命令，消除失败重启后的指针残留。

### M2 【中】临界区不保存/恢复 `PRIMASK`，破坏嵌套临界区

- 【文件:行号】`bsp_i2c.c:234-249`、`262-278`
- 【问题描述】两处都用 `__disable_irq() ... __enable_irq()`，而不是 `uint32_t pm = __get_PRIMASK(); __disable_irq(); ... __set_PRIMASK(pm);`。若调用方本身处在自己的临界区里（`user/app_encoder.c:110-112、133-142` 就是这种用法），I2C 读会**在返回时把中断重新打开**，嵌套临界区失效。
- 【为什么是问题】`bsp_i2c.c:14-16` 与 `bsp_delay.h:6-7` 都宣称「本模块在临界区内调用是安全的、不会破坏调用方的中断状态」；`GetUs` 本身确实做到了，但这两个包装函数没有 —— 承诺与实现不一致，属于典型的中断安全缺陷（当前调用路径恰好没踩到，属潜伏）。
- 【严重程度】**中**
- 【建议修法】改用 `__get_PRIMASK()/__set_PRIMASK()` 保存恢复；或在函数入口直接 `__disable_irq()` 并用 `__set_PRIMASK(1)` 恢复「进入前的状态」。

### M3 【中】`GetUs()` 的单调性声明不成立：回绕点会倒退，且备用路径读寄存器非原子

- 【文件:行号】`bsp_delay.c:144-158`（DWT 路径）、`bsp_delay.c:160-167`（备用路径）；声明在 `bsp_delay.h:32-36`；受害者 `my_lib/pid.c:56-67`、`user/app_balance.c:221-233`、`user/app_encoder.c:130,157`
- 【问题描述】
  1. DWT 路径的返回值是 `delay_ticks*1000 + sub_us`。`sub_us` 只反映本次调用内部的几十个周期，和「距本毫秒边界的余量」无关；在 32 位 CYCCNT 回绕瞬间（72 MHz 下约 59.6 s 一次），`delay_ticks` 尚未进位而 `sub_us` 从 ~999 跳回 ~0 → **返回值可倒退最多约 1 ms**，与 `bsp_delay.c:141-143`「时间轴永不倒退」的注释不符。
  2. 备用路径先读 `delay_ticks` 再读 `SysTick->VAL`（`164`、`166`），两次读之间若刚好发生 SysTick 重装/进位，会得到「旧 tick + 新 VAL」→ 同样可倒退近 1 ms。
- 【为什么是问题】调用方全部按无符号差值使用：`pid.c` 的 `deltaT = (t_k - t_k_1)*1e-6f` 一旦遇到倒退会得到 `1.8e13` 量级的巨大 dt（梯形积分一步冲到限幅，输出被夹到极限，表现为偶发一拍电机猛冲）；`app_balance.c` 因有 `dt > 0.05f` 限幅而只丢一拍；`app_encoder.c` 的 `now - t_last` 会瞬间大于 `ENC_TIMEOUT_US` 而误判停转。虽然每 ~60 s 才可能发生一次，但这是「安全关键控制环里的一次饱和输出」。
- 【严重程度】**中**（低频但会直接作用到电机输出）
- 【建议修法】① 用「读两次直到一致」或「先读计数器、再读 tick、再读计数器」的容错法保证 (tick, 余量) 同源（原实现 `my_lib/delay.c:80-95` 的 while 重读思路可借鉴，但不要在里面开关中断）；② 在调用方把「负差值」显式处理掉（`pid.c` 的 `deltaT` 建议加 `if (t_k < PID->t_k_1) deltaT = 上次值`），或直接改回用 SysTick 余量法（天然单调）；③ 修正 `bsp_delay.h` 的单调性描述。

### L1 【低】`My_I2C_ResetBus()` 的 GPIO 切换有毛刺、且「补停止位」实际补出的是一个 START

- 【文件:行号】`bsp_i2c.c:575-600`
- 【问题描述】
  1. `GPIO_Init(Out_OD)`（`580`）先于 `GPIO_SetBits`（`581`）：ODR 里这两个位此前为 0，切到开漏输出的瞬间两根线被**驱动为低**，几百纳秒后才被拉高 —— 这会在线路上产生一次非预期的 START/STOP 边沿。对比 `bsp_si2c.c:225-235` 特意按「先使能时钟 → 先写 ODR=1 → 再配模式」的顺序做，并在注释里写明了原因。
  2. `#4 补一个停止位`（`595-600`）：9 个脉冲循环结束时 SCL 仍为**高**，随后先拉低 SDA（`596`）→ 这是「SCL 高时 SDA 下降」= **START 条件**，再拉高 SDA 才是 STOP。也就是救总线时先发了一个 START 再发 STOP（多数从机能容忍，但对正在恢复的从机是多余的地址唤醒）。
- 【为什么是问题】抢救函数的目的是把总线带回确定的空闲态（`bsp_i2c.c:595` 注释自己写的是「SCL 为高时把 SDA 由低拉高」），实际波形却不是干净的 STOP；毛刺则可能在救总线时又给从机添一次起始条件。旧实现（`my_lib/i2c.c:107-127`）顺序相同，属未修正的历史遗留而非新引入。
- 【严重程度】**低**
- 【建议修法】① 切模式前先 `GPIO_SetBits`（或写 `BSRR`）再 `GPIO_Init`；② 停止位按 `SCL 低 → SDA 低 → SCL 高 → SDA 高` 顺序产生。

### L2 【低】`I2C_WaitFlag()` 的参数语义与函数名相反，容易误用

- 【文件:行号】`bsp_i2c.c:98-111`，调用点 `182`（`I2C_FLAG_BUSY, SET` 表示「等 BUSY 变 0」）
- 【问题描述】函数实际语义是「等到标志**不等于** State」，但形参名为 `State`、函数名为 `WaitFlag`，读起来像「等标志变成 State」。所有调用点（BUSY/SB/RXNE/TXE）**逻辑都是对的**，属可读性/可维护性隐患。
- 【严重程度】**低**
- 【建议修法】改名 `I2C_WaitFlagUntilNot(I2Cx, Flag, State)`，或加一行注释明确「State = 要等它离开的状态」，并对 BUSY 的调用写清「等总线转为空闲」。

### L3 【低】空指针被当成「成功」，句柄为空不校验

- 【文件:行号】`bsp_i2c.c:404-407`、`bsp_si2c.c:290-293`、`320-323`、`258-261`
- 【问题描述】`if((pBuffer == 0) || (Size == 0u)) return 0;` —— 缓冲区为空且 `Size>0` 时返回 **0（成功）**，调用方会把未初始化的缓冲区当有效数据用（这正是 `docs/驱动重构方案.md` §1.3 想避免的「拿栈垃圾当数据」）。另外传输函数不校验 `I2Cx`/`SI2C` 是否为空（`My_SI2C_Init` 校验了，`bsp_si2c.c:214`），传空句柄会硬故障。
- 【严重程度】**低**（当前调用点都不为空的，属防御性）
- 【建议修法】参数错误返回 `-2`（或新增 `-4` 参数错误码），与「成功 0」区分开；对 `I2Cx/SI2C == 0` 直接返回错误。

### L4 【低】`Delay_Init()` 哨兵先置位（存在除数=0 窗口）、`DWT_Probe()` 判据偏弱

- 【文件:行号】`bsp_delay.c:66-85`、`bsp_delay.c:38-48`
- 【问题描述】① `delay_init_done = 1u;`（`70`）在真正初始化完成（`85` 算出 `cycles_per_us_q8`）之前就置位：若在 `Delay_Init` 执行期间有中断调 `GetUs()`（例如把 `Delay_Init` 放到较晚的位置、而某外设中断已使能），`GetUs` 会提前返回并执行 `... / cycles_per_us_q8`，**除数为 0**（Cortex-M3 的 UDIV/软件除法返回 0，不会异常，但会得到一个 0 或错误的时间片）。② `DWT_Probe()` 只判断「16 个 NOP 后 CYCCNT != 0」，若 CYCCNT 恰在探测窗口内回绕到 0（59.6 s 一次的机会）会误判 DWT 不可用 → 退回备用路径（功能正确，只是分辨率/行为变化），属极小概率。
- 【严重程度】**低**
- 【建议修法】把 `delay_init_done = 1u` 移到函数最后（或用「初始化中」状态机）；`DWT_Probe` 改为「读两次、要求变化量在合理范围」，并在探测失败时打印/记录一次。

### L5 【低】软 I2C 不处理从机时钟延展

- 【文件:行号】`bsp_si2c.c:107-137`、`140-176`
- 【问题描述】主机只驱动 SCL，从不回读 SCL 是否真的变高；UM10204 允许从机拉低 SCL 延展时钟，主机必须等待。对 SSD1306（不延展）无影响，但 `bsp_si2c.h:10-11` 声明「以后接 EEPROM 之类的软 I2C 器件不用返工」，接会延展的器件就会出错。
- 【严重程度】**低**
- 【建议修法】在拉高 SCL 后加「等 SCL 实际变高（加上限超时）」；`SI2C_TypeDef` 增加 SCL 端口/引脚已有，直接用 `GPIO_ReadInputDataBit` 即可。

### L6 【低】1 字节路径的注释与事实不符（清 ADDR 是空操作）

- 【文件:行号】`bsp_i2c.c:218`（地址阶段已清 ADDR）↔ `bsp_i2c.c:227-229`、`236-238`
- 【问题描述】`My_I2C_ReceiveBytes` 的 Size==1 路径上，ADDR 已在 `I2C_AddressPhase()` 末尾被清；`I2C_ReadSingleByte` 里的「清 ADDR」是空操作，注释「地址阶段之后 ADDR 还没清」只对 `My_I2C_RegReadBytes` 的 Size==1 分支成立。
- 【为什么是问题（含无害结论）】实际顺序变成「清 ADDR（地址阶段）→ 关 ACK → STOP」，与 RM0008 EV6_1 的字面顺序不同；但单字节的应答时隙在清 ADDR 后约 8 个位时间（400 kHz ≈ 20 µs）才到，ACK=0 早已写好，**本次未发现功能影响**；风险在于后人照注释改代码。
- 【严重程度】**低**
- 【建议修法】让 `I2C_ReadData` 明确区分「ADDR 是否已由调用方清除」，把注释放到正确的位置；或统一由 `I2C_ReadData` 负责 ADDR/ACK 时序（与 P0-3 的修法合并）。

### L7 【低】关于 BTF 清除条件的注释不准确

- 【文件:行号】`bsp_i2c.c:254-256`（「读 DR 会同时清掉 RXNE 和 BTF」）
- 【问题描述】按 ST SPL 官方注释（`std_periph_driver/src/stm32f10x_i2c.c:1200-1202`），BTF 的清除序列是「先读 SR1，再读/写 DR」；单读 DR 不足以清 BTF（这是 F1 与 F4 的差异点）。本代码每次等 RXNE 都会读 SR1，所以紧随其后的 `I2C_ReceiveData()` 确实会清掉 BTF，**行为正确，仅注释不严谨**。
- 【严重程度】**低**
- 【建议修法】注释改为「等 RXNE 的轮询已读过 SR1，随后的读 DR 会一并清掉 RXNE 与 BTF」。

### L8 【低】串口发送通路无超时、`ReceiveLine` 对非法分隔符值不报错

- 【文件:行号】`bsp_usart.c:47-51`、`71-74`、`214`
- 【问题描述】`My_USART_SendByte/SendBytes` 裸等 `TXE/TC`（`bsp_usart.c:47`、`71`），USART 时钟未使能或外设异常时永久阻塞；`My_USART_ReceiveLine` 的 `else` 分支把任何非 CR/LF 的值都当 CRLF 处理，非法参数不返回 -2。
- 【为什么是问题】与本模块「接收都有超时」的风格不一致；发送通路挂在控制环外的调试任务里（`main.c:211`），风险有限。属接口健壮性建议。
- 【严重程度】**低**
- 【建议修法】可选加一个 `TX` 超时（用 `GetTick()`）；`ReceiveLine` 对未知 `LineSeperator` 返回 -2。

### L9 【低】`OLED_SetCursor()` 的上边界夹到了屏幕外

- 【文件:行号】`bsp_oled.c:236-243`
- 【问题描述】`Y > OLED_HEIGHT(64)` 时夹到 64，而合法范围是 0~63；`Y=64` 后 `OLED_DrawString` 会在 `bsp_oled.c:259-262` 直接返回，等于「设了个合法范围内的光标但什么都画不出来」，与注释「避免调用方算出越界坐标后整个画面被裁掉」的意图不完全一致（X 同理夹到 128）。
- 【严重程度】**低**
- 【建议修法】夹到 `OLED_HEIGHT-1` / `OLED_WIDTH-1`，或明确文档化「Y≥64 时不再绘制」。

---

## 3. 已核查、确认无误的清单

1. **`bsp_si2c.c` ACK/NAK 语义**（任务重点 3）：`SI2C_ACK=1 → 拉低 SDA`，`ReceiveBytes`（`308`）与 `RegReadBytes`（`353`）都是「中间 ACK、最后 NACK」，**旧实现反了的那处没有重犯**。
2. **`bsp_si2c.c` 信号电平顺序**（任务重点 4）：START/STOP/重复起始/写位/读位/ACK 位全部符合 UM10204，SDA 只在 SCL 低时改变，MSB first，SDA 在读之前被释放；每个边沿都有显式半周期延时。（实际宽度受 P0-1 影响，见 1.4。）
3. **`bsp_si2c.c` 初始化顺序**：先使能端口时钟 → 先写 ODR=1 → 再配开漏模式（`225-246`），不会产生低电平毛刺；`HalfPeriodUs==0` 有保底（`219-223`）。`SI2C_EnablePortClock` 覆盖 GPIOA~E，够用。
4. **ADDR 清标志的写法**：`I2C_ClearAddrFlag()` 先读 SR1 再读 SR2（`bsp_i2c.c:165-169`），与 RM0008/SPL 注释（`stm32f10x_i2c.c:1203-1205`）一致；`I2C_GetFlagStatus` 只读单个寄存器（`stm32f10x_i2c.c:1132-1176`），等待标志不会误清 ADDR。
5. **多字节接收的字节数与缓冲下标**：`Size>=3` 的 `for(i=1; (i+2)<Size; i++)` + 末尾 `Size-2`、`Size-1`，对 Size=3/4/14 逐一推演正确，无越界、无漏读、无多读；`Size==2` 的两个字节也落在 `Size-2/Size-1`（逻辑正确，缺的只是清 ADDR，见 P0-3）。
6. **发送路径的收尾**：`My_I2C_SendBytes`/`MemWriteBytes` 在发完最后一字节后等 BTF 再发 STOP（`386-394`、`532-540`），符合 RM0008「BTF 后再 STOP 不会截断最后一字节」；重复起始前等 TXE（`439-457`）也是 AN2824 的标准做法。
7. **除临界区外所有等待都有超时**，没有裸 `while` 死等；错误路径统一发 STOP（`I2C_Abort`）把总线放回空闲。
8. **毫秒 tick 唯一写入者**：只有 `SysTick_Handler → Delay_TickInc()`（`stm32f10x_it.c:164-169`）；`Delay()` 用无符号差值可跨 49.7 天回绕；`DelayUs` 「至少延时 us」的语义成立。
9. **Q8 换算精度**：整数 MHz 下 `cycles_per_us_q8` 精确（72 MHz → 18432 → 等价于 `cycles/72`），无累积误差；`(uint64_t)elapsed << 8` 无溢出。
10. **DWT/SysTick 初始化序列**：`DEMCR.TRCENA → CYCCNT=0 → CTRL.CYCCNTENA` 符合 DDI0337；SysTick「先停表→清 VAL→写 LOAD→设优先级→最后写 CTRL」顺序正确，优先级在 `NVIC_PriorityGroupConfig` 之后设置（`main.c:173/175`）。
11. **`GetUs()` 本身可重入、不关中断**（原实现「在临界区里调用会重新开中断」的硬伤确实修掉了）——但由此产生的超时问题见 P0-2。
12. **OLED 帧缓冲下标与位运算**：`page*128 + x`、`1<<(y&7)`、字模「按列取模、bit0 在上」三者自洽；独立解码 `bsp_font6x8.c` 验证 95 个字形、无 bit7 置位、'A'/'5'/'g'/'.' 形状正确。
13. **OLED 初始化命令取值**：逐条与 SSD1306 命令表核对均合法；控制字节 `0x00` + 多条命令合并到一个 I2C 传输是手册允许的（唯一问题是 `0x20` 的取值与刷新实现不匹配，见 M1）。
14. **OLED 边界与内存**：`OLED_FlushAll`/`OLED_Pump` 的临时缓冲均不越界；`SetPixel` 有边界裁剪；`OLED_DrawString` 到底部即停；`OLED_Printf` 由 `vsnprintf` 保证 NUL 结尾（截断安全）；帧缓冲在调用方结构体里，无 `malloc`。
15. **`bsp_usart.c` 的接收边界**（任务重点 7）：`ReceiveLine` 的写入下标最大 `MaxLength-1`、末尾必补 `'\0'`、CRLF 仅在 `"\r\n"` 上成功、`MaxLength` 与 CRLF 的最小长度校验正确、`ret` 默认 -1 且有未初始化路径的旧问题已消除；`ReceiveBytes` 的超时/计数正确；`SendBytes` 等 TC 正确；`Printf` 的截断长度处理正确（`112-119`）。
16. **通用**：未发现数组越界、整数溢出、无符号回绕误用（时间差一项见 M3）、位运算错用、可重入数据竞争（`bsp_oled` 的帧缓冲与 `Pump` 由 `Busy` 互斥，`user/app_oled.c:109-116` 的调用约束与之一致）。

---

## 4. 验证记录（本次复核实际执行）

| 手段 | 结果 |
|---|---|
| `tools\compile.ps1` | **51 / 51 通过**，输出中 warning/error 行数 = 0 |
| `tools\link.ps1` | **成功**，产物 `build\template.axf`、`build\template.hex`；`Total ROM Size = 27620 B (26.97 kB)`、`Total RW Size = 6296 B (6.15 kB)`（预算 45 kB / 12 kB 内） |
| 字模独立解码 | 解析 `bsp_font6x8.c` 的 95 个字模，'A'/'5'/'g'/'.' 渲染正确，bit7 全 0，与驱动取值方式一致 |
| SPL 交叉核对 | `stm32f10x_i2c.c:1132-1176`（标志只读单寄存器）、`:1200-1205`（BTF/ADDR 清除序列）、`stm32f10x_i2c.h:346-347`（EV6 事件）、`:373-376`（EV7 未及时处理会置 BTF、通信被拉长）、`:382`（EV7 定义） |
| 旧实现对照 | `my_lib/i2c.c`（临界区用循环计数、`247` 先置 ACK 再清 ADDR）、`my_lib/oled.c:935-945`（水平模式用 `0x21/0x22`）、`my_lib/delay.c:80-101`（SysTick 余量法补微秒） |
| 外部佐证 | SSD1306 水平寻址下页/列命令被忽略、靠顺序侥幸正确：[u8g2 issue #1362](https://github.com/olikraus/u8g2/issues/1362) |

未执行：上板实测（本任务为只读静态审查，且无硬件）。因此 P0-4 的实际发生率、OLED 实际刷新耗时、编码器读数误差这三项以代码推算给出，需上板复核（见下）。

---

## 5. 建议修复顺序与上板复测清单

**修复顺序（建议）**

1. **P0-1 + P0-2 一起修**（同一个地基）：`GetUs` 改为「tick*1000 + SysTick 余量（或 DWT - 本拍起点）」，保证**关中断期间时间也推进**；然后 `I2C_WaitFlagCritical` 的超时才真正有效。
2. **P0-3**：`I2C_ReadData` 统一负责三条分支的 ADDR/ACK 时序（含 `RegReadBytes` 路径）。
3. **P0-4**：每笔传输开始前 `ACK=ENABLE`，收尾恢复 `ACK=ENABLE`；错误路径同样恢复。
4. **M1**（OLED 寻址模式统一）、**M2**（PRIMASK 保存恢复）、**M3**（时间戳同源/单调）。
5. 其余低优先级项按需清理；`L1`（抢救波形）与 `L2`（命名）建议顺手做掉。

**上板复测清单**

1. 示波器/逻辑分析仪测 PB8/PB9：修复前 SCL 周期应在 ms 量级（≈1 kHz），修复后应为 2 µs 半周期（≈250 kHz）；同时确认 t<sub>LOW</sub>/t<sub>HIGH</sub>/t<sub>SU;STA</sub> 满足快速模式最小值。
2. 在 `App_MPU6050_Update` 临时统计 `My_I2C_RegReadBytes(...,14)` 的返回值分布：修复前若出现隔次 -3、且耗时约 10 ms，即命中 P0-4。
3. 临时调用 `My_I2C_RegReadBytes(I2C2, 0xD0, 0x75, buf, 2)`：修复前应**卡死**（验证 P0-3 + P0-2），修复后返回 0 且首字节正确。
4. 抓一次 14 字节读的波形，核对：地址 ACK、第 0 字节 ACK、最后字节 NACK、STOP 位置。
5. 测 `App_OLED_Init` 的整屏耗时与 `App_OLED_Pump` 单次耗时（修复前 ~15 s / ~100 ms，修复后应分别 < 200 ms / < 1 ms）。
6. 在 39 rad/s 附近对比编码器读数与转速计的偏差（验证 M3/P0-1 对 `ENC_MIN_EDGE_US` 判据的影响）。
7. 修复 P0-2 后，人为拉低 SDA 制造总线异常，确认超时能返回 -3 且中断已恢复（不再死机）。

---

## 6. 结论

- **可以判定为「必须修完才能上板」的缺陷**：P0-1、P0-2、P0-3、P0-4（其中 P0-1/P0-2 影响所有模块，P0-3 是潜伏死锁，P0-4 命中当前唯一的传感器读路径）。
- **必须随批修正的中级项**：M1（SSD1306 寻址模式混用）、M2（PRIMASK）、M3（时间戳单调性）。
- **数值/接口层面质量不错**：帧缓冲下标与位运算、字模位序、串口行接收边界、Q8 换算、DWT/SysTick 初始化顺序、软 I2C 的 ACK/NAK 与信号顺序都经得起核对；编译 51/51、链接通过且体积比旧实现更小。
- 本次复核**未修改 `bsp/` 下任何文件**，仅新增本报告。

---

## 7. 修复记录（Lead 追加）

本节由 Lead 在收到本报告后追加，记录每一项的处置结果与验证方式。
改动后重新跑过 `tools\compile.ps1`（51/51 通过、0 warning）与 `tools\link.ps1`（链接成功）。

### 7.1 高危：已全部修复

| 编号 | 问题 | 处置 |
|---|---|---|
| **P0-1** | `GetUs()` 微秒分量恒为 0 | **已修**。彻底放弃「DWT 两次读相减」的做法，改成纯 SysTick 时基：`GetUs = delay_ticks*1000 + (LOAD - VAL)*1000/cycles_per_ms`。因为 SysTick 重装与中断递增是同一事件的两面，「先读毫秒戳再读 VAL」不会跳变，也不需要重试。DWT 相关代码（`DWT_Init` / `DWT_Probe` / `dwt_ready`）已整段删除。`bsp_delay.h` 的接口说明同步改写 |
| **P0-2** | 临界区内超时永不到期 → 死循环且中断一直关着 | **已修**。`bsp_i2c.c` **不再关中断**：删除 `I2C_WaitFlagCritical` 及其专用超时宏，`I2C_ReadSingleByte` / `I2C_ReadLastTwoBytes` 去掉 `__disable_irq()`/`__enable_irq()`，统一走带超时的 `I2C_WaitFlag`。文件头注释说明了「为什么不关中断」以及原子性靠什么保证 |
| **P0-3** | `RegReadBytes(Size==2)` 从不清 ADDR | **已修**。`I2C_ReadData()` 重排为：Size==1 走单字节路径（关应答→清 ADDR→发 STOP）；Size>=2 一律**先置应答位、再清 ADDR**，之后 Size==2 先读掉第 0 个字节再交给 `I2C_ReadLastTwoBytes`，Size>=3 继续按原循环读取 |
| **P0-4** | ACK 位残留 DISABLE（首字节竞态） | **已修**。同上：`I2C_AcknowledgeConfig(ENABLE)` 移到 `I2C_ClearAddrFlag()` **之前**，两条接收路径都覆盖 |

### 7.2 中级：已修复

| 编号 | 问题 | 处置 |
|---|---|---|
| **M1** | SSD1306 水平寻址与页寻址命令混用 | **已修**。全模块统一到水平寻址模式：新增 `SSD1306_CMD_SET_COL_ADDR (0x21)` / `SSD1306_CMD_SET_PAGE_ADDR (0x22)`，`OLED_FlushAll()` 先设整屏窗口再一次性发 1024 字节，`OLED_Pump()` 每段先按 `Progress` 设置本段窗口。不再出现 `0xB0+page` 与单字节列地址 |
| **M2** | 临界区不保存/恢复 PRIMASK | **已修**。`user/app_encoder.c` 两处临界区（`Encoder_GetCount`、`Encoder_GetSpeed` 的状态快照）改成 `primask = __get_PRIMASK(); __disable_irq(); ... __set_PRIMASK(primask);`。这两个函数当前只在主循环调用，所以不存在现实缺陷，但去掉这个隐患 |
| **M3** | `GetUs` 单调性不成立 | **已修**，随 P0-1 一并解决。用 SysTick 模型做了数值验证：连续 1us 采样差值恒为 1；跨毫秒重装边界无跳变；跑满 1 秒每 100us 采样**零次倒退**。另外确认 `user/app_balance.c:226-233` 本来就把 `dt` 限幅在 50ms，即使将来再出现异常也不会把积分带飞 |

### 7.3 低级：部分修复，其余记录为已知限制

**已修**：
- `OLED_SetCursor` 上边界夹到屏外 → 改为夹到 `WIDTH-1` / `HEIGHT-1`。

**评估后不改，记录为已知限制**（均不在本次驱动自研化的范围内，或需要硬件才能定标）：

| 项 | 为什么不改 |
|---|---|
| `bsp_si2c.c` 不处理时钟延展 | 本板软 I2C 上只挂 OLED（只写、不做时钟延展）。核心 `SI2C_HalfDelay()` 已经把半周期放慢到 2us，实际给了从机余量。若将来接会拉 SCL 的器件，需要在 `SI2C_WriteByte`/`SI2C_ReadByte` 里加「等 SCL 真的变高」 |
| `bsp_usart.c` 发送无超时 | 该文件是纯调试输出通道，硬件上没有流控，TXE 在 115200 下几微秒就会置位。发送路径在中断里调用过（`busy` 标志用于避免重入） |
| `I2C_WaitFlag` 参数语义易读错 | 已加注释说明调用点把「错误标志」放在前面、两标志同时置位时按出错处理。属可读性问题 |

### 7.4 复核结论的落地状态

- 报告的 P0-1/P0-2/P0-3/P0-4、M1/M2/M3 **全部已修复并重新编译链接通过**。
- 报告第 5 节的 7 项上板复测清单**仍然有效**，尤其是第 5、6、7 项（OLED 刷新耗时、编码器读数偏差、总线异常时能否返回 -3 而不死机）—— 这三项只能上板确认。
- 修改后的体积：`Total RO = 27324 B`（Code + RO Data，26.68 kB），`Total RW = 6304 B`（6.16 kB），
  相对重构前基线 `RO = 37960 B` 减少约 28%。

---

## 8. 第二轮复核（修复验证）

- 复核范围：**只核对本次修复**（`bsp_delay.c/.h`、`bsp_i2c.c`、`bsp_oled.c`，抽查 `user/app_encoder.c`），未重新全量审查，未修改 `bsp/` 下任何文件。
- 验证手段：`tools\compile.ps1` → **51 / 51 通过、0 warning**；`tools\link.ps1` → **链接成功**，`template.map`：`Total RO = 27324 B`、`Total RW = 6304 B`、`Total ROM = 27540 B`（与 §7.4 数字一致）。
- 另运行了 Lead 提供的两个脚本，并新增三组独立数值模型（当前 C 控制流忠实回放 / 新 `DelayUs` 边界 / SysTick 重装+中断延迟时基模型），结论见 8.1、8.3、8.8。
- **总判定**：P0-1 ✓（含口径修正）、P0-2 ✓、P0-4 ✓、M1 ✓、M2 ✓、M3 部分 ✓；**P0-3 的「不清 ADDR」已修，但 `Size==2` 变成新的功能缺陷（高）**；**新引入 `DelayUs(us≥1001)` 死循环（中）**。修掉 8.8 的两条即可整体通过。

### 8.1 P0-1（`GetUs` 微秒分量为 0）—— **已修复且正确**（附一处口径修正）

**已修且正确的部分**

- 公式与换算正确：`GetUs = delay_ticks*1000 + (systick_reload - SysTick->VAL)*1000/systick_cycles_per_ms`（`bsp_delay.c:56-67`、`135-146`）。`systick_reload = LOAD+1`，`elapsed ∈ [1, 72000]` → `sub_us ∈ [0, 1000]`，`elapsed*1000` 最大 7.2e7 ≪ 2³² **不会溢出**；`(uint64_t)delay_ticks*1000` 也不会溢出；`systick_cycles_per_ms == 0` 有独立保护（`60-63`）；`delay_init_done` 移到函数**最后**置位（`116`），顺带修掉了第一轮 L4 的「除数未就绪」窗口。**已核查，无误**。
- 分辨率恢复：µs 部分按 1 µs 步进（72 MHz 下 72 周期/µs），`bsp_delay.h:42-44` 的描述与实际一致。`DelayUs(2)` 不再是「睡到下个毫秒边界」，按代码回放实际约 2~3 µs → 软 I2C 半周期回到设计值、总线约 170~250 kHz（`user/app_oled.c:13` 的 `HalfPeriodUs = 2` 才真正生效）。附带收益：轮询循环里的 64 位除法消失（`GetUs` 现在只有一次 32 位乘除），400 kHz 下 `I2C_WaitFlag` 的轮询延迟从「µs 级」降到「百 ns 级」，对 8.3 的首字节 ACK 余量是加分项。
- 不再依赖中断控制寄存器：`GetUs` 纯读内存与硬件计数器，`bsp_delay.h:10-11` 的「临界区内调用安全、不改变中断状态」成立；DWT 相关代码（`DWT_Init`/`DWT_Probe`/`dwt_ready`/`cycles_per_us_q8`）已整段删除，`bsp_delay.c:23-27` 也把「为什么原来错了」写清楚了。**已核查，无误**。

**口径修正（残留窗口，非新引入缺陷，但 §7.2「不会跳变」的说法过强）**

- `SysTick` 硬件重装（`VAL` 跳回 `LOAD`）与「中断里 `delay_ticks++`」**不是同一瞬间**：中断有延迟 L。在窗口 `[T_k, T_k + L)` 内采样，会读到「**旧的**毫秒戳 + 刚重装的小余量」= 比真实值小最多 ~1000 µs。
- 数值模型（`T=1ms`、每 1 µs 采样 20 ms）：`L = 0` → 倒退 **0** 次；`L = 0.2 / 0.5 / 1.0 / 3.0 µs` → **每个重装边界各 1 次**，最大幅度 **999 µs**。也就是说 §7.2 的「跑满 1 秒零次倒退」只在 `L = 0` 的理想模型下成立；真实硬件 `L` 是「硬件重装 → 异常入口 → `delay_ticks++`」的时间，通常 0.3~1 µs（SysTick 抢占优先级 0 最高，`user/` 里 EXTI=2、USART2=3，不会被它们抢占；只有 `app_encoder.c` 那两处 ~1 µs 的临界区会把它拉长）。
- 影响（按本项目调用面推算）：概率 ≈ `L/1000`（≈0.03~0.1% 的采样落在窗口内）。
  - 采样间隔 ≥ 1 ms 的调用方（`my_lib/pid.c:56` 的 `deltaT`、`user/app_balance.c:221-233` 的 `dt`，都是 5 ms 任务）只会「少算 ≤1 ms」，**有界、无害**（`app_balance` 另有 50 ms 限幅）。
  - 采样间隔 < 1 ms 的调用方只有 `user/app_encoder.c:73` 的中断边沿（满速时边沿间隔 ≈770 µs）：`now - t_last` 会得到无符号回绕后的巨大值 → 表现为**一拍「速度=0」**（`app_encoder.c:162` 判停）+ 一个偏低 1 ms 的边沿时间戳，被 `ENC_TIMEOUT_US` 与调速环的变化率限制吞掉，不构成安全风险。
- 建议（任选，推荐 ①②同做）：① 在 `my_lib/pid.c` 的 `deltaT` 上加兜底（例如 `deltaT` 超过一个合理上限时沿用上一拍的值），把「任何来源的异常 dt」都挡在积分器之外；② 若要把窗口也闭合：让 `SysTick` ISR 在自增后读一次 `SysTick->CTRL`（清 `COUNTFLAG`），`GetUs` 采样前后各读一次 `COUNTFLAG`，若发现「重装已发生但中断尚未记上」则 `tick + 1`（seqlock 形式；`cf` 前后不一致则重读）；③ 至少把 `bsp_delay.c:17-21`、`bsp_delay.h:7-9` 的表述从「不会跳变」改成「误差上界 = 中断延迟 L（通常 <1 µs），不再是 1 ms」。
- 另有一处**理由写反**（低）：`bsp_delay.c:142-144` 说「先读 VAL 再读毫秒戳会得到『新的亚毫秒值 + 旧的毫秒值』→ 倒退 1 ms」。实际该组合是「旧的亚毫秒值 + 新的毫秒戳」= **超前** ~1 ms。当前「先 tick 后 VAL」的顺序确实是更安全的一侧（误差方向是落后而非超前，对无符号差值更友好），**结论对、理由错**，建议改写。

### 8.2 P0-2（临界区超时永不到期）—— **已修复且正确**

- **死锁根因消失**：全局搜索确认 `bsp_i2c.c` 已没有任何 `__disable_irq()/__enable_irq()`，`I2C_WaitFlagCritical` 与 `I2C_LAST_BYTE_TIMEOUT_US` 已删除；`I2C_ReadSingleByte`（`214-228`）与 `I2C_ReadLastTwoBytes`（`243-267`）全部改用带 10 ms 超时的 `I2C_WaitFlag`。因此「关中断 → 时间轴停 → 超时永不成立」这条链断了 ✓。文件头 `bsp_i2c.c:16-21` 把「为什么不关中断」写清楚了，与实现一致 ✓。
- **去掉临界区后原子性仍站得住（同意 Lead 的论证，且能给出依据）**：`RXNE` 由硬件置位，`I2C_ReceiveData()` 是单次外设访问；软件来不及处理 EV7 时，F1 硬件会置起 `BTF` 并拉低 SCL 等软件（本仓库 SPL 自带说明：`std_periph_driver/inc/stm32f10x_i2c.h:373-376`「若软件不保证在当前字节结束前处理 EV7，可同时判 EV7 与 BTF……通信会变慢」，BTF 的清除序列见 `std_periph_driver/src/stm32f10x_i2c.c:1200-1202`）。所以中断插进来最多让最后一个字节晚一点读到，**不会丢、不会多读**。
- **残留一处极窄的原子性缺口（低，可选加固）**：`I2C_ReadLastTwoBytes` 的「读 DR（`253`）→ 关 ACK（`255`）→ 发 STOP（`256`）」三步之间若被一个**长于一个 I2C 字节时间**（400 kHz ≈ 22.5 µs）的中断/流程拖住，最后一字节会被 ACK 而不是 NACK，从机会多送一个字节，`line 259` 的等待就可能读到那个多出来的字节 → 最后一字节错位。当前工程里没有这么长的 ISR（EXTI 几 µs、USART2 只是环形缓冲入队），三步窗口本身只有约 0.1 µs，属低概率事件。若要完全闭合，可只把这两行用一个**不含任何等待**的极短临界区包起来（`primask = __get_PRIMASK(); __disable_irq(); ACK=0; STOP; __set_PRIMASK(primask);`）——因为里面没有等待，不会重现 P0-2 的死锁。
- 另附一条维护性提示（低）：`I2C_ReadSingleByte`/`I2C_ReadLastTwoBytes` 正常返回与超时返回后都没有把 ACK 恢复成 ENABLE，模块空闲态仍是 `ACK=0`。目前靠「每个入口先置 ACK 再清 ADDR」（`285-286`、`216-217`）兜住，无现实缺陷；但建议收尾统一恢复，避免以后新增入口踩坑。

### 8.3 P0-3 + P0-4（ADDR 清除 / ACK 顺序）—— **Size=1 / 3 / 14 已修复且正确；Size==2 仍有问题（本次修复新引入的缺陷，高）**

**已修且正确的部分**

- **ADDR 清除时机**：`My_I2C_RegReadBytes` 读方向进入 `I2C_ReadData` 时 ADDR 仍置（`455-466` 只等不清），由 `I2C_ReadData` 自己清：Size==1 走 `I2C_ReadSingleByte:217`（关应答→清 ADDR→STOP，符合 RM0008 EV6_1）；Size>=2 在 `285-286` 清；Size>=3 依此继续。`My_I2C_ReceiveBytes` 路径 ADDR 虽已在地址阶段（`201`）清掉，`286` 再清一次是空操作，无害。**第一轮的「Size==2 死等」诱因（ADDR 从不清）已消除** ✓。
- **ACK 顺序**：`I2C_AcknowledgeConfig(ENABLE)`（`285`）现在位于 `I2C_ClearAddrFlag()`（`286`）**之前** → 第 0 个字节开始接收前 ACK 已是 1，而它的应答时隙还要再等约 8 个位时间（400 kHz ≈ 20 µs）→ 余量充足 ✓。`ReceiveBytes` 路径虽然 ACK 是在地址阶段清 ADDR（`201`）之后约 1 µs 才置（一次函数调用），距应答时隙同样有 ~19 µs ✓ 两条入口都安全。**第一轮的 P0-4 首字节竞态已消除** ✓。
- **字节数与下标**（逐条按当前代码回放）：
  - Size==1：1 次读 → `pBuffer[0]` ✓；
  - Size==3：`pBuffer[0]` + `ReadLastTwoBytes` 的 `pBuffer[1]`、`pBuffer[2]` ✓（循环 `(i+2)<Size` 不执行）；
  - Size==14：`pBuffer[0]` + 循环 `i=1..11` + `pBuffer[12]`、`pBuffer[13]` = 14 字节 ✓；
  - 每个字节被接收前 ACK 位都是它需要的值（第 0 字节 ACK、最后一个 NACK，NACK 在最后一个字节**之前**置好）✓。

**仍有问题：`Size == 2` 多读一个字节（新缺陷）**

- 【文件:行号】`bsp_i2c.c:288-302`（预读第 0 字节）+ `bsp_i2c.c:243-267`（`I2C_ReadLastTwoBytes` 固定再读 2 个）
- 【问题描述】`I2C_ReadData` 的 `Size==2` 分支先自己「等 RXNE 读掉第 0 个字节」（`293-299`），然后调用 `I2C_ReadLastTwoBytes`；而该函数的契约就是读**最后两个字节**（下标 `Size-2`、`Size-1`）——对 `Size==2` 而言这两个下标正是 `pBuffer[0]`、`pBuffer[1]`，也就是**全部请求字节**。于是 2 字节的请求共发生 **3 次** `I2C_ReceiveData()`，写入下标依次为 `pBuffer[0]`、`pBuffer[0]`（覆盖）、`pBuffer[1]`。
- 【为什么会出问题（两套独立回放，均按当前代码的控制流）】
  - **从机守规范**（收到 NACK 就停，MPU6050 属此类）：第 3 次等 RXNE 永远等不到 → **10 ms 超时 → 返回 -3**，`pBuffer = [0x11, 未写入]`；
  - **从机继续吐**（正是 `tools/_verify_i2c_read.py` 里那个「只要 RXNE 空就供数据」的从机模型）：第 3 次读成功 → **返回 0，但 `pBuffer = [0x11, 0x12]`** —— 首字节丢失、整体错位、多读一个字节，**静默错误数据**（对调用方比 -3 更危险）；
  - 正确结果应为 `pBuffer = [0x10, 0x11]`。
  - 依据：`I2C_ReadLastTwoBytes` 的注释与实现都把它定义为「收尾两个字节」（`253` 写 `Size-2`、`264` 写 `Size-1`），因此 `288-300` 的预读既多余又有害。**修复前的代码在 `Size==2` 分支是直接调用 `I2C_ReadLastTwoBytes`（字节数/下标正确，只是 ADDR 没清）**，所以这是本次修复**新引入的回归**，而不是原有问题。
- 【严重程度】**高**（公开 API 的功能性错误：要么必然 -3 + 10 ms 阻塞，要么静默错位数据；当前 `user/` 只用到 Size=14/1，属潜伏）
- 【建议修法】二选一：
  1. **最小改动**：删掉 `288-302` 的预读，让 `Size==2` 直接 `return I2C_ReadLastTwoBytes(I2Cx, pBuffer, Size);`（此时 ADDR 已在 `285-286` 清好、ACK=1 → 第 0 字节被 ACK、第 1 字节被 NACK + STOP，字节数与下标都对）；
  2. 若一定要保留预读，则最后一字节必须另写一段（`ACK=0 → STOP → 等 RXNE → pBuffer[Size-1] = DR`），不能再调用 `I2C_ReadLastTwoBytes`。
- 【顺带修正 Lead 的验证脚本】`tools/_verify_i2c_read.py` 的 `run_case()`（`91-123`）对 Size==2 回放的是「循环 size 次读」的**旧结构**，与当前 C 代码不一致（当前是 1 次预读 + 2 次收尾 = 3 次读），所以它输出的 `字节序正确: True` **不能作为 Size==2 的修复证据**；另外 `nack_events` 只计算、既不打印也不断言（`107-108`、`125-132`）。建议改为按当前分支结构回放，并加断言 `nack_events == [size-1]`、`got == payload[:size]`（我把 C 流程忠实回放后，Size==2 立刻暴露；Size=1/3/14 仍正确 ✓）。

### 8.4 M1（SSD1306 寻址模式混用）—— **已修复且正确**

- ✓ 全模块统一到水平寻址：仍设 `0x20,0x00`（`bsp_oled.c:419`），刷新改用水平模式命令 `0x21`/`0x22`（宏定义 `38-39`；`OLED_FlushAll` 的 `83-89`；`OLED_Pump` 的 `345-351`）；`0xB0+page` 与单字节列地址已从刷新路径彻底消失 ✓。
- ✓ **窗口边界算法正确**：`OLED_Pump` 中 `col_s = start%128`、`col_e = 127`、`page_s = start/128`、`page_e = (start+chunk-1)/128`（`347-351`），窗口起点 `page_s*128 + col_s == start` ✓，窗口容量 `(127-col_s+1) + (page_e-page_s)*128 ≥ chunk` ✓（我独立复算 128 段全部成立，与 `tools/_verify_oled_chunks.py` 输出一致：每段起点精确对齐、覆盖 1024 字节、无 BAD）。因为实际写入量 = `chunk ≤ 容量`，不会触发 SSD1306 的窗口回绕 ✓。
- ✓ `OLED_FlushAll`：先 `0x21,0,127` + `0x22,0,7` 设整屏窗口，再单次写 1024 字节（`83-105`）；水平模式下指针线性自增、页尾自动翻页，下标与帧缓冲一一对应 ✓。
- ✓ 失败恢复语义现在**真的成立**了：每段重设窗口（`340-342` 的声明）能把 SSD1306 被中断打断后残留的地址指针拉回，第一轮 M1 里「部分刷写后整帧错位」的风险消除 ✓。
- ⚠ 附注 1（低，需记录）：`OLED_FlushAll` 现在在栈上放 `uint8_t buffer[1025]`（`77`）并做 1026 字节单次 I2C 传输。当前构建设置里 `startup/startup_stm32f10x_md.s:35` 的 `Stack_Size = 0x1000`（4 KB，`tools/link.ps1:33` 正是汇编这个文件）→ 够用 ✓；但仓库里另有 4 个 startup 变体是 `0x400`（1 KB，如 `startup_stm32f10x_md_vl.s:35`），一旦换启动文件就会栈溢出。建议在注释里显式写明「依赖 Stack_Size ≥ 4 KB」，或改成静态缓冲/分段发送（更稳）。
- ⚠ 附注 2（低）：文件头注释 `bsp_oled.c:14` 仍写「每段都要重新发一次『设置页地址』命令」，与新实现（列+页窗口）不一致，建议顺手更新。

### 8.5 M2（PRIMASK）—— **已修复且正确**

- ✓ `Encoder_GetCount`（`user/app_encoder.c:115-120`）与 `Encoder_GetSpeed`（`143-155`）改为 `primask = __get_PRIMASK(); __disable_irq(); ... __set_PRIMASK(primask);` —— 语义正确（进入时若本来就关中断，退出时不会擅自打开）✓；`bsp_i2c.c` 已无临界区，该项在那里自然消失 ✓。
- 附注（低）：同类写法还剩 `App_Encoder_ClearLeft/Right`（`app_encoder.c:328/333`、`338/343`）用裸 `__disable_irq()/__enable_irq()`。这两个函数（含 `App_Encoder_ClearAll`）在 `user/`、`bsp/`、`test/` 里都**没有调用点**（死代码），所以无现实影响；建议顺手统一或删除。

### 8.6 M3（单调性）—— **部分修复**；§7.2 的验证模型不足以支撑「零次倒退」

- 主症已解决 ✓：不再依赖 DWT，`GetUs` 不会每 59.6 s 在回绕点整拍倒退；1 ms 分辨率问题也随 P0-1 一并消失（见 8.1）。
- **残留**：重装瞬间的窗口仍然存在（8.1 已量化：`L > 0` 时每个重装边界都会产生一次约 -999 µs 的采样），因此 `bsp_delay.h:43` 的「做差值运算时按无符号处理即可」在「采样间隔 < 1 ms」的调用点（`app_encoder.c` 的中断边沿）仍会回绕。
- §7.2/§7.3 引用的「SysTick 数值模型」未随脚本一起提交（`tools/` 下只有 `_verify_i2c_read.py`、`_verify_oled_chunks.py`，都不是时基模型），且「重装与中断递增是同一事件的两面」等价于把 ISR 延迟 L 设为 0；我的复算显示 `L=0 → 0 次倒退，L>0 → 每拍 1 次约 999 µs 倒退`。建议把该条结论改成「已修复主症（1 ms 分辨率 + 59.6 s DWT 回绕），残留重装窗口概率 ≈ L/1000、幅度 ≤1 ms，调用方加兜底或按 COUNTFLAG 补偿」，并把模型脚本落盘到 `tools/`。

### 8.7 低级项复核

| 项 | 判定 | 依据 |
|---|---|---|
| `OLED_SetCursor` 上界夹到屏外 | **已修复且正确** | `bsp_oled.c:221-243` 改为 `WIDTH-1/HEIGHT-1`，注释同步 |
| `I2C_WaitFlag` 参数语义易读错 | **仍有问题（低）** | 函数自己的注释 `bsp_i2c.c:96` 仍写「等到 Flag 变成 State」，而实现是 `while(I2C_GetFlagStatus(...) == State)`（等它**离开** State）。§7.3 说「已加注释」实际只覆盖了调用点的 AF 优先级说明（`114-115`、`185`）。照这条注释改循环就会死锁，建议改成「等待 Flag 不再等于 State（置位/清零）」 |
| `My_I2C_ResetBus` 毛刺 + 伪 START | **未改（低，可接受）** | `bsp_i2c.c:567-568`（先配模式后写 ODR）、`582-587`（SCL 仍为高时先拉低 SDA = 一个 START）。不属本次修复范围，与 §7.3 的取舍一致 |
| 空指针返回「成功」 | **未改（低，建议后续顺手修）** | `bsp_i2c.c:391-394`、`bsp_si2c.c:290-293/320-323`；当前调用点都不为空 |
| `bsp_si2c` 不处理时钟延展 / 串口发送无超时 | **未改（低，接受 §7.3 的理由）** | 本板软 I2C 只挂 OLED（只写、不延展）；串口是调试通道且 `TXE` 在上电后恒会置位 |

### 8.8 新引入缺陷汇总（本轮重点，必须处理）

1. **（高，新）`My_I2C_RegReadBytes` / `My_I2C_ReceiveBytes` 的 `Size==2` 路径多读一个字节。**
   【文件:行号】`bsp_i2c.c:288-302` + `243-267`。
   【后果】从机守规范 → 10 ms 超时返回 -3（`pBuffer=[0x11,未写]`）；从机继续吐 → 返回 0 但数据整体错位（`[0x11,0x12]`）。
   【修法】见 8.3 ①：`Size==2` 直接交给 `I2C_ReadLastTwoBytes`，不要再预读。
   【回归用例】`Size==2` 加进 `_verify_i2c_read.py`（按当前分支结构回放 + 断言字节序列），并考虑在 `user/` 侧留一个上板用例。

2. **（中，新）`DelayUs(us)` 在 `us ≥ 1001` 时必然死循环、`us == 1000` 时可能死循环。**
   【文件:行号】`bsp_delay.c:151-162`（`Delay_CyclesBetween` 把 `diff_us > 1000` 直接判 0，返回值上限 `1000 * 72 = 72000`）+ `bsp_delay.c:195-202`（`want_cycles = us * (cycles_per_ms/1000)`，72 MHz 下即 `us*72`）。
   【为什么会死循环】`us > 1000` → `want_cycles > 72000` → 循环条件 `Delay_CyclesBetween(...) < want_cycles` **永远成立**（函数只会返回 0 或 ≤72000）；`us == 1000` 需要某次采样**恰好**落在 `diff_us == 1000` 才能退出。
   【数值验证】`DelayUs(2/5/100/999/1000)` 正常（按 1 µs 步进退出，`DelayUs(2)` ≈ 2 µs ✓）；`1001/5000/60000` **一律挂死**；`1000` 在「轮询间隔 1.0~1.9 µs 抖动 + 周期性 8 µs 中断」的 24 组组合里**挂了 14 组**。另外 `want_cycles = us * 72` 在 `us > 59.65e6` 时还会 32 位溢出（当前实现下先挂死，暂时观察不到）。
   【当前影响】唯一调用点是 `bsp_si2c.c:52`（`HalfPeriodUs = 2`，且 `bsp_si2c.c:220-223` 对 0 有保底）→ **不影响现有功能**；但这是修复引入的 API 级死循环：只要有人把 `HalfPeriodUs` 调到 >1000（`uint16_t` 允许），或在别处用 `DelayUs` 做毫秒级等待，就会挂死。
   【修法】`DelayUs` 直接用 µs 差值比较即可（1 µs 分辨率与现状等价，因为亚微秒信息本来就被丢弃）：
   ```c
   void DelayUs(uint32_t us)
   {
       uint64_t start = GetUs();
       while((GetUs() - start) < (uint64_t)us) { }
   }
   ```
   若要保留周期数比较，则 `Delay_CyclesBetween` 不能把跨度限在 1000 µs（应返回完整 64 位差值），并把「差值异常」与「时间到」分开处理，绝不能返回 0 让循环继续转。

### 8.9 第二轮验证记录

| 手段 | 结果 |
|---|---|
| `tools\compile.ps1` | **51 / 51 通过**，warning/error 行数 0 |
| `tools\link.ps1` | **成功**；`template.map`：`Total RO 27324`、`Total RW 6304`、`Total ROM 27540`（与 §7.4 一致） |
| `tools\_verify_i2c_read.py` | 输出 Size=1/2/3/14 全部 `字节序正确: True`；但按 8.3 的分析，它对 Size==2 回放的是旧结构，**该结论不成立**（脚本也无法发现多读一字节，因为它只循环 `size` 次读） |
| `tools\_verify_oled_chunks.py` | 128 段全部 `OK`、覆盖 1024 字节、窗口起点精确对齐 `start` ✓ 与我的独立复算一致 |
| 独立模型 ①（本轮新增） | 按**当前 C 代码分支结构**忠实回放：Size=1/3/14 字节序列与返回值正确；Size==2 → 「守规范从机」得 `buf=[0x11,--] ret=-3`，「继续吐的从机」得 `buf=[0x11,0x12] ret=0`（应为 `[0x10,0x11]`） |
| 独立模型 ②（本轮新增） | 新 `DelayUs` 边界：`2/5/100/999/1000` 正常；`1001/5000/60000` 必然挂死；`1000` 在 24 组抖动/中断组合中挂 14 组 |
| 独立模型 ③（本轮新增） | SysTick 重装 + ISR 延迟 L：`L=0` → 0 次倒退；`L=0.2/0.5/1/3 µs` → 每个重装边界 1 次、幅度 999 µs |

### 8.10 第二轮结论

- **确认修复到位**：P0-1（时基公式/换算/掉电边界，附口径修正）、P0-2（不再关中断，原子性论证成立）、P0-4（ACK 先于清 ADDR，两条入口余量充足）、M1（水平寻址 + 0x21/0x22 窗口算法正确）、M2（PRIMASK 保存恢复）、M3 主症、`OLED_SetCursor`。
- **必须再修两条**：8.8-1（`Size==2` 多读一字节，高）、8.8-2（`DelayUs(≥1001)` 死循环，中）；顺手把 8.1 的时基口径、8.7 的 `I2C_WaitFlag` 错注释、8.4 的栈依赖说明与 `bsp_oled.c:14` 旧注释一起更新。
- 修完这两条后，本轮修复即可判定为**通过**；报告第 5 节的上板复测清单仍然有效，建议追加一条「`My_I2C_RegReadBytes(..., Size==2)` 回归用例（修复前必挂/必错，修复后应返回 0 且数据正确）」。本轮复核同样**未修改 `bsp/` 下任何文件**。
