# 迷你平衡车（STM32F103C8T6）

> **许可证：MIT，但带范围限定。**
> 根目录 [`LICENSE`](LICENSE) 只覆盖本项目作者编写的部分
> （`user/` `test/` `bsp/` `my_lib/` `tools/` `docs/`）。
> 仓库中的 STMicroelectronics 代码（`std_periph_driver/`、`startup/`、
> `user/system_stm32f10x.*`、`user/stm32f10x_it.*`、`user/stm32f10x_conf.h`）
> **不适用 MIT**，仍按 MCD-ST Liberty SW License V2 使用，
> 见 [`LICENSE-ST-SPL`](LICENSE-ST-SPL)。
> 细节与原因：[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)

两轮自平衡小车的嵌入式固件。基于 STM32F103C8T6 + STM32F10x 标准外设库（SPL）
开发，裸机周期任务调度，无 RTOS。底层驱动为**本项目自研**（`bsp/`，依据公开
数据手册独立实现）；硬件方案来自江协科技。

---

## 功能

- **姿态解算** — MPU6050 六轴数据互补滤波递推得到 roll / pitch / yaw，单位度。
  上电时自动校准陀螺零偏（约 1 秒，期间板子必须静止）。
- **平衡控制** — 三层串级结构：

  ```
  速度环 (ẋ → θ_ref) → 角度环 (θ → θ̇_ref) → 角速度环 (θ̇ → θ̈)
        ↓ θ̈ 逆解算得 ẍ，积分后除以轮半径 R_w 得目标轮速
     电机调速环（编码器测速 + PID）
  ```

  车身线速度由编码器和陀螺共同重建：`ẋ = R_w·ω轮 + l_p·θ̇`
- **电机调速** — TB6612FNG 驱动，20 kHz PWM（`PSC=0`、`ARR=3599` @ 72 MHz），
  占空比分辨率 3600 档。
- **按键启停** — K1 单击启停；启动前要求 `|pitch| < 10°`，避免躺着按键直接冲出去。
- **蓝牙遥控** — USART2 接蓝牙模块，收发走中断 + 环形缓冲区（TX 256 B / RX 128 B），
  不阻塞控制环。
- **OLED 显示** — 软件 I2C 驱动 4 针 OLED，支持画点 / 线 / 圆 / 矩形 / 位图 / 中文字体。
- **安全保护** — 倾角超限自动停机、传感器读数失效自动停机，停机原因可查询：

  | 原因码 | 含义 |
  |---|---|
  | `BAL_STOP_NONE` | 未停机 / 人为停机（按键、初始化） |
  | `BAL_STOP_TILT` | 车身倾角超过 `BAL_MAX_ANGLE`，判定为倾倒 |
  | `BAL_STOP_MPU` | 传感器数据失效（I2C 连续读失败） |

---

## 硬件

基于**江协科技「平衡车控制板 V2.0」**方案，控制板购自淘宝。
原理图著作权归江协科技所有，**本仓库不包含其原图**。

### 引脚分配

依据本项目代码整理（`user/app_pwm.c`、`user/app_encoder.c`、
`user/app_bluetooth.c`、`user/app_oled.c`、`user/app_mpu6050.c` 等）：

| 功能 | 引脚 | 备注 |
|---|---|---|
| 左轮 PWM（PWMA） | PA0 / TIM2_CH1 | 复用推挽 |
| 右轮 PWM（PWMB） | PA1 / TIM2_CH2 | 复用推挽 |
| 左轮方向 AIN1 / AIN2 | PB12 / PB13 | 推挽输出 |
| 右轮方向 BIN1 / BIN2 | PB14 / PB15 | 推挽输出 |
| 左编码器 A 相 / B 相 | PA6 / PA7 | 外部中断 |
| 右编码器 A 相 / B 相 | PB6 / PB7 | 外部中断 |
| MPU6050（SCL / SDA） | PB10 / PB11 | 硬件 I2C2 |
| OLED（SCL / SDA） | PB8 / PB9 | 软件 I2C |
| 蓝牙模块 TX / RX | PA2 / PA3 | USART2（挂在 APB1） |
| 调试串口 TX / RX | PA9 / PA10 | USART1 @ 115200 |
| 按键 K1 | PB1 | 启停 |
| 电池检测 | PB2 | 预留引脚，ADC 功能当前已停用 |

> 板载其他按键（K2–K4）、NRF24L01 接口等当前代码未使用。

---

## 目录结构

| 目录 / 文件 | 内容 | 来源 |
|---|---|---|
| `user/` | 应用层：`main.c`、`app_balance`、`app_attitude`、`app_motor`、`app_pwm`、`app_encoder`、`app_mpu6050`、`app_bluetooth`、`app_usart1`、`app_oled`、`adc_bat` | 本项目作者 |
| `test/` | 分模块台架测试：`pwm_test`、`encoder_test`、`mpu6050_test`、`motor_test`、`balance_test`、`bat_test` | 本项目作者 |
| `bsp/` | 板级驱动：`bsp_delay`（SysTick+DWT 计时）、`bsp_usart`（阻塞串口）、`bsp_i2c`（硬件 I2C）、`bsp_si2c`（软件 I2C）、`bsp_oled`+`bsp_font6x8`（SSD1306 与自研点阵字模） | **本项目作者（自研，见 `bsp/README.md`）** |
| `my_lib/task.*`、`pid.*`、`button.*` | 周期任务调度器、PID、按键驱动 | 本项目作者（AI 辅助） |
| `tools/` | 命令行验证脚本与字模生成脚本（不参与固件构建） | 本项目作者 |
| `docs/` | 设计文档与重构方案 | 本项目作者 |
| `std_periph_driver/`、`startup/` | STM32F10x 标准外设库 V3.6.2 + CMSIS + 启动文件 | STMicroelectronics |
| `user/system_stm32f10x.*`、`stm32f10x_it.*`、`stm32f10x_conf.h` | 系统时钟、中断向量、库配置 | STMicroelectronics |
| `template.uvprojx` / `template.uvoptx` | Keil MDK 工程文件 | 本项目 |

> **底层驱动已全部自研。** 重构前 `my_lib/` 下的驱动代码来自第三方（B 站「铁头山羊」，
> 未附带开源许可证）。现已依据公开数据手册与官方参考手册重写为 `bsp/` 下的独立实现，
> 接口名保持不变，因此 `user/` 与 `test/` 无需改动。
> 依据文档清单与验证方式见 [`bsp/README.md`](bsp/README.md) 与
> [`docs/驱动重构方案.md`](docs/驱动重构方案.md)。

### 任务调度

`my_lib/task.c` 是一个极简的裸机周期任务调度器（最多 `TASK_MAX_NUM = 8` 个任务），
在 `main()` 的 `while(1)` 里轮询。

**注册顺序必须与调用顺序一致**：MPU6050 → 姿态 → 平衡 → 电机。
各控制环任务周期 5 ms；注意控制环内不能有阻塞式串口打印
（`My_USART_Printf` 阻塞约 2 ms），需要打印时单独开一个 ≥ 50 ms 的任务。

---

## 编译与运行

**环境**

- Keil MDK-ARM 5.x
- 器件支持包：STM32F1 系列（STM32F103C8）
- 目标芯片：STM32F103C8T6（LQFP48，64 KB Flash / 20 KB RAM）

**步骤**

1. 用 Keil 打开 `template.uvprojx`；
2. 直接 Build（所有源码与启动文件已随仓库提供，无需额外配置）；
3. 通过 ST-Link 下载，或使用 `Objects/template.hex`（该文件不入库，需自行编译生成）。

**无 Keil 时的命令行验证**（不依赖 UV4，用 armcc/armlink 直接编译链接）：

```powershell
pwsh -File tools\compile.ps1   # 逐文件编译，应全通过
pwsh -File tools\link.ps1      # 链接出 build\template.axf / .hex，并打印体积
```

**注意**

- 工程编译产物（`Objects/`、`Listings/`、`build/`）已加入 `.gitignore`；
- 上电后 1 秒内为陀螺零偏校准期，**板子必须保持静止**，否则姿态角会带固定偏差；
- 首次上地面前需标定机械中位 `BAL_CENTER_ANGLE`
  （见 `user/改正方案.md` 第 4 步）。

---

## 开发进度

详细的改进路线与调试顺序记录在 [`user/改正方案.md`](user/改正方案.md)，主要待办：

- [x] 底层驱动自研化重构（`my_lib/` → `bsp/`），解除第三方代码版权依赖
- [x] I2C 读写加超时并检查返回值，避免总线异常时死等
- [ ] 转向环（轮速差纠偏 + 陀螺阻尼）
- [ ] 独立看门狗（IWDG）+ 堵转保护 + `omega_ref` 限幅
- [ ] 蓝牙失联保护（500 ms 未收到指令自动减速停车）
- [ ] 电池电压检测（需先决定硬件方案）

---

## 第三方组件声明

本项目包含第三方代码与资料，**使用前请阅读 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)**。摘要：

| 来源 | 内容 | 许可状态 |
|---|---|---|
| STMicroelectronics | 标准外设库 V3.6.2、CMSIS、启动文件、`system_stm32f10x.*`、`stm32f10x_it.*` | MCD-ST Liberty SW License V2；**禁止**置于 MIT/BSD/Apache/GPL 之下 |
| 江协科技 | 平衡车控制板硬件方案 | 原理图版权归其所有，本仓库不包含原图 |
| ~~铁头山羊（B 站）~~ | ~~`my_lib/` 下大部分底层驱动~~ | **已在本次重构中全部重写为 `bsp/` 下的自研实现，不再是第三方代码** |

第三方许可原文随仓库提供：[`LICENSES/`](LICENSES/) 与 `std_periph_driver/LICENSE.txt`。

---

## 许可证

本仓库采用**带范围限定的 MIT 许可证**：

| 部分 | 许可证 |
|---|---|
| `user/` `test/` `bsp/` `my_lib/` `tools/` `docs/`（本项目作者编写） | **MIT** — [`LICENSE`](LICENSE) |
| `std_periph_driver/`、`startup/`、`user/system_stm32f10x.*`、`user/stm32f10x_it.*`、`user/stm32f10x_conf.h`（STMicroelectronics） | **MCD-ST Liberty SW License V2** — [`LICENSE-ST-SPL`](LICENSE-ST-SPL) |

### 为什么 MIT 必须带范围限定

**不能**在根目录放一个声明覆盖「全部内容」的 MIT 许可证。

**ST 的许可（SLA0044 Rev5）第 5 条禁止**以任何方式使**标准外设库本身**受制于
开源条款，并点名了 GPL、EPL、Apache、BSD、MIT。关键在「this software」指的是
SPL 本身，所以：

- 根目录放 MIT 并声明覆盖整个仓库 → ❌ 让 SPL 受制于 MIT，违反第 5 条
- **根目录放 MIT，但正文明确限定只覆盖自有代码** → ✅ 本仓库的做法
- 完全不加许可证 → ✅ 可行，但别人不知道能用什么

早先本项目认为「仓库里有一部分代码的著作权不属于作者」，**那条理由已随本次重构
消除**（第三方驱动已全部重写为自研实现）；上面这条 ST 的约束是**唯一仍然成立**的
限制，且不因重构而消失。

如果将来想整仓统一用 MIT，需要先去掉 ST 的 SPL —— 改用 **STM32CubeF1 HAL/LL**
（BSD-3-Clause）或完全直接操作寄存器。

各部分完整的权利状态与说明：
- [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)
- [`LICENSES/`](LICENSES/) — 第三方许可原文
- `std_periph_driver/LICENSE.txt` — ST 标准外设库许可
