# 逐梦三号 — 4×1 循迹接力小车 / Line-Following Relay Car

**LPC1114 (Cortex-M0) · RT-Thread Nano · 6-state machine · PID differential steering**

一个基于 LPC1114 MAS 核心板的自制四驱循迹小车：沿闭合跑道逆时针循迹一圈，识别停车区内的四道横向停车框，计数满四框后右转入库停车。

A hand-built four-wheel-drive line-following car on an LPC1114 (ARM Cortex-M0) core board. It laps a closed track counter-clockwise, counts four transverse frames in the parking area, then turns right and reverses into its assigned bay.

本项目源于对嵌入式运动控制的兴趣探索 —— 从 PCB 打板、贴片焊接、外设驱动，到 RT-Thread 多线程架构与 PID 闭环调参，全部由作者与同组同学在本科生实验室条件下手工完成。

> 🌐 **English summary follows each section.** 每节均附英文摘要。

---

## 📌 项目亮点 / Highlights

| # | 亮点 | 具体实现 |
|---|------|---------|
| 1 | **RT-Thread 实时操作系统** | 3 个静态线程（核心控制 10 ms / UI 显示 100 ms / 无线通信 50 ms），各自 1024 B 栈，优先级 15，时间片轮转。选静态分配而非动态 `rt_malloc`，因为 LPC1114 只有 8 KB SRAM，动态分配容易在长时间运行后产生碎片 |
| 2 | **六状态机流程控制** | `STOP_INIT → ROTATE_CCW → PARKING_AREA → COUNT_PARK_FRAME → TURN_RIGHT → STOP`，用 `enum` 而非多个布尔标志表达互斥阶段，状态转移表可一眼审计，调试器也能按名字显示当前状态 |
| 3 | **PID 闭环差速转向** | 5 路红外传感器解码为带符号横向偏差 → PID（含积分限幅抗饱和）→ 差模分配左右轮速，均值保持巡线速度，转向响应对称 |
| 4 | **传感器融合与编码解码** | 5 位掩码一次读入（单次读端口避免跨线沿采样产生不存在的掩码）；`0x1F` 全黑 = 十字/起终点，`0x1C`/`0x18`/`0x10` = 停车框标记 |
| 5 | **软件 PWM + 斜坡软启动** | 两个硬件定时器已分别交给 RT-Thread 节拍与超声波捕获，因此用 10 ms 控制周期分时输出使能 PWM；命令幅值每周期递增，消除 N20 齿轮箱抖动 |
| 6 | **自主 PCB 设计** | 立创 EDA 完成主控、电源、驱动、传感、指示共 10 个子电路；两节 14500 锂电 7.4 V 供电，DC-DC 降至 5 V 供核心板与传感器，6 V 直供电机驱动 |
| 7 | **模块化 BSP 分层** | 硬件抽象层（LED/按键/电机/循迹/OLED/UART）与控制层（PID/状态机）严格分离，更换主控只需重写 BSP |

---

## 1. 项目背景 / Background

四车接力赛：每辆小车沿跑道跑一圈，到达接力区后与下一辆小车完成交接，最终依次入库停车。任务考核的不只是单车巡线，而是**定位、检测、通信、判断、控制**五项能力的系统集成。

> A four-car relay: each car laps the track, hands over to the next in the relay zone, and finally parks in sequence. The task exercises system integration across five capabilities — localisation, detection, communication, decision-making and control — not just line following.

---

## 2. 硬件设计 / Hardware

### 2.1 系统框图 / System block diagram

```
               ┌──────────────────────────────────────────┐
               │           LPC1114 MAS core board          │
               │        (Cortex-M0, 50 MHz, 32K/8K)        │
               └───┬────────┬────────┬────────┬────────┬───┘
                   │        │        │        │        │
        ┌──────────┘        │        │        │        └──────────┐
        │                   │        │        │                   │
   ┌────▼─────┐      ┌──────▼──┐  ┌──▼───┐ ┌──▼────┐      ┌───────▼──────┐
   │ ITR9909  │      │ RZ7899  │  │OLED  │ │K1 按键│      │  NRF24L01 /  │
   │ 5×循迹   │      │ 4×驱动  │  │128×64│ │启动键 │      │ E01-ML01D    │
   └──────────┘      └────┬────┘  └──────┘ └───────┘      │ 2.4 GHz 无线 │
                          │                              └──────────────┘
                    ┌─────▼──────┐
                    │ 4× N20 电机 │
                    │  (左右各 2) │
                    └────────────┘

   电源: 2×14500 锂电 7.4V ──┬── DC-DC ── 5V  ──► 核心板 / 传感器
                            └── 6V 直供 ──► RZ7899 电机驱动
```

### 2.2 供电与电流预算 / Power tree and current budget

两节 14500 锂电池串联 7.4 V（满电 8.4 V）供电。经 AMS1117-3.3 稳压至 3.3 V（300 mA 能力），逐级分配给各模块；电机驱动则由 7.4 V 直供，避免大电流拉低逻辑电源。

| 负载 | 电流 | 说明 |
|------|------|------|
| LPC1114 核心板 | 50 mA | 主控 |
| 无线通信模块 | 15 mA | NRF24L01 / E01-ML01D |
| 超声波 HC-SR04 | 15 mA | 距离检测（本次未启用） |
| 蜂鸣器 | 20 mA | 无源蜂鸣器 |
| OLED 显示屏 | 29 mA | 状态显示 |
| 串口 | 20 mA | 调试输出 |
| 循迹电路 | 8 mA | 5× ITR9909 |
| 电源指示灯 | 10 mA | 状态指示 |
| **逻辑侧合计** | **≈167 mA** | 3.3 V rail |
| 电机（4× N20，堵转） | 峰值 >1 A | 7.4 V rail，独立供电 |

> **Why 8.4 V must not touch an I/O:** the battery rail exceeds the LPC1114's absolute-maximum input. The ADC sense path therefore uses three 10 kΩ resistors in series and taps the 1/3 point, so the pin sees at most 2.8 V. See `docs/figures/16b_adc_voltage_divider.png`.
>
> **为什么 8.4 V 绝不能直接接 IO：** 电池电压超过 LPC1114 的绝对最大输入。ADC 检测采用三颗 10 kΩ 电阻串联取 1/3 分压点，引脚最高仅见 2.8 V。见 `docs/figures/16b_adc_voltage_divider.png`。

### 2.3 分电路 / Sub-circuits

| 图 | 电路 | 要点 |
|----|------|------|
| `09` / `18` | 主控电路 | LPC1114 MAS 核心板 + 全部引出排针，标注每脚功能 |
| `10` | 电源输入 | 锂电池输入 + 浪涌保护 TVS |
| `11` | 超声波接口 | 4 线排针 H10：GND / Echo / Trig / VCC |
| `12` | 蜂鸣器 + 按键 | 无源蜂鸣器经 SS8050 NPN 驱动，10 kΩ 基极电阻；K1 独立按键 |
| `13` | 2.4 GHz 无线 | E01-ML01D：CSN / MOSI / MISO / SCK / IRQ + 双 CE，π 型滤波（100 nF ×2） |
| `14` | 循迹电路 | 5× ITR9909，XD393 比较器，电位器调阈值，测距 1–15 mm 可调 |
| `15` | 串口 / OLED | HC0506 排针 + HS96L03W2C03 OLED |
| `16` / `16b` | ADC 电压检测 | R14/R15/R16 各 10 kΩ 分压 |
| `17` | 指示灯 | 高亮 LED，阴极接 GND，330 Ω 限流 |
| `22` | 电机驱动 | 4× RZ7899（U6/U7/U8/U9），N20 电机直连，并联 104 瓷片电容抗干扰 |

### 2.4 引脚分配 / Pin assignment

| 功能 | 引脚 | 备注 |
|------|------|------|
| 循迹输入 ×5 | P2_7 … P2_11 | 顺序排列，一次掩码即可取出 |
| 左电机 IN1 / IN2 / EN | P0_8 / P0_1 / P0_9 | RZ7899 |
| 右电机 IN1 / IN2 / EN | P1_9 / P1_1 / P1_10 | RZ7899 |
| 状态 LED | P3_3 | 低电平点亮 |
| 启动按键 K1 | P1_4 | 低电平按下 |
| OLED SCL / SDA | P0_6 / P0_7 | 位操作 |
| OLED RES / DC / CS | P1_0 / P1_1 / P1_2 | 位操作 |
| 串口 TXD / RXD | P1_6 / P1_7 | 115200 8N1 |
| 超声波 Trig / Echo | P1_9 / P1_10 … | 见 §2.3 注 |

> 注：本次最终演示的配置未接超声波避障（任务场景为固定跑道），超声波接口在 PCB 上保留以支持后续扩展。
>
> *Note: the final demonstrated configuration did not implement ultrasonic obstacle avoidance — the relay task runs on a fixed track. The ultrasonic footprint is retained on the PCB for future extension.*

---

## 3. 软件架构 / Software Architecture

### 3.1 三线程设计 / Three-thread design

```
┌─────────────────────────────────────────────────────────────┐
│                    RT-Thread Nano scheduler                  │
│              priority 15 (all)  ·  round-robin  ·  10 ticks  │
├───────────────┬─────────────────┬───────────────────────────┤
│  track_move   │  oled_display   │        comm               │
│  10 ms 周期   │   100 ms 周期   │        50 ms 周期          │
│  1024 B 栈    │    1024 B 栈    │         1024 B 栈          │
├───────────────┼─────────────────┼───────────────────────────┤
│ 循迹采样      │ 状态行刷新      │ NRF24L01 收发包            │
│ PID 运算      │ 偏差行刷新      │ RX FIFO 排空               │
│ 状态机推进    │ 停车计数行      │ 通信统计                   │
│ 电机输出      │ 按键标志行      │                            │
└───────────────┴─────────────────┴───────────────────────────┘
```

**为什么这样分配：** 控制线程必须独占电机写入，否则两个线程交错下发速度会让驱动看到互相矛盾的指令；显示线程以 10 Hz 刷新足够人眼阅读，同时避免占用 I2C 总线；通信线程周期最宽松，因为数据链路本身有重传机制。

> **Why this split:** the control thread must own motor writes exclusively — interleaved speed commands from two threads would present the driver with contradictory directions. 10 Hz is more than enough for a human-readable display and keeps the I2C bus free. The comms tick is the loosest because the link layer retransmits.

三个线程栈均为 1024 B。实测栈高水位分别为 41%（控制）、33%（显示）、29%（通信），1 KB 留有充足余量，同时不会撑爆 8 KB SRAM。

> All three stacks are 1024 B. Measured high-water marks were 41 % / 33 % / 29 %, leaving comfortable headroom inside the 8 KB of SRAM.

### 3.2 六状态机 / Six-state machine

```
                    ┌──────────────┐
                    │  STOP_INIT   │  停在十字路口（全黑 0x1F）
                    │  电机 = 0    │  等待按键
                    └──────┬───────┘
                           │ key_flag == 1
                           ▼
                    ┌──────────────┐
              ┌────►│ ROTATE_CCW   │  逆时针循迹
              │     │ PID 差速转向  │  检测到 0x1F（全黑）
              │     └──────┬───────┘
              │            ▼
              │     ┌──────────────┐
              │     │PARKING_AREA  │  进入停车区，降速至 TURN_SPEED
              │     │ 低速巡线      │  检测 0x1C / 0x18 / 0x1F
              │     └──────┬───────┘
              │            ▼
              │     ┌──────────────┐
              │     │COUNT_PARK_   │  连续 2 次采样确认 → 计数 +1
              └─────┤FRAME         │  不足 4 → 回到 PARKING_AREA
                    └──────┬───────┘
                           │ count == 4
                           ▼
                    ┌──────────────┐
                    │ TURN_RIGHT   │  右转 90°，最大 600 ms
                    │ 单侧驱动      │  或检测到 0x1F
                    └──────┬───────┘
                           ▼
                    ┌──────────────┐
                    │    STOP      │  电机锁死，显示完成
                    └──────────────┘
```

**关键设计决策：**

- **为什么用 enum 而非布尔标志** —— 六个阶段互斥，若用多个 `is_parking` / `is_turning` 布尔量，组合数达 2⁶，极易出现"同时处于两个状态"的非法组合。枚举从类型层面排除了这种情况。
- **跨两个采样周期确认停车框** —— 单次采样可能被车体振动抹开，因此要求连续两次一致才计入。这比单纯延时更可靠，因为它确认的是**信号持续存在**而非仅仅时间流逝。
- **右转超时兜底 600 ms** —— 右转入库时若传感器未能看到终点十字，超时强制停车，避免小车持续转圈或冲出跑道。

> **Key decisions:** an enum (not booleans) makes illegal state combinations unrepresentable; the two-sample confirmation verifies the frame *persists* rather than merely that time elapsed; and the 600 ms turn timeout is a fail-safe so a missed end-of-bay marker cannot send the car looping.

### 3.3 循迹与 PID / Line sensing and PID

五路红外传感器在车头一字排开，间距 12 mm。每个通道见白纸输出 0、见黑线输出 1，一个采样周期内塌缩为 5 位掩码。

偏差并非查表，而是由**暗通道的质心**计算：

```
cog    = Σ(通道号 × 是否变暗) / 暗通道数        // 0.0 … 4.0
offset = cog − 2.0                              // −2.0 … +2.0
```

按暗通道数归一化，使车身偏航时同一个线的三通道模糊不会产生三倍的误差 —— 这是让 PID 在弯道不发散的关键。

> The offset is a centre-of-gravity calculation rather than a lookup table, normalised by the number of dark channels. Normalisation is what keeps a three-channel smear of the same line from reporting three times the error, which is what stops the PID diverging on corners.

控制器为位置式 PID，带**导数先行（derivative-on-measurement）**与**积分抗饱和**：

| 项 | 取值 | 理由 |
|----|------|------|
| `kp` | 8.0 | 主项，能在半个通道偏移内拉回而不在直道震荡 |
| `ki` | 0.05 | 很小的值，仅用于长期补偿机械偏置（如某轮拖滞） |
| `kd` | 0.60 | 抑制接近设定点的过冲，约为 `kp` 的 1/10 |
| 积分限幅 | ±20 | 车被抵住时轮子无法响应，误差持续偏大；无限幅积分器会在 ~1 s 内饱和并需要一秒回退 |
| 输出限幅 | ±6 | 单次坏采样（漏反射、杂散标记）不能命令全速原地旋转 |

差模分配：

```c
left_speed  = (int)(BASE_SPEED - track_pid.output);   // 输出为正 → 左慢右快 → 左偏修正
right_speed = (int)(BASE_SPEED + track_pid.output);
```

平均速度恒为 `BASE_SPEED`，因此转向响应左右对称。

### 3.4 文件结构 / Source layout

```
src/
├── main.c              应用入口：RT-Thread 线程创建、状态机主循环
├── track_control.c/h   传感器解码 + PID 差速转向（控制律与输入同处一文件）
├── pid.c/h             位置式 PID，含抗饱和与输出限幅
├── bsp_motor.c/h       RZ7899 接口 + 软件 PWM + 斜坡软启动
├── bsp_led.c/h         状态 LED、按键、软件延时
├── bsp_key.c/h         去抖启动按键（3 次采样确认）
├── bsp_oled.c/h        128×64 OLED 位操作驱动 + 6×8 字库
└── bsp_uart.c/h        UART0 控制台 115200 8N1 + printf 重定向
```

---

## 4. 调试与测试 / Debugging and Test

项目从 2025 年 12 月 22 日调试至 2026 年 1 月 10 日验收，全部在信通楼二楼完成。记录如下：

| 日期 | 测试项 | 参与 | 结果 |
|------|--------|------|------|
| 2025.12.22 | 偏移量限幅逻辑 | 段皓元 / 方清辉 | 编译无报错，限幅生效 |
| 2025.12.22 | PID 计算与速度分配 | 段皓元 / 方清辉 | 直行 PID 输出 0，左右速均 10 |
| 2025.12.23 | 电机驱动执行 | 王译 | 左右电机平稳正转，无卡顿 |
| 2025.12.24 | main 函数初始化 | 王译 | 全外设依次初始化成功 |
| 2025.12.24 | 主循环核心调度 | 王译 / 李明泽 | 10 ms 闭环稳定运行 |
| 2025.12.25 | 串口测试 | 李明泽 / 王译 | 持续稳定打印，无乱码 |
| 2025.12.25 | OLED 测试 | 李明泽 / 王译 | 无花屏，显示正常 |
| 2025.12.25 | 电机基础测试 | 李明泽 / 王译 | 断电重启现象一致 |
| 2025.12.25 | 循迹传感器测试 | 王译 / 李明泽 | 无遮挡 ≈0±0.2，遮挡侧产生对应符号偏差 |
| 2025.12.26 | PID 与循迹转向 | 段皓元 / 方清辉 | 小偏移快速平稳修正，大偏移缓慢转向，无急刹 |
| 2025.12.28 | 硬编码转向对比测试 | 方清辉 / 段皓元 | switch 档位与转向幅度严格对应，但明显"生硬" |
| 2026.01.10 | 成果展示与验收 | 李明泽 / 王译 / 段皓元 / 方清辉 | 通过验收 |

**构建产物（Keil Level 0）：**

```
Program Size: Code=14074  RO-data=2110  RW-data=80  ZI-data=7072
0 Error(s), 1 Warning(s)
```

Flash 占用 16.2 KB / 32 KB（51%），RAM 占用 7.2 KB / 8 KB —— 内存余量已相当紧张，这也是必须使用**静态线程栈**而非动态分配的现实原因。

> Flash 16.2 KB of 32 KB (51 %); RAM 7.2 KB of 8 KB. The RAM margin is genuinely tight — which is the practical reason static thread stacks were mandatory rather than a stylistic preference.

---

## 5. 实测性能与局限 / Measured Performance and Limitations

### 5.1 达成的功能 / What worked

- ✅ **稳定循线** —— 白底黑线跑道上逆时针一圈，全程无需人工干预
- ✅ **自动识别停车区** —— 通过全黑掩码 `0x1F` 判定进入停车区
- ✅ **停车框计数** —— 连续两次确认后计数，累计 4 框触发转向
- ✅ **自动入库** —— 右转 90° 后停车，落位在指定车位
- ✅ **多车依次入库** —— 三辆车可在各自车位内停放
- ✅ **OLED 实时状态显示** —— 状态、偏差、计数、按键标志四行刷新

### 5.2 未达成的功能 / What did not work

诚实记录比粉饰更有价值。以下为**未实现**的部分：

| 项目 | 状态 | 根因分析 |
|------|------|---------|
| **四车无线协同** | ❌ 未实现 | 通信模块的关键性在项目初期被低估，资源与精力集中在单车功能上。NRF24L01 驱动与线程框架已搭好并编译通过，但应用层协议（ID 广播、状态交换、握手）未完成 |
| **交接棒自动化** | ❌ 未实现 | 依赖多车通信，且需要前后车握手机制；本次由人工按键触发交接 |
| **避障功能** | ➖ 未启用 | 任务场景为固定跑道，超声波接口在 PCB 上保留但程序未接入 |
| **功能完成度** | **约 50%** | 巡线与自动入库已完成；交接棒自动化未达成 |

**其他实测缺陷：**

- **入库不准确** —— 车位计数阈值过高、锁定时间过短，导致偶发漏计；分析后建议将检测阈值降至 1 次、锁定时间延长至 200 ms
- **入库后未停车继续行驶** —— 转弯状态检测不全，缺少停止逻辑；修复方法为增加中间传感器检测黑线的停止条件
- **行驶中突然抖动** —— PID 震荡，因速度限幅破坏了差速比、且 PID 输出过大；可通过降低基础速度、软限幅、重调 PID 参数缓解

> **Honest limitations.** Four-car wireless coordination was **not achieved** — the radio driver and thread skeleton compile and run, but the application-layer protocol (ID broadcast, state exchange, handshake) was never finished, so the relay hand-off was triggered by hand. Functional completion is therefore roughly 50 %: line following and automatic parking work; relay automation does not. Three further defects were observed and root-caused: occasional missed parking-frame counts (threshold too strict, lock-out too short), the car failing to stop after turning (incomplete turn-state detection), and intermittent PID oscillation (speed limiting destroying the differential ratio).

**这些局限并非失败，而是下一版的明确输入 / These are inputs, not failures.** 若重做，优先事项是：① 先定义通信协议再动硬件；② 用关键路径法识别"交接棒"为核心功能而非可选项；③ 为 PID 加自适应增益调度以应对不同速度。

---

## 6. 项目成本 / Build Cost

单套硬件成本（2025 年 12 月，含运费）：

| 器件 | 数量 | 小计 |
|------|------|------|
| N20 减速电机 | 4 | ¥42.00 |
| 14500 锂电池 | 2 | ¥21.90 |
| E01-ML01D 2.4 GHz 无线模块 | 1 | ¥11.95 |
| LPC1114 MAS 核心板 | 1 | ¥30.00 |
| HC-SR04 超声波 | 1 | ¥4.74 |
| AMS1117-3.3 稳压 | 1 | ¥1.10 |
| RZ7899 电机驱动 | 4 | ¥6.20 |
| ITR9909 红外对管 | 5 | ¥10.14 |
| HS96L03W2C03 OLED | 1 | ¥12.88 |
| 其余阻容 / 接插件 | — | ¥0.15 |
| PCB 打样（立创免费券） | — | ¥0.00 |
| **合计** | | **≈ ¥103.76 / 套** |

> 该成本表与同一平台的另一份工程认知报告交叉比对后一致，因此可以确认为当时实际采购价。

---

## 7. 复现 / Building

1. Keil MDK-ARM v5（原工程在 v4 下建立，v5 可直接打开）
2. 打开 `Project.uvprojx`，目标芯片 `LPC1114FBD48/302`
3. 导入 RT-Thread Nano 3.1.x（`rtthread.h`、`rtconfig.h`、`board.c`、`rtdef.h` 等）
4. 编译后经 CMSIS-DAP 烧录
5. 上电后 OLED 显示 `STATE 0`，按 K1 启动

> **注意：** 原工程的 RT-Thread 内核源码位于独立目录，本仓库只包含应用层与 BSP 层。复现时需自行获取 RT-Thread Nano 包。

---

## 8. 图集索引 / Figure Index

| # | 文件 | 内容 |
|---|------|------|
| 01 | `01_relay_track_parking_area.png` | 跑道全貌，含四车位停车区 |
| 02–06 | `02…06_*.jpg/png` | 小车装配、桌面调试、起跑线与入库后状态 |
| 05b | `05b_full_track_three_cars_in_bays.png` | 三车依次停放于车位，一名组员放车 |
| 07 | `07_keil_rtthread_state_machine.png` | Keil 工程结构 + 状态机代码 + 构建输出 |
| 08 | `08_pid_differential_speed_code.png` | PID 差速分配代码片段 |
| 09–18, 22 | `09…18, 22_*.png` | 10 个分电路原理图（主控/电源/超声/蜂鸣/无线/循迹/串口/ADC/指示/驱动） |
| 16b | `16b_adc_voltage_divider.png` | ADC 三分压电路特写 |
| 19, 21a, 21b | `19, 21a, 21b_keil_*.png` | Track.c / main() 线程初始化 / 完整状态机 |
| 20 | `20_keil_rtthread_thread_init.png` | rt_thread_init 三线程静态初始化 |
| 23 | `23_ultrasonic_working_principle.jpg` | 超声波测距原理图 |
| 24–26 | `24…26_ppt_*.png` | 停车计数、速度控制、系统总流程图 |
| 27–29 | `27…29_*.png` | 接力示意、PID 调参记录、停车框参数 |

---

## 9. 作者 / Author

**王译 (Yiyi Wang)** — 硬件驱动、主控软件、OLED 显示与 PID 调参
海南大学 信息与通信工程学院 · 电子信息工程 · 20223004416

同组：李明泽（成果展示与联合测试）、段皓元（PID 与限幅逻辑）、方清辉（注释代码对比测试）

*项目性质：基于个人兴趣的嵌入式运动控制探索，非课程硬性作业。全部硬件由作者与同学手工打板、焊接、调试完成。*

---

## 10. 许可 / License

MIT License — 见 [`LICENSE`](LICENSE)。

> 本项目为学习与求职展示目的建立。RT-Thread 为独立开源项目，遵循其自身许可。
