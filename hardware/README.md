# 硬件设计说明 / Hardware Notes

这块板子是"逐梦三号"平台的自制四驱循迹小车控制板。本文记录设计取舍、总线划分、供电架构与布局观察，以及如果重画会改什么。

This board is a self-built four-wheel-drive line-following car controller for the "逐梦三号" platform. These notes record the design trade-offs, bus partitioning, power architecture, layout observations, and what would change on a respin.

---

## 1. 系统分块 / Block overview

```
┌──────────────────────────────────────────────────────────────────────┐
│                      LPC1114 MAS core board (2×13 header)             │
│                                                                       │
│  GPIO2[7:11] ──► 循迹输入 ×5                                          │
│  GPIO0[8,1,9] ──► 左电机 RZ7899 (IN1/IN2/EN)                          │
│  GPIO1[9,1,10] ──► 右电机 RZ7899 (IN1/IN2/EN)                         │
│  GPIO3[3] ──► 状态 LED                                               │
│  GPIO1[4] ──► K1 启动按键（中断或轮询）                                │
│  GPIO0[6,7] ──► OLED SCL / SDA（位操作）                              │
│  GPIO1[0,1,2] ──► OLED RES / DC / CS                                  │
│  GPIO1[6,7] ──► UART0 TXD / RXD（调试串口）                            │
│  GPIO1[x] ──► 蜂鸣器（经 SS8050 驱动）                                 │
│  ADC0 ──► VBAT 1/3 分压                                                │
│  SSP1 ──► NRF24L01 / E01-ML01D（CSN/MOSI/MISO/SCK/IRQ）                │
└──────────────────────────────────────────────────────────────────────┘
```

## 2. 总线划分 / Bus partitioning

| 总线 | 挂载 | 配置 | 说明 |
|------|------|------|------|
| GPIO 直驱 | 循迹 ×5、电机 ×6、LED、按键 | 数字输入/输出 | 时序最简单，无共享冲突 |
| 位操作（bit-bang） | OLED ×5 | 无时钟，软件循环 | OLED 刷新 10 Hz，总线 99.9% 空闲 |
| UART0 | 调试串口 | 115200 8N1 | 仅单向打印调试信息 |
| SSP1 | NRF24L01 | SPI 主机 | 与 OLED 位操作不冲突 |
| ADC0 | VBAT 分压 | 10 bit | 电池电压监测 |

**为什么 OLED 用位操作而不是 SSP：** SSP 外设被 NRF24L01 占用，而两者时钟相位需求不同。复用同一 SSP 意味着每次切换都要重新初始化外设，代价高于位操作几个字节的时间。OLED 刷新率极低，因此位操作是更划算的选择。

> **Why the OLED is bit-banged:** the SSP peripheral is committed to the NRF24L01 and the two need incompatible clock phases. Multiplexing them means re-initialising the peripheral on every switch — more expensive than the few microseconds a bit-banged byte takes. With a 10 Hz refresh the bus is idle 99.9 % of the time anyway.

## 3. 供电架构 / Power tree

```
   锂电池输入
   2×14500 串联 (7.4V 标称 / 8.4V 满电)
        │
        ▼
   浪涌保护 TVS
        │
   ┌────┴─────────────────────────────┐
   │                                  │
   ▼                                  ▼
 AMS1117-3.3                     直供 7.4V
 (300mA 能力)                         │
   │                                  ▼
   ▼                            4× RZ7899
 3.3V 逻辑电源                  + 4× N20 电机
   │
   ├─► LPC1114 核心板      50 mA
   ├─► 无线通信模块         15 mA
   ├─► 超声波               15 mA
   ├─► 蜂鸣器               20 mA
   ├─► OLED                 29 mA
   ├─► 串口                 20 mA
   ├─► 循迹电路              8 mA
   └─► 电源指示灯           10 mA
   ───────────────────────────────
   逻辑侧合计             ≈ 167 mA
```

**关键取舍：电机独立供电。** 四路 N20 同时启动时瞬态电流可以轻易超过 1 A。若与逻辑电源共用同一条 3.3 V 轨，电机启动会在电源上产生瞬间压降，足以让 LPC1114 复位 —— 现象是"每次加速小车就重启"。分成两条轨后问题彻底消失。

> **The critical choice — a separate motor rail.** Four N20 motors starting together can transiently draw over 1 A. Sharing the 3.3 V logic rail would sag it enough to brown out the LPC1114 on every acceleration — the symptom was a car that reset each time it sped up.
>
> **8.4 V 绝不直接接 IO。** 电池电压超过 LPC1114 绝对最大输入，因此所有电池侧检测都必须先分压。本板用三颗 10 kΩ 串联取 1/3 点接 ADC，引脚最高仅见 2.8 V。

## 4. 引脚分配 / Pin assignment

| 网络 | 引脚 | 方向 | 备注 |
|------|------|------|------|
| TRK_1 … TRK_5 | P2_7 … P2_11 | IN | 顺序排列，一次掩码取出全部 5 位 |
| M_L_IN1 / M_L_IN2 / M_L_EN | P0_8 / P0_1 / P0_9 | OUT | 左驱动（两电机并联） |
| M_R_IN1 / M_R_IN2 / M_R_EN | P1_9 / P1_1 / P1_10 | OUT | 右驱动（两电机并联） |
| LED_STATUS | P3_3 | OUT | 低电平点亮，330 Ω 限流 |
| KEY_K1 | P1_4 | IN | 低电平按下 |
| OLED_SCL / OLED_SDA | P0_6 / P0_7 | OUT | 位操作 |
| OLED_RES / OLED_DC / OLED_CS | P1_0 / P1_1 / P1_2 | OUT | 位操作 |
| UART_TXD / UART_RXD | P1_6 / P1_7 | OUT/IN | 115200 8N1 |
| NRF_CSN / SCK / MOSI / MISO | P2_x（SSP1） | — | E01-ML01D |
| VBAT_SENSE | ADC0 | IN | 1/3 分压 |

## 5. 布局观察 / Layout observations

- **循迹探头到主控的走线应尽量短** —— 五路模拟量（比较器后为数字）走线一旦过长，电机电流产生的耦合会在信号上叠加噪声，表现为偏差值随机跳动。实测中把探头线束缩短并远离电机走线后，直道偏差的抖动幅度显著下降。
- **电机并联 104 瓷片电容** —— 每个 N20 电机两端并联 100 nF，吸收换向电刷产生的尖峰。未加电容时可以在示波器上看到明显的换向毛刺。
- **大电流回路与信号回路分开走** —— 电机供电的铜箔与循迹信号线在地平面上避免平行，减少共阻抗耦合。
- **排针连接电机控制脚** —— 每路电机的 IN1/IN2/EN 用排针引出，方便单独测试某一路而不必飞线。

## 6. 打板与焊接 / Fabrication

- PCB 由立创 EDA 设计，打样使用平台免费券，因此 PCB 本身成本为 ¥0
- 贴装元件由作者与同学手工焊接；0402/0603 封装为主
- 四个 RZ7899 为 SOP-8 封装，是焊接中最容易虚焊的器件 —— 首次上电时右前轮不转，重新补焊后正常

## 7. 如果重画 / What I would change

1. **电机驱动布局更紧凑** —— 四个 RZ7899 当前分散布置，使大电流走线长度增加。集中布局可以让电机供电形成更短的环路，降低辐射。
2. **增加串行电阻与 ESD 保护** —— 外接排针（超声、串口）没有保护，实际调试中曾因插拔导致一次 IO 损伤。
3. **循迹探头接口改为连接器而非直焊** —— 直焊的传感器一旦损坏难以更换，本次就遇到过一个 ITR9909 失效需要整线重焊。
4. **预留 I²C 总线** —— 当前所有通信都走 GPIO 或 SSP1，若需要增加传感器将无总线可用。
