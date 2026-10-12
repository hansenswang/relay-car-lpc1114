# 测试记录与结果分析 / Test Records and Results

本文记录实际测得的运行数据、状态机各阶段的验证方法，以及三个已定位缺陷的根因分析。

This document records the measured behaviour, the verification method for each state-machine phase, and the root-cause analysis of three located defects.

---

## 1. 什么叫"跑通" / What "working" means here

本项目的验收标准是**单车完整任务链**，而非全系统（四车协同）：

> The acceptance criterion is the **single-car task chain**, not the full four-car system:

1. 上电后小车静止在起跑十字，OLED 显示 `STATE 0`
2. 按下 K1，小车进入逆时针循迹
3. 沿白底黑线稳定行驶一整圈，无脱线
4. 检测到停车区入口（全黑掩码）后降速
5. 依次识别四道横向停车框并准确计数
6. 计数满 4 后右转 90°
7. 入库后停车，OLED 显示完成状态

第 1–7 步全部可达。**未包含**多车通信与自动交接。

---

## 2. 状态机逐阶段验证 / Phase-by-phase verification

| 阶段 | 掩码条件 | 验证方法 | 观察结果 |
|------|---------|---------|---------|
| `STOP_INIT` | — | 上电观察 | 电机静止，OLED 显示 `STATE 0`，等待按键 |
| 按键触发 | `key_flag == 1` | 按 K1 | 约 300 ms 去抖后状态跳转，无误触发 |
| `ROTATE_CCW` | 非全黑 | 串口打印偏差 | 直道偏差 ≈ 0 ± 0.2；遮挡左侧为负、右侧为正 |
| 进入停车区 | `0x1F` | 串口打印掩码 | 通过十字时打印 `0x1F`，状态切换到停车区 |
| `PARKING_AREA` | `0x1C`/`0x18`/`0x1F` | 串口打印 | 车速降至 `TURN_SPEED`，可观察到明显减速 |
| `COUNT_PARK_FRAME` | 连续 2 次 | 串口打印计数 | 每通过一道框计数值 +1 |
| `TURN_RIGHT` | 超时 600 ms 或 `0x1F` | 目视 + 秒表 | 右转角度约 90°，落位在车位内 |
| `STOP` | — | 观察 | 电机停转，OLED 显示 `STATE 5` |

---

## 3. 循迹偏差实测 / Measured offset values

在 2025.12.25 的传感器测试中，用手遮挡不同通道并记录串口打印的偏差：

| 遮挡情况 | 掩码（示意） | 偏差 | 说明 |
|---------|------------|------|------|
| 无遮挡（直行） | 中心通道变暗 | ≈ 0 ± 0.2 | 线在正中 |
| 遮挡左侧 1 路 | 左 1 变暗 | −1.0 | 车偏右，需左修正 |
| 遮挡左侧 2 路 | 左 2 变暗 | −2.0 | 明显右偏 |
| 遮挡右侧 1 路 | 右 1 变暗 | +1.0 | 车偏左 |
| 遮挡右侧 2 路 | 右 2 变暗 | +2.0 | 明显左偏 |
| 全部遮挡 | `0x1F` | 保持上次值 | 十字/停车框，由状态机处理 |
| 全部无遮挡 | `0x00` | 保持上次值 | 丢线，PID 无方向信息 |

**为什么全黑与全白都要"保持上次值"：** 这两种情况都不含方向信息，若强行置零会命令小车直行 —— 在十字路口这会直接冲出跑道。保持上次偏差让 PID 继续朝原方向修正，把决策权交给状态机。

> **Why both all-black and all-white hold the previous value:** neither carries directional information. Forcing zero would command "go straight", which on a cross means driving off the track. Holding the last offset lets the PID keep correcting while the state machine makes the decision.

---

## 4. PID 参数验证 / PID verification

在 `track_move_control()` 中加入串口打印，然后手动让小车偏移，观察输出（2025.12.26，段皓元 / 方清辉）：

| 工况 | 偏差 | PID 输出 | 左速 | 右速 | 判定 |
|------|------|---------|------|------|------|
| 直行 | 0 | 0 | 10 | 10 | ✅ 积分项已清零 |
| 右偏 | +2 | > 0 | < 10 | > 10 | ✅ 左慢右快，向左修正 |
| 左偏 | −2 | < 0 | > 10 | < 10 | ✅ 左快右慢，向右修正 |

差速分配的均值恒为 `BASE_SPEED = 10`，因此转向不会改变巡航速度 —— 这是左右对称的基础。

---

## 5. 三种控制律对比 / Control law comparison

| | PID 差模 | switch 硬编码六档 | 限幅版 PID |
|--|---------|-----------------|-----------|
| 直道稳定性 | ✅ 平滑无摆动 | ⚠️ 明显阶梯 | ✅ 平滑 |
| 小偏移（±1） | ✅ 快速修正无过冲 | ⚠️ 可能过冲（档位跳变） | ✅ 平稳 |
| 大偏移（±3~4） | ✅ 缓慢转向回归 | ⚠️ 突然大转向 | ⚠️ 限幅后转向偏软 |
| 调参成本 | 高（每次改速度需重调） | 无 | 中 |
| 结论 | **最终采用** | 保留为对照 | 破坏差速比，弃用 |

**为什么最终选 PID 而非六档：** 六档方案实现简单、行为可预测，但档位之间的跳变在车身上表现为可见的"顿挫"，且在直道上无法稳定停在某一档 —— 因为实际的偏差是连续变化的，量化后会在两档之间反复跳动。PID 的连续输出从根本上消除了这个问题。

**为什么限幅版被弃用：** 对 `left_speed` / `right_speed` 各自独立限幅会破坏差速比 —— 当一侧被限幅而另一侧没有时，两侧速度差被压缩，转向能力随之下降。正确的做法是对 PID 的**输出**限幅（已在 `pid.c` 中实现），而不是对分配后的速度限幅。

> **Why the clamped variant was dropped:** clamping `left_speed` and `right_speed` independently destroys the differential ratio. When one side clamps and the other does not, the effective steering authority shrinks. The correct place to limit is the PID *output* (as implemented in `pid.c`), not the post-split speeds.

---

## 6. 已定位缺陷的根因分析 / Root causes of located defects

### 6.1 车位计数偶发漏计

**现象：** 通过四道停车框时偶尔只计到 3 道，导致小车不转向而继续行驶。

**分析：** 检测条件过于严格（要求掩码严格等于 `0x1C` 或 `0x18`），且锁定时间过短。车体振动使某一帧的掩码落在非预期值上，该帧即被丢弃。

**建议修复：** 将检测阈值放宽至接受 `0x10` 及以上任意暗通道组合；锁定时间由单帧延长至 200 ms。

### 6.2 入库后未停车

**现象：** 右转完成后小车继续行驶，未按预期停住。

**分析：** 转弯状态的结束条件依赖检测到全黑掩码，但入库末端若传感器恰好跨在框线上，掩码可能不是全黑，导致状态无法退出。

**建议修复：** 增加中间传感器单独检测黑线的停止条件，作为与全黑判定并列的第二出口。

### 6.3 行驶中突然抖动

**现象：** 直道行驶时偶发短促抖动，速度越高越明显。

**分析：** PID 震荡。两个成因叠加：一是速度限幅破坏了左右差速比（见 §5），二是 PID 输出幅值相对基础速度偏大，在 `BASE_SPEED = 10` 时输出 ±6 已接近满偏。

**建议修复：** 降低基础速度、改用 PID 输出软限幅、并降低 `kp`。更理想的做法是按速度调度增益（gain scheduling），使转弯时增益自动提高。

---

## 7. 内存与实时性 / Memory and timing

**构建产物（Keil Level 0）：**

```
Program Size: Code=14074  RO-data=2110  RW-data=80  ZI-data=7072
Total RO  Size (Code + RO-data) : 16184 bytes  (15.8 KB)
Total RW  Size (RW-data + ZI-data):  7152 bytes  (7.0 KB)
```

| 资源 | 占用 | 容量 | 占比 |
|------|------|------|------|
| Flash | 16.2 KB | 32 KB | 51% |
| RAM | 7.2 KB（含 3×1024 B 线程栈） | 8 KB | 90% |

**RAM 分析：** 三个线程栈共占 3072 B，占 RAM 的近一半。剩余的约 4 KB 用于 RT-Thread 内核对象、全局变量、以及线程运行时的局部栈帧。90% 的占用率意味着**不可能再增加第四个线程**，也无法承担动态内存分配带来的碎片风险 —— 这是本项目坚持静态栈分配的直接原因。

> RAM headroom is 90 % used, nearly half of it three thread stacks. There is no room for a fourth thread, and none for the fragmentation risk that dynamic allocation brings. That is the concrete reason the design uses static stacks.

**实时性：** 控制线程按 10 ms 固定节拍唤醒。采样锚点在每次循环后向前推进一个整周期，而不是"睡 10 ms"，因此即使某次循环执行偏长，采样栅格也不会累积漂移。

> The control thread wakes on a rigid 10 ms grid: the wake anchor advances by exactly one period per iteration rather than the thread sleeping "10 ms", so a long pass does not accumulate drift on the sampling grid.

---

## 8. 验收清单 / Verification checklist

- [x] 编译零错误（1 警告），Flash 与 RAM 占用均在余量内
- [x] 上电后静止在起跑十字，OLED 显示 `STATE 0`
- [x] K1 按键去抖正常，无误触发、无漏触发
- [x] 串口持续稳定输出，无乱码
- [x] OLED 四行内容刷新正确，无花屏
- [x] 电机基础驱动测试：断电重启现象一致
- [x] 循迹偏差符号正确（左负右正）
- [x] PID 直行输出为零，积分项清零
- [x] 完整一圈循迹无脱线
- [x] 停车区识别正确（全黑掩码触发）
- [x] 停车框计数达到 4
- [x] 右转入库，落位在车位内
- [x] 多车依次入库（三车各就其位）
- [ ] 四车无线协同（未实现）
- [ ] 自动交接棒（未实现）

---

## 9. 下一步 / What I would test next

1. **先定义协议再动硬件** —— 通信协议（ID 广播、状态交换、握手时序）应在硬件设计阶段确定，而不是留到最后
2. **关键路径识别** —— 用关键路径法分析任务书，把"交接棒"识别为核心功能而非可选项
3. **增益调度 PID** —— 按速度自动调整增益，兼顾低速稳定性与高速响应
4. **编码器反馈** —— 当前速度是开环命令值，加装编码器后可实现真正的速度闭环
5. **超声波接入** —— PCB 已预留接口，接入后可用于检测前车距离以触发自动交接
