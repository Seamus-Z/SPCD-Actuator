# 架构设计说明

> 本文是 `xtellar_mbed` 的设计深入说明。快速上手请看根目录 [README.md](../README.md)，
> 代码质量评估请看 [CODE_QUALITY_REVIEW.md](CODE_QUALITY_REVIEW.md)。

---

## 1. 设计目标与约束

| 目标 | 具体要求 | 对架构的影响 |
|---|---|---|
| 硬实时电流环 | 15 kHz，抖动 < 5 µs | 无 RTOS、无堆、ISR 内不做阻塞操作 |
| 算法可验证 | 控制律能脱离硬件推理 | `math/` 强制零 IO 依赖 |
| 上下位机协同 | 线格式不能双份漂移 | `xt_can.h` 单一真源 + `static_assert` 锁 ABI |
| 现场可维护 | 不拆机升级、不重编译改参数 | 独立 bootloader + Flash 运行时配置 |
| 语义对齐 moteus | 便于迁移与对照 | 命令模型沿用 `kPosition` / `kCurrent` |

**最强的一条约束**：`math/` 层不允许 `#include` 任何 HAL / device / telemetry / nvs 头文件。
这条规则由 `fw/math/BUILD` 的 `deps` 显式表达 —— 违反时编译失败，不依赖人工评审。
实测验证：`math/` 下 17 个头文件的非标准库 include 全部指向 `math/` 内部，零外泄。

---

## 2. 分层模型

```
┌─────────────────────────────────────────────────────────┐
│ L5  上位机 (Python + Web)                                │
│     app.js ── HTTP ── web_server.py ── xt_proto.py       │
└──────────────────────────┬──────────────────────────────┘
                           │ CAN-FD 1M/2M
┌──────────────────────────┴──────────────────────────────┐
│ L4  链路层  telemetry/binary_link + xt_can (wire ABI)    │
├─────────────────────────────────────────────────────────┤
│ L3  编排层  app/Application                              │
│     主循环 · 控制 ISR · 命令分发 · 模式状态机 · 遥测组包  │
├─────────────────────────────────────────────────────────┤
│ L2  服务层  middleware/                                  │
│     EncoderService · CalibrationManager                  │
├──────────────────────────┬──────────────────────────────┤
│ L1a 算法层 math/(纯函数) │ L1b 器件层 device/            │
│     foc · servo_mode     │     DRV8353S · MA600 · motor  │
│     calibration          │                               │
├──────────────────────────┴──────────────────────────────┤
│ L0  平台层  HAL/ · nvs/ · pool/                          │
│     FDCAN · PhasePwm · CurrentAdc · SPI · Flash · 内存池 │
└─────────────────────────────────────────────────────────┘
```

### 依赖规则矩阵

| 层 | 允许依赖 | 禁止依赖 | 强制手段 |
|---|---|---|---|
| `math/` | 仅 `math/base` | HAL、device、telemetry、nvs、app | BUILD `deps` 白名单 |
| `device/` | HAL、math/base | app、middleware | BUILD `visibility` |
| `middleware/` | HAL、device、math、nvs | app | 约定 + BUILD |
| `app/` | 以上全部 | — | — |
| `telemetry/` | HAL | app、math | BUILD |

`middleware/` 是唯一被明确授权可以同时依赖「器件」和「算法」的层，
承担把物理传感器读数翻译成算法输入的职责（例如 `EncoderService` 把 MA600 原始计数
经补偿、PLL 滤波后变成连续机械角）。

### 代码体量分布

| 模块 | 行数 | 文件数 | 性质 |
|---|---|---|---|
| `fw/app` | 3694 | 9 | 编排层（含 1715 行的 `application.cc`）|
| `fw/math` | 3640 | 17 | 纯算法，全 header-only |
| `fw/HAL` | 1862 | 10 | 外设封装 |
| `fw/bootloader` | 1493 | 12 | 独立镜像 |
| `fw/telemetry` | 1326 | 7 | 协议 + 链路 |
| `fw/device` | 881 | 4 | 器件驱动 |
| `fw/nvs` | 623 | 2 | Flash 持久化 |
| `fw/middleware` | 585 | 2 | 业务服务 |
| `fw/pool` | 89 | 1 | 静态内存池 |
| `fw/protocol` | 18 | 2 | 兼容 shim（已无引用）|

`math/` 与 `app/` 体量相当（3640 vs 3694），说明算法复杂度确实被抽离出来了；
但 `app/` 仅 9 个文件承载 3694 行，单文件平均 410 行，集中度偏高。

---

## 3. 执行模型

单核裸机，两条执行流，无 RTOS、无线程、无动态分配。

### 3.1 控制 ISR（TIM5 更新中断，NVIC 优先级 2）

```
15 kHz 周期开始
  │
  ├─ 计算实际 dt（用 us 定时器实测，不信任标称周期）
  │
  ├─ 判定当前活跃模式（8 个互斥布尔量）
  │    cal_on / bemf_on / r_on / l_on / cogging_on / mit_on / vel_on / current_on
  │
  ├─ 无任何模式活跃 → 关闭 ISR 并返回（防止僵尸 ISR）
  │
  ├─ 采样：PhaseCurrentAdc.ReadLatest() + EncoderService.Sample()
  │
  ├─ 外环（按模式二选一）
  │    ServoMode.Step()  轨迹 → PID → 力矩 → Iq
  │    MitMode.Step()    Kp·Δp + Kd·Δv + Tff → Iq
  │    标定算法 .Step()  各自的激励生成
  │
  ├─ 内环：FocController.Step()
  │    Clarke → Park → PI(d) / PI(q) → 反 Park → SVPWM
  │
  └─ 输出：PhasePwm.SetDuty() + 快照缓冲 PushIsr()
```

**ISR 内的硬性禁忌**：不做 Flash 擦写、不做 CAN 阻塞发送、不做除法密集运算。
Flash 写入统一延后到主循环的 `CalibrationManager::PersistPending()`。

**dt 实测而非标称**：`period_s()` 的标称值与真实 UEV 间隔可能不一致，
直接用标称值会让 PLL 的 ω 缩放错误，进而让速度环误以为已经跟上目标。
代码用 us 定时器实测并把 dt 钳位在 20~500 µs，防止 IRQ 合并造成的异常值。

### 3.2 主循环 `RunOnce()`

```
while(true)
  ├─ 读 DRV8353S 状态 → 有故障则 StopOutput + 进入 DRIVER_FAULT
  ├─ CalibrationManager::PersistPending()   ← Flash 写入唯一出口
  ├─ PollCan()          解析命令 → HandleCommand 分发
  ├─ MaybeCommandTimeout()  500 ms 无命令则停机
  ├─ MaybeSendTelemetry()   组包回传
  └─ MaybeSendSnapshot()    快照分帧搬运
```

### 3.3 时序契约

| 事件 | 周期 / 时限 | 违约后果 |
|---|---|---|
| 控制 ISR | 66.7 µs | 超时则 PWM 更新滞后，电流环失稳 |
| 上位机命令流 | 20 ms（50 Hz） | 超 500 ms 板端自动停机 |
| Flash 页擦除 | 数十 ms | 期间控制 ISR 已被主动关闭 |
| CAN 帧 | 64 B @ 2 Mbps ≈ 40 µs | — |

---

## 4. 内存模型

### 4.1 RAM

```
0x20000000 ┬ 512 B   bootloader ↔ app 复位保持握手区
0x20000200 ┼         .data / .bss
           │         └─ g_pool_storage: SizedPool<40960>  (40 KB 静态池)
           │              └─ Application（含 1024 点齿槽表、标定累加器）
           ┆
0x2001FFFF ┴ 8 KB    栈（STACK_SIZE = 0x2000）
```

**无堆**：`main.cpp` 用 placement new 在静态缓冲区上构造内存池，
所有对象经 `pool::PoolPtr<T>` 从池中分配。没有 `malloc` 就没有碎片、
没有分配失败的运行期分支、没有不确定延迟 —— 这是硬实时系统的正确取舍。

### 4.2 Flash

```
0x08000000 ┬ 48 KB  中断向量表（reset → BootloaderEntry @ 0x0800C001）
0x0800C000 ┼ 16 KB  Bootloader（独立镜像，无 C 运行时）
0x08010000 ┼ 440 KB Application                    ← APP_FLASH
0x0807E000 ┼  2 KB  Runtime Config  magic 'XCFG'   ← page 124
0x0807E800 ┼  2 KB  预留空隙
0x0807F000 ┴  4 KB  编码器/电机标定 magic 'XENC'   ← page 126-127
```

> **为什么 APP 是 440 KB 而不是 448 KB**：NVS 两页必须落在应用镜像之外。
> 否则用户在 GUI 点一次 Config Save，Flash 擦除会打掉正在执行的代码。
> 这个约束写在 `stm32g474_app.ld` 的注释里，属于把事故固化成设计。

两块 NVS 物理分离的理由：保存 PID 参数时不应该重写几万字节的齿槽/补偿表 blob，
擦写寿命和写入耗时都不在一个量级。

---

## 5. 通信协议

### 5.1 分层

```
CAN-FD 帧
  └─ Header{ magic, version, type, seq }   4 B
      └─ payload（按 type 分派，小端紧凑）
```

标准 11 位 ID，`node_id` 默认 1：

- Host → Board: `0x100 + node_id`
- Board → Host: `0x180 + node_id`

### 5.2 单一真源与 ABI 锁

`fw/telemetry/inc/telemetry/xt_can.h` 是唯一的线格式定义，包含 **19 处 `static_assert(sizeof(...))`**：

```cpp
static_assert(sizeof(CtrlReply) == 64, "CtrlReply size");
static_assert(sizeof(ServoRequest) == 48, "ServoRequest size");
static_assert(sizeof(MotorConf) == 32, "MotorConf size");
```

字段顺序或类型改错，编译期就失败，不会等到现场丢帧才发现。
这是本项目在接口治理上做得最好的一点。

### 5.3 命令模型：单入口多模式

`kCmdServo`（ID=6）是所有实时控制的统一入口，48 B `ServoRequest` 的 `control` 字段选模式：

| `control` | 模式 | 语义 |
|---|---|---|
| 0 | Position | 轨迹 → PID → 力矩 → Iq（moteus `kPosition`）|
| 1 | Current | 直接给定 Id/Iq，跳过外环 |
| 2 | MIT | `T = Kp·(P_cmd−P_fdb) + Kd·(V_cmd−V_fdb) + T_ff` |

**向后兼容**：`HandleServo` 同时接受三种 payload 长度 —— 8 B（遗留纯速度）、
20 B（遗留 pos/vel/id/iq）、≥48 B（完整）。老上位机不会被新固件打死。

### 5.4 位置语义差异

| 模式 | 位置表示 | 理由 |
|---|---|---|
| Position | 单圈 `[0, 2π)`，固件映射最短路径 | GUI 输入直观，避免用户手算圈数 |
| MIT | 多圈连续（PLL 解缠绕值） | 阻抗控制需要连续误差，折返会导致力矩突跳 |

同一个 `position_mrad` 字段在两种模式下语义不同，这是刻意的设计取舍，
已在 `xt_can.h` 注释中标明。

---

## 6. 控制算法栈

```
                位置命令 / 速度命令 / 力矩命令
                            │
        ┌───────────────────┼───────────────────┐
        ▼                   ▼                   ▼
   ServoMode            MitMode             (Current 直通)
   轨迹生成器            阻抗律
   位置/速度 PID         Kp·Δp+Kd·Δv+Tff
        │                   │                   │
        └───────────────────┼───────────────────┘
                            ▼
                      力矩 → Iq  (Iq = T / Kt)
                            │
                            ▼
                    FocController（电流环）
                 PI(d) · PI(q) · 反电动势前馈
                 交叉耦合前馈 · 相位超前补偿
                            │
                            ▼
                   DqModulator → SVPWM → 占空比
```

### 关键设计点

**相位超前补偿**：`phase_lead_s = 1.5 / f_pwm + filter_us`。
这个值必须与 MA600 的滤波时间常数绑定，两者一旦解耦就会出问题 ——
历史上 64 µs 超前配 1024 µs 滤波导致 60 rad/s 飞车，1124 µs 超前配 1024 µs
滤波则把转速卡在 80 rad/s。现在 `board_config.h` 用表达式把两者锁死。

**编码器 PLL**：`filtered_` 保持单圈用于换相，`position_` 单独积分输出多圈连续值
供位置环和 MIT 使用。角度尖刺由 `spike_error_rad` 阈值剔除。

**齿槽前馈**：1024 点 int8 表，按转子位置注入 q 轴电流补偿，在速度环和 MIT 模式都生效。

**Kt 与 Ke 的换算**：采用幅值不变的 2/3 Park 变换，因此 `Kt = 1.5 × Ke_dq`；
厂商给的是线电压峰值/krpm，需除以 `√3` 转为相/dq 峰值。这两个换算集中在
`board_config.h` 的 `BemfVPerMechRadS` / `VendorTorqueConstantNmPerA`，避免散落。

---

## 7. 参数与标定体系

### 7.1 参数优先级（当前设计）

**Config 的 Motor 表是 R / L / Ke 的唯一真源。**

```
board_config.h 板级默认
       ↓
Runtime Flash (XCFG) 存在？
       ├─ 是 → 以 Flash 为准
       └─ 否 → 用标定 Flash 的 R/L/Ke 预填 Motor 表
       ↓
ApplyRuntimeConfig()
       ↓ 下发
FocController / ServoMode / MitMode / EncoderPLL / MA600
       ↓
加载标定 Flash 的编码器表（offset / 齿槽 / 几何补偿）
       └─ 只加载几何类数据，不覆盖电气参数
```

标定完成后，R / L / Ke 会**回写 Motor 表（RAM）并立即生效**；
是否持久化由用户在 Config 页点 Save 决定。

> 这个设计解决了早期的一个困惑：标定出的 R/L 生效了，但 Config 页显示的还是旧值 ——
> 因为当时是两套数各走各的。现在统一到一张表。

### 7.2 标定流程依赖图

```
R 相电阻 (无需编码器)
   └─→ Ld/Lq 电感 (需要 R 做 τ=L/R 换算)
编码器转圈映射
   ├─→ 编码器几何补偿 (需要已标定的 offset)
   ├─→ 齿槽补偿      (需要准确角度)
   └─→ Ke 反电动势   (需要闭环速度能跑起来)
```

---

## 8. 扩展点索引

新增功能时应该改哪里：

| 场景 | 主要触点 | 实测成本 |
|---|---|---|
| 新控制模式 | `xt_can.h` 加枚举 → `math/servo_mode/` 加算法 → `application.h` 加 `Start*` → `application.cc` 加 ISR 分支 + 遥测 + StopOutput + ApplyRuntimeConfig → `binary_commands.cc` 加解析 → `xt_proto.py` 加 pack → GUI 三件套 | **11 文件 / +334 行**（MIT 实测）|
| 新标定项 | `math/calibration/` 加算法 → `calibration_manager.h` 加 Apply/Persist → `xt_can.h` 加 `kCalSub*` → `application.cc` 加 ISR 分支 + CalTelem → GUI | ~8 文件 |
| 新配置参数 | `runtime_config_store.h` 加字段（注意 `static_assert` 尺寸）→ `xt_can.h` 同步 → `FillDefault` + `ApplyRuntimeConfig` → `xt_proto.py` → GUI | ~5 文件 |
| 新遥测通道 | `xt_can.h` 加字段 → `BuildCtrlReply` → `xt_proto.py` 解包 → `app.js` 通道表 | ~4 文件 |
| 换电机 | 只改 Config 的 Motor 表（无需重编译）；或改 `device/motor.h` 的默认 preset | 0 文件 |

> 「新控制模式」需要触碰 11 个文件，这是当前架构最明显的扩展成本，
> 详见 [CODE_QUALITY_REVIEW.md](CODE_QUALITY_REVIEW.md) 的 **E-1** 条目。

---

## 9. 启动时序

```
上电 / 复位
  │
  ├─ 向量表 @ 0x08000000 → BootloaderEntry @ 0x0800C001
  │
  ├─ Bootloader
  │    ├─ 检查复位保持区握手标志
  │    ├─ 有升级请求 → 进入 CAN 接收循环，写 Flash
  │    └─ 无 → 跳转 App @ 0x08010000
  │
  └─ App: AppReset()
       ├─ 清 BSS / 拷贝 .data（无 CRT，手写）
       ├─ SCB->VTOR = 0x08010000
       ├─ 使能 FPU（CP10/CP11）← 必须在任何浮点代码之前
       ├─ SetupSystemClock()  HSI → PLL 170 MHz
       ├─ __enable_irq()
       ├─ placement new 构造静态内存池
       └─ Application::Run()
            └─ Init(): 栅驱 → ADC → PWM → 编码器 → 配置加载 → 标定加载
```

> FPU 使能顺序是个坑：`PhaseCurrentAdc` 构造函数里有浮点运算，
> 如果在 `SCB->CPACR` 之前构造就会 HardFault，且现象是「CAN 完全无响应」，
> 极难定位。注释已在 `main.cpp` 标注。

---

## 10. 上位机架构

```
浏览器
  │ fetch / 轮询
  ▼
web_server.py  (ThreadingHTTPServer, 无框架依赖)
  ├─ CanBridge          持有 python-can 总线
  │    ├─ _stream_loop  50 Hz 后台线程，持续下发控制帧
  │    ├─ _rx_loop      接收 → 按 type 分发到各队列
  │    └─ msglog        环形缓冲，供 GUI 控制台
  ├─ /api/cmd           控制命令（切换 stream op）
  ├─ /api/telem         Live 数据轮询
  ├─ /api/cal/*         标定流程
  ├─ /api/conf          运行时配置 CRUD
  └─ /api/snap/*        快照采集与分析
       └─ snap_analysis.py  FFT / 幅值 / 残差 / 极对数反推
```

**流式控制模型**：GUI 点「Start servo」不是发一次命令，而是设置 `stream_op`，
由后台线程以 50 Hz 持续重发。这样板端的 500 ms 超时保护才有意义 ——
浏览器关闭或网络断开，电机自动停。

**零框架依赖**：只用 Python 标准库 + `python-can`，前端无构建步骤（原生 JS）。
代价是 `app.js` 已 1812 行、`web_server.py` 1872 行，接近单文件可维护上限。

---

## 11. 已知架构债

以下问题已识别，详细分析、量化指标与改进建议见 [CODE_QUALITY_REVIEW.md](CODE_QUALITY_REVIEW.md)：

| 编号 | 问题 | 严重度 |
|---|---|---|
| S-1 | 无看门狗，ISR 卡死则 PWM 冻结在最后占空比 | P0 |
| S-2 | 无软件过流/过温保护，仅依赖 DRV 硬件 VDS 阈值 | P0 |
| S-3 | Flash 写入期间电机处于无控窗口 | P0 |
| B-1 | `WORKSPACE` 硬编码绝对路径，换机器无法构建 | P1 |
| T-1 | 固件零单元测试，`math/` 的可测试性红利未兑现 | P1 |
| E-1 | `ControlIsrStep` 425 行 / 8 布尔量分派，新增模式需改 11 文件 | P1 |
| C-1 | `xt_can.h` ↔ `xt_proto.py` 双份手工维护 | P1 |
| R-1 | 单位转换魔数散落 64 处（`* 1000.0f` / `* 0.001f`）| P2 |
| E-2 | MIT 模式复用 `id_mA`/`iq_mA` 承载 Kp/Kd，语义误导 | P2 |
| M-1 | 死代码：`app_telemetry`、`telemetry:telemetry`、`protocol/`、`fw/stm32g474.ld` | P2 |
