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

**最强的一条约束**：`math/` 层不允许 `#include` 任何 HAL / device / middleware / protocol / app 头文件。
这条规则由 `fw/math/BUILD` 的 `deps` 显式表达 —— 违反时编译失败，不依赖人工评审。
实测验证：`math/` 下 21 个文件（17 头 + 4 个 host 测试）的非标准库 include 全部指向 `math/` 内部，零外泄；测试文件同样只依赖 `math/`。

---

## 2. 分层模型

```
┌──────────────────────────────────────────────────────────────┐
│ L5  上位机 (Python + Web)                                    │
│     app.js ── HTTP ── web_server.py ── xt_proto.py           │
└──────────────────────────┬───────────────────────────────────┘
                           │ CAN-FD 1M/2M
┌──────────────────────────┴───────────────────────────────────┐
│ L4  通信边界  middleware/communication                         │
│     CanCommandAdapter · TelemetryPublisher · BinaryLink       │
│     protocol/xt_can 只定义稳定 wire ABI                        │
├──────────────────────────────────────────────────────────────┤
│ L3  业务编排  app/Application                                │
│     初始化顺序 · 顶层 FSM · 主循环 · 故障/超时/Flash 安全策略  │
│     只依赖 application_ports 与业务服务接口                    │
├──────────────────────────────────────────────────────────────┤
│ L2  业务服务  middleware/                                    │
│     control · encoder · calibration · snapshot                │
│     config · persistence                                     │
├──────────────────────────┬───────────────────────────────────┤
│ L1a 算法层 math/(纯算法) │ L1b 器件层 device/                 │
│     foc · servo · cal    │     DRV8353S · MA600 · motor       │
├──────────────────────────┴───────────────────────────────────┤
│ L0  基础设施  core/ · ports/ · HAL/                           │
│     静态内存/格式化 · ISpiBus/IAngleSensor · PWM/ADC/CAN      │
└──────────────────────────────────────────────────────────────┘

board/xtellar_stm32g4/FirmwareComposition 是唯一组合根：
具体 STM32G4、MA600、CAN、算法和服务对象只在这里构造并注入。
```

### 依赖规则矩阵

| 层 | 允许依赖 | 禁止依赖 | 强制手段 |
|---|---|---|---|
| `math/` | 仅 `math/base` | HAL、device、middleware、protocol、app | BUILD `deps` 白名单 |
| `ports/` | 标准整数类型 | HAL、device、app | 独立 header-only target |
| `device/` | `ports/`、必要的 HAL/math base | app、middleware | BUILD `visibility` |
| `middleware/` | core、protocol、ports、HAL、device、math、稳定 app ports | 具体 board 组合 | 分服务 BUILD targets |
| `application_core` | application ports、业务服务 | HAL、device、raw protocol、board | 独立 Bazel target |
| `board/` | 全部具体实现 | — | 只由 `firmware_composition` 汇合 |
| `core/` / `protocol/` | 标准库整数/容器 | HAL、device、middleware、app | host/bare-metal 双平台 target |

`Application` 不再构造器件、不解析 CAN、不组遥测、不运行 15 kHz 控制律；
它只表达“按什么顺序初始化、当前在哪个状态、每轮调哪些服务”。MA600 通过
`IAngleSensor` 注入 `EncoderService`，更换编码器只替换 device/board 组合，不改业务编排。

### 代码体量分布

| 模块 | 行数 | 文件数 | 性质 |
|---|---:|---:|---|
| `fw/app` | **598** | **8** | 顶层业务编排 + 纯安全策略/host 测试；`application.cc` 186 行 |
| `fw/board/xtellar_stm32g4` | **1002** | **7** | 具体对象图、板级策略、平台/配置实现 |
| `fw/middleware` | **3929** | **16** | 控制、编码器、标定、快照、通信、配置与持久化 |
| `fw/ports` | **52** | **2** | SPI/延时/角度传感器抽象 |
| `fw/math` | 4252 | 21 | 纯算法（含 4 个 host 测试文件）|
| `fw/HAL` | 1866 | 10 | 外设封装 |
| `fw/core` | 179 | 2 | 静态内存池与无状态文本格式化 |
| `fw/protocol` | 491 | 1 | 稳定 CAN wire ABI |

统计口径为对应目录中的 `.h/.cc/.cpp`；不含 BUILD、链接脚本和汇编。
重构把原 `app` 中的器件构造、788 行实时控制、708 行协议解析和 395 行遥测组包
分别迁到 board/middleware；原 `pool`、`telemetry`、`nvs` 三个混合包又按职责收敛到 `core`、`protocol` 与 `middleware/{communication,config,persistence}`。APP 本身现在只保留生命周期与状态机。

---

## 3. 执行模型

单核裸机，两条执行流，无 RTOS、无线程、无动态分配。

### 3.1 控制 ISR（TIM5 更新中断，NVIC 优先级 2）

ISR 的唯一入口是 `MotorControlService::StepIsr()`，`Application` 不进入实时路径。

```
15 kHz 周期开始
  │
  ├─ 读取统一会话 Mode（Stopped / Calibration / Servo / Current / Mit）
  ├─ 无活跃会话且无快照 → 关闭 ISR 并返回
  ├─ 计算实际 dt（us 定时器，钳位 20~500 µs）
  ├─ PhaseCurrentAdc.ReadLatest() + EncoderService.UpdatePwmIsr()
  ├─ 峰值过流检查（连续 5 拍 → MotorControlService::Stop）
  ├─ 当前会话生成 Id/Iq 或 D/Q 电压请求
  │    Servo / MIT / Current / 六类 Calibration
  ├─ FocController 或 DqModulator → SVPWM
  └─ PhasePwm.SetDuty() + SnapshotService.PushIsr()
```

`MotorControlService` 独占控制会话、PWM ISR、FOC/Servo/MIT 和标定实时步进的所有权；
协议适配器只能请求 `Start*`/`Stop`，不能直接拼接实时状态。

**ISR 内的硬性禁忌**：不做 Flash 擦写、不做 CAN 阻塞发送、不做堆分配。
Flash 写入统一延后到主循环的 `CalibrationManager::PersistPending()`。

### 3.2 顶层主循环 `Application::Run()`

```
while (true)
  ├─ FeedWatchdog()
  └─ switch (state)
       ├─ INIT
       │    └─ GateDriver → CurrentSense → Offset → PWM → Sync ADC
       │       → Encoder → RuntimeConfig
       ├─ INIT_FAILED（锁存安全停机；仅接受 bootloader 命令）
       ├─ RUN
       │    ├─ DriverFaulted() → DRIVER_FAULT
       │    ├─ 标定待写 Flash → StopMotor → PersistPending
       │    ├─ commands.Poll()          命令入口/协议适配
       │    ├─ EnforceCommandTimeout()  500 ms 无本节点命令必停机
       │    └─ telemetry.Poll()         快照/遥测出口
       ├─ DRIVER_FAULT（仅实际门驱故障允许 TryRecoverDriverFault）
       └─ ENTER_BOOTLOADER（先停机/断门驱）
```

初始化失败和运行期门驱故障是两个不同状态：前者不会用“门驱已恢复”绕过未完成的
ADC/PWM/配置初始化；后者才允许门驱恢复后回到 `RUN`。命令超时只由发给本节点且
通过 `BinaryLink` 头校验的帧续期，其他 CAN 节点流量和快照发送都不能屏蔽停机。

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
所有对象经 `core::memory::PoolPtr<T>` 从池中分配。没有 `malloc` 就没有碎片、
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

`fw/protocol/inc/protocol/xt_can.h` 是唯一的线格式定义，包含 **19 处 `static_assert(sizeof(...))`**：

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
滤波则把转速卡在 80 rad/s。现在 `board/xtellar_stm32g4/board_config.h` 用表达式把两者锁死。

**编码器 PLL**：`filtered_` 保持单圈用于换相，`position_` 单独积分输出多圈连续值
供位置环和 MIT 使用。角度尖刺由 `spike_error_rad` 阈值剔除。

**齿槽前馈**：1024 点 int8 表，按转子位置注入 q 轴电流补偿，在速度环和 MIT 模式都生效。

**Kt 与 Ke 的换算**：采用幅值不变的 2/3 Park 变换，因此 `Kt = 1.5 × Ke_dq`；
厂商给的是线电压峰值/krpm，需除以 `√3` 转为相/dq 峰值。这两个换算集中在
`board/xtellar_stm32g4/board_config.h` 的 `BemfVPerMechRadS` / `VendorTorqueConstantNmPerA`，避免散落。

---

## 7. 参数与标定体系

### 7.1 参数优先级（当前设计）

**Config 的 Motor 表是 R / L / Ke 的唯一真源。**

```
board/.../board_config.h 板级默认
       ↓
Runtime Flash (XCFG) 存在？
       ├─ 是 → 以 Flash 为准
       └─ 否 → 用标定 Flash 的 R/L/Ke 预填 Motor 表
       ↓
RuntimeConfigService::Apply()
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

| 场景 | 主要触点 | APP 是否修改 |
|---|---|---|
| 新控制模式 | `math/` 加算法 → `MotorControlService` 注册 Mode/Start/Step → `CanCommandAdapter` 解码 → `TelemetryPublisher` 映射 → 主机协议/GUI | **否** |
| 新标定项 | `math/calibration` → `CalibrationManager` 持久化 → `MotorControlService` 会话 → 通信适配 | **否** |
| 新配置参数 | `RuntimeConfig`/wire ABI → `RuntimeConfigService::FillDefaults/Apply` → 主机协议 | **否** |
| 新遥测字段 | `xt_can.h` → `TelemetryPublisher` → `xt_proto.py`/GUI | **否** |
| 换编码器芯片 | 实现 `IAngleSensor`，在 `FirmwareComposition` 替换构造 | **否** |
| 换 MCU/板卡 | 实现 HAL/platform，新增 `board/<target>/FirmwareComposition` | **否** |
| 换电机 | 运行时 Config Motor 表；或 board 默认 preset | **否** |

扩展仍可能触碰控制、协议、主机多个端点，但不再改顶层 `Application`；
业务生命周期与具体器件、wire ABI、实时控制实现解耦。

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
       ├─ FirmwareComposition 构造平台/器件/算法/服务对象图
       └─ Application::Run()
            └─ Initialize(): 栅驱 → ADC → PWM → 编码器 → 配置/标定
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

| 编号 | 问题 | 严重度 | 复评状态 |
|---|---|---|---|
| S-1 | 看门狗（主循环喂狗 + HardFault 关 PWM）；Init 期预算见 N-7 | P0 | 部分解决 |
| S-2 | 峰值过流已做；缺 I²t / 温度 / 母线 ADC；采样失效见 N-6 | P0 | 部分解决 |
| S-3 | Flash 写入前先 `StopOutput()` | P0 | 已解决 |
| B-1 | 绝对路径已清零；新克隆仍失败见 N-2/N-3/N-5 | P1 | 部分解决 |
| T-1 | 6 个 host test targets / 38 用例；应用生命周期/超时与命令帧合法性已有回归；控制/标定服务仍缺 host 覆盖 | P1 | 部分解决 |
| E-1 | 控制扩展已迁入 `MotorControlService`，APP 不再因新增模式修改；`StepIsr` 仍约 270 行 | P1 | 部分解决 |
| C-1 | `xt_can.h` ↔ `xt_proto.py` 双份手工维护 | P1 | 未解决 |
| R-1 | 单位转换魔数散落 **68** 处（`* 1000.0f` / `* 0.001f`）| P2 | 恶化 |
| E-2 | MIT 复用 `id_mA`/`iq_mA` 承载 Kp/Kd（注释已标明，降为位浪费）| P2 | 未解决 |
| M-1 | 死代码主清单及 N-8 编译数据库/注释尾巴均已清理 | P2 | 已解决 |
| N-1 | ISR 已迁出 APP 并收敛到 `MotorControlService::StepIsr`；内部仍需按会话继续拆分 | P1 | 部分解决 |
| N-2 | `local_override` 静默丢弃 patches，声明的 mbedos patch 未生效 | P1 | 未解决 |
| N-3 | `third_party/rules_mbed` 与 `bazel_toolchain` 未入 git | P1 | 未解决 |
| N-7 | IWDG 在 Init 前启动，Init 忙等路径可能吃掉 ~500 ms 预算 `[INFERENCE]` | P0 | 未解决 |
