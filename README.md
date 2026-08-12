# xtellar_mbed

STM32G474 无刷电机驱动器固件（FOC + 编码器闭环）与配套上位机工具链。

硬件参考 moteus-x1 / family 3 拓扑：DRV8353S 栅极驱动 + 三相电流采样 + MA600 磁编码器 + CAN-FD 通信。
控制语义对齐 [moteus](https://github.com/mjbots/moteus) 的 `kPosition` / `kCurrent`，并额外提供 MIT 阻抗模式。

---

## 1. 硬件目标

| 项目 | 配置 |
|---|---|
| MCU | STM32G474xE（Cortex-M4F，170 MHz，512 KB Flash / 128 KB RAM）|
| 栅极驱动 | DRV8353S（SPI 配置，3xPWM 模式）|
| 电流采样 | 三相低边分流 0.5 mΩ + CSA 20x，ADC1/2/3 与 PWM 同步 |
| 位置传感 | MA600 磁编码器（SPI 6 MHz）|
| PWM | TIM5，15 kHz 中心对齐；mbed us-ticker 让位到 TIM15 |
| 通信 | FDCAN2，1 Mbps 仲裁 / 2 Mbps 数据，BRS 开启 |

板级引脚与电气默认值集中在 `fw/app/inc/board_config.h`。

---

## 2. 仓库结构

```
fw/                     固件（Bazel 构建）
├── app/                 顶层生命周期：初始化、FSM、主循环与安全策略
├── board/xtellar_stm32g4/
│   └── firmware_composition.*  唯一对象组合根与 STM32G4 平台实现
├── core/                与业务无关的静态内存池、文本格式化
├── protocol/            稳定 CAN wire ABI（xt_can.h）
├── ports/               ISpiBus、IAngleSensor 等平台无关端口
├── middleware/          业务服务与适配器
│   ├── control/         15 kHz 控制会话与 ISR
│   ├── encoder/         编码器采样与 PLL
│   ├── calibration/     标定编排
│   ├── snapshot/        PWM 率快照
│   ├── communication/   CAN 命令、遥测与 BinaryLink
│   ├── config/          运行配置数据模型
│   └── persistence/     Flash 配置与标定存储
├── HAL/                 裸机外设封装：FDCAN、PWM、电流 ADC、SPI、定时器、时钟
├── device/              器件驱动：DRV8353S、MA600、电机参数表
├── math/                纯算法层（无 HAL / 无 IO 依赖，全 header-only）
│   ├── foc/             Park/Clarke、PI、电流环、DQ 调制器
│   ├── servo_mode/      编码器 PLL、位置 PID、轨迹生成、MIT 阻抗
│   └── calibration/     编码器相位、R、Ld/Lq、Ke、齿槽辨识算法
└── bootloader/          独立 CAN 引导加载器（16 KB）

tools/
├── bazel                Bazel 版本包装器
└── host/                上位机
    ├── xt_proto.py      协议打包 / 解包（与 xt_can.h 对应）
    ├── test_xt_proto.py 协议单元测试
    ├── gui/             本地 Web 上位机（HTTP + 静态页面）
    ├── snap_analysis.py     快照频域分析
    └── compensate_encoder.py 编码器几何补偿生成

hw/x1/                  KiCad 硬件工程
```

---

## 3. 分层架构

```
        ┌──────────────────────────────────────────────┐
  Host  │  Web GUI (JS)  ──HTTP──  web_server.py       │
        │                             │ xt_proto.py    │
        └─────────────────────────────┼────────────────┘
                                      │ CAN-FD
        ┌─────────────────────────────┼────────────────┐
        │  protocol/xt_can  ←→  middleware/communication│
        ├──────────────────────────────────────────────┤
        │  app/Application：初始化 · FSM · 主循环 · 安全策略│
        ├──────────────────────────────────────────────┤
        │  middleware：control · encoder · calibration │
        │              snapshot · config · persistence │
        ├───────────────────────┬──────────────────────┤
  MCU   │  math/  (纯算法)      │  device/  (器件驱动)  │
        │  FOC · ServoMode      │  DRV8353S · MA600     │
        │  MitMode · Cal        │                       │
        ├───────────────────────┴──────────────────────┤
        │  core/ · ports/ · HAL/                       │
        └──────────────────────────────────────────────┘
```

**依赖规则**（由 Bazel `visibility` 与 BUILD 注释约束）：

- `math/` 不得依赖 HAL、device、middleware、protocol、app —— 保持纯算法、可移植、可离线测试
- `middleware/` 是唯一允许同时依赖 device/HAL 与 math 的业务层；通信和 Flash 适配器也归此层
- `application_core` 只编排 middleware 服务，不直接碰寄存器或 wire DTO
- `protocol/xt_can.h` 是固件侧**唯一线格式真源**，`tools/host/xt_proto.py` 与之一一对应

### 执行模型

单核裸机，无 RTOS，两条执行流：

| 上下文 | 频率 | 职责 |
|---|---|---|
| TIM5 控制 ISR（优先级 2） | 15 kHz | `MotorControlService::StepIsr()`：采样 → 外环 → FOC → PWM |
| 主循环 `Application::Run()` | 尽力而为 | 喂狗、状态机、命令/遥测服务轮询、Flash 持久化 |

Flash 擦写前主动关闭控制 ISR，避免写 Flash 阻塞造成控制周期抖动。

---

## 4. Flash 布局

```
0x08000000 ┬ 48 KB  中断向量表（reset → BootloaderEntry）
0x0800C000 ┼ 16 KB  Bootloader
0x08010000 ┼ 440 KB Application            ← APP_FLASH
0x0807E000 ┼  2 KB  Runtime Config (XCFG)  ← page 124
0x0807E800 ┼  2 KB  预留空隙
0x0807F000 ┴  4 KB  编码器/电机标定 (XENC) ← page 126-127
```

> 应用镜像长度被刻意限制为 440 KB，保证 NVS 两页在镜像之外；否则 Config Save 会擦掉正在运行的程序。

RAM `0x20000000-0x200001FF` 为 bootloader ↔ app 的复位保持握手区，两个链接脚本都从 `0x20000200` 起始。

---

## 5. 构建与烧录

### 依赖

- Bazel 7.4.1（`tools/bazel` 自动获取）
- ARM GCC 交叉工具链（由 rules_mbed / bazel_toolchain 提供）
- OpenOCD + CMSIS-DAP（烧录）
- Python 3.10+，`python-can`（上位机）

### 命令

```bash
# 编译应用
tools/bazel build //fw:app.elf

# 编译 bootloader + app 合并镜像
tools/bazel build //fw:xtellar.combined

# 一键烧录（自动构建后 OpenOCD 下载）
./flash.sh

# CAN 在线升级（无需拆机）
python3 tools/bootload_test.py
```

### 协议单元测试

```bash
python3 tools/host/test_xt_proto.py
```

---

## 6. CAN 协议

标准 11 位 ID，`node_id` 默认 1：

| 方向 | ID | 说明 |
|---|---|---|
| Host → Board | `0x100 + node_id` | 命令 |
| Board → Host | `0x180 + node_id` | 遥测 / 应答 |

所有帧以 4 字节 `Header{magic, version, type, seq}` 开头，小端紧凑排布。

### 命令（`kCmd*`）

| ID | 名称 | 说明 |
|---|---|---|
| 0 | Stop | 停止输出、关闭 ISR |
| 4 | Info | 查询固件版本与电机参数 |
| 5 | Snap | 请求 PWM 率快照（512 点 × 7 通道）|
| 6 | Servo | **统一控制入口**，由 `control` 字段选模式 |
| 7 | Cal | 启动标定子流程 |
| 8 | Query | 保活轮询，返回 `CtrlReply` |
| 9 | EncComp | 写入编码器几何补偿表 |
| 10 | Conf | 运行时配置 get/set/save/load/defaults |

### 遥测（`kType*`）

`Tel(2)` `Ack(3)` `Info(4)` `SnapMeta(5)` `SnapData(6)` `Enc(7)` `Cal(8)` `CtrlReply(9)` `Conf(10)`

`CtrlReply`（64 B）是主要的 Live 数据源，合并了电流、角度、速度、模式与编码器状态。

---

## 7. 控制模式

三种模式共用 `kCmdServo`（48 B `ServoRequest`），由 `control` 字段选择：

### `control = 0` — ServoMode（moteus `kPosition`）

轨迹生成 → 位置/速度 PID → 力矩 → Iq。支持 `position` / `velocity` / `stop_position` /
`velocity_limit` / `accel_limit` / `kp_kd_ilimit_scale` / `feedforward`。
`position = NaN` 表示纯速度跟踪。有限位置命令按**单圈 `[0, 2π)`** 解释，固件映射到最短路径。

### `control = 1` — Current（moteus `kCurrent`）

直接给定 Id / Iq 参考，跳过外环，仍用编码器 θe 换相。

### `control = 2` — MIT 阻抗

```
T = Kp · (P_cmd − P_fdb) + Kd · (V_cmd − V_fdb) + T_ff
```

**位置为多圈连续值**（PLL 解缠绕输出），不做单圈折返。
线格式字段复用：`position/velocity` = P/V 命令，`id_mA/iq_mA` = Kp/Kd（毫牛米单位），
`feedforward_mNm` = T_ff，`max_torque_mNm` = 力矩限幅。

> 兼容性：`HandleServo` 同时接受 8 B（遗留速度）、20 B（遗留 pos/vel）、≥48 B（完整）三种 payload。

---

## 8. 标定流程

推荐顺序：**R → Ld/Lq → 编码器转圈映射 → 编码器几何补偿 → 齿槽 → Ke**

| 子命令 | 方法 | 是否需编码器 |
|---|---|---|
| R 相电阻 | 固定 θ=0 的 Id 扫描线性回归 | 否 |
| Ld/Lq | 锁转子电压阶跃，测时间常数 τ=L/R | 否 |
| 编码器转圈映射 | 恒 D 轴电流正反各转两圈，生成 64 点换相补偿表 | — |
| 编码器几何补偿 | 恒速惯性法生成 256 点表，压制磁环偏心引起的 1/2 次纹波 | 是 |
| 齿槽补偿 | 慢速正反转记录 q 电流，生成 1024 点前馈表 | 是 |
| Ke 反电动势 | 闭环速度扫描 + Vq/Iq 回归 | 是 |

标定结果写入独立的标定 Flash（状态灯），**同时回写 Config → Motor 表（RAM）并立即生效**。
掉电保留需在 Config 页手动点 **Save**。

---

## 9. 运行时配置（Runtime Config）

`Motor` / `FOC` / `Servo` / `Encoder` 四组参数常驻 RAM，可持久化到独立 Flash 页。

**参数优先级：Config 的 Motor 表是 R / L / Ke 的唯一真源。**

启动顺序：

```
板级默认 (board_config.h)
    ↓
Runtime Flash 存在？ ── 是 ──→ 以 Flash 为准
    ↓ 否
用标定 Flash 的 R/L/Ke 预填 Motor 表
    ↓
ApplyRuntimeConfig() → 下发到 FOC / ServoMode / MitMode / EncoderPLL
    ↓
加载标定 Flash 的编码器表（offset / 齿槽 / 几何补偿），不覆盖电气参数
```

| 操作 | 作用域 |
|---|---|
| Refresh / Apply | 板端 RAM |
| Save / Load | Runtime Flash |
| Defaults | 恢复板级默认到 RAM |

---

## 10. 上位机

```bash
python3 tools/host/gui/web_server.py
# 打开 http://127.0.0.1:8765
```

四个页面：

- **Live** — 实时示波器，遥测通道树、信号质量统计、缩放平移
- **Snapshot** — PWM 率突发采样（512 点 × 7 通道）+ 频域分析
- **Cal** — 标定向导，含结果判定与建议
- **Config** — Motor/FOC/Servo/Encoder 参数表 + 标定 Flash 状态只读展示

上位机以 50 Hz 流式下发控制命令，板端回 `CtrlReply`；超过 500 ms 无命令则自动停止输出。

其他脚本：

```bash
python3 tools/host/bin_client.py --info      # 命令行探测
python3 tools/host/sine_vel.py               # 正弦速度带宽测试
python3 tools/host/compensate_encoder.py     # 离线生成编码器补偿表
```

---

## 11. 当前版本

固件语义化版本由 `fw/protocol/inc/protocol/xt_can.h` 的 `kFwMajor/Minor/Patch` 定义，
经 `kCmdInfo` 上报，上位机连接时显示。

**当前：0.6.6**（新增 MIT 阻抗模式）
