# roplat_safety

[English](README.md) | [简体中文](README_zh.md)

[`roplat_behavior`](../roplat_behavior/README_zh.md) 控制通道的安全门：关节限位、限速、样本新鲜度、周期看门狗与共享停止锁存。

属于 [roplat_behavior 工作区](../README_zh.md)。0.9 尚未发布到 crates.io，请按[从源码开发](../README_zh.md#从源码开发)从源码构建。

## 安全门怎样工作

安全门只检查、只拒绝，从不把一条命令修成另一条。对命令限幅属于控制器设计，应放在上游；限幅后的命令同样要经过检查。

拒绝是一个 `SafetyViolation`。在控制域中，`gate?` 把它变成失败的周期，设备会话因此收到 `Break`、不发命令；这正是 `ControlStep` 契约对无法给出命令的控制域规定的语义。

## 内容

- 各通道的命令检查，检查范围以及相邻两条命令之间的变化率：`JointPositionCheck`、`JointVelocityCheck`、`JointTorqueCheck` 与 `CartesianVelocityCheck`。
- 样本新鲜度（`AgeLimit`、`Sampled`）与周期看门狗（`Watchdog`）。
- 各控制域共享的停止锁存（`SafetyLatch`）。
- feature `roplat`：节点 `CommandGuard`、`FreshnessGate`、`CycleWatchdog`、`LatchGate` 与 `Trip`。`Trip` 在拒绝时触发锁存，`LatchGate` 之后的控制域随之停止。

这些检查是普通类型，不依赖 roplat 运行时也能使用。[设计](docs/design_zh.md)给出设计、取舍与尚未解决的问题。

## Feature

| Feature | 用途 |
|---|---|
| `roplat` | 节点与错误传递；检查本身不需要运行时。 |

默认 feature 为空。

## 许可证

Apache-2.0，见 [LICENSE](../LICENSE)。
