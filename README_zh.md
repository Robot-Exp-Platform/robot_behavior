# robot_behavior

[English](README.md) | [简体中文](README_zh.md) | [crates.io](https://crates.io/crates/robot_behavior)

`robot_behavior` 是一个 Rust 机器人接口库，让应用围绕**能力**编写：到达目标、读取结构化状态，或在每个控制周期计算下一条指令。硬件驱动或仿真后端实现这些能力，本库提供共同的类型和接口契约。

如果你希望在兼容的后端之间复用应用与控制器，或为新驱动建立一致的接口，可以使用本库。连接真机还需要选择对应驱动；物理仿真需要 [RsBullet](https://github.com/Robot-Exp-Platform/rsbullet) 等后端。本库自身不完成这两项工作。

## 设计理念：统一接口，把设备差异留在驱动中

关节位置与关节力矩都可能是 `[f64; 6]`，但含义不同。`JointSpace<6>` 表示运动目标空间，`JointPositionControl<6>` 与 `TorqueControl<6>` 表示周期控制通道。这些类型选择对应的观测和指令类型，让应用明确表达操作意图。

各项能力独立实现。支持关节运动的驱动，不会自动获得力矩控制、逆运动学或原生异步 I/O。泛型应用声明需要的 trait，不必假定所有机器人都提供相同功能。

驱动负责通信、周期组织、状态采集和会话收尾。控制闭包接收观测值与 `Duration`，计算下一条指令。控制器辅助函数返回普通闭包，不自行创建调度器，也不提供硬件实时性保证。

| 想完成的操作 | 应用入口 | 后端实现 |
|---|---|---|
| 到达一个目标 | `Motion::move_to::<S>` | `MoveTo<S>` |
| 发送采样轨迹 | `Motion::move_traj::<S>` | `MoveTraj<S>` |
| 每周期计算指令 | `Control::control_with::<S, _>` | `ControlWith<S>` |
| 等待完整异步控制会话 | `AsyncControl::control_native_async::<S, _>` | `AsyncControlWith<S>` |
| 读取后端原生状态 | `Robot::read_state` | `Robot` |
| 使用机械臂状态或模型 | `Arm<N>`、`SpaceMap` 等模型 trait | 后端实际支持的能力 |

## 第一个程序：不连接机器人

当前发布版本为 **0.6.1**。依赖构建需要 C++ 工具链（Windows 使用 MSVC Build Tools 和 Windows SDK），库本身使用 nightly Rust 特性。先安装工具链并创建项目：

```sh
rustup toolchain install nightly
cargo new behavior-demo --edition 2024
cd behavior-demo
```

在生成的 `Cargo.toml` 中添加依赖：

```toml
[dependencies]
robot_behavior = "0.6.1"
roplat_exrobot = "0.2.0"
```

`roplat_exrobot` 提供把指令打印到控制台的参考机器人，返回合成观测值。这个例子不需要设备、SDK、机器人模型或 Roplat 运行时。将 `src/main.rs` 替换为：

```rust
use robot_behavior::{
    Control, JointPositionControl, JointSpace, Motion, Robot, RobotResult,
};
use roplat_exrobot::ExRobot;

fn main() -> RobotResult<()> {
    let mut arm = ExRobot::<6>::new();
    arm.init()?;
    arm.move_to::<JointSpace<6>>([0.1; 6])?;

    let mut cycles = 0;
    arm.control_with::<JointPositionControl<6>, _>(|_state, _dt| {
        cycles += 1;
        ([0.1; 6], cycles == 3)
    })?;

    println!("completed {cycles} control cycles");
    arm.shutdown()
}
```

执行 `cargo +nightly run`。程序会打印参考机器人的生命周期与指令信息，随后输出 `completed 3 control cycles` 并结束。第三次闭包返回 `true`，表示发送本次指令后完成会话。这里演示的是接口调用和退出流程；参考机器人不会产生物理运动或积分更新状态。

接下来接入设备时，替换 `ExRobot` 的构造过程，按对应驱动完成连接与运行模式配置，并仅保留它支持的通道。相同的 Rust 类型便于复用代码，但机器人限位、坐标系、周期和控制器增益仍需分别确认。

## 理解状态

`Robot::State` 由后端定义。需要跨后端使用机械臂状态时，查看驱动的 `Arm<N>` 实现和统一状态类型：

- `JointState<N>` 包含 `meas`、`cmd`、`des` 三种视图，分别表示测量反馈、接受的指令和期望参考。
- `JointSample<N>` 中的 `q`、`dq`、`tau` 等字段使用 `Option`。`None` 表示没有提供该值，不应直接解释为零。
- `ArmState<N>` 还包含法兰状态，以及可选的 TCP、刚度坐标系和负载信息。原生状态与统一视图不一定通过同一个设备操作获取。

各后端文档会说明单位、坐标系、字段可用性与状态新鲜度。默认值或 `Some` 的存在本身，不能证明新传感器数据已经到达。

## 运动调用与控制会话

希望由后端执行目标或轨迹时，使用运动接口；需要自己的代码逐周期产生指令时，使用控制会话。`control_with` 的闭包返回 `(command, done)`，整个调用阻塞到会话结束。

如果需要在本周期没有算法指令时退出，使用 `control_with_flow`，返回 `ControlStep<Command>`：

| 闭包返回值 | 含义 |
|---|---|
| `ControlFlow::Continue((command, false))` | 发送指令，继续执行。 |
| `ControlFlow::Continue((command, true))` | 发送这条最终指令，然后正常完成。 |
| `ControlFlow::Break(())` | 本周期不发送算法指令，按驱动协议收尾。 |

收尾行为由设备协议决定。`Break` 不统一替换成零指令，也不等同于急停。

异步接口有两种不同含义：

- `control_with_async` / `control_with_flow_async` 是**阻塞会话中的异步闭包**，不会返回整个会话的 Future。
- `control_native_async(&mut callback)` 返回包含 I/O 与收尾的**完整会话 Future**。它要求驱动实现 `AsyncControlWith<S>`，并使用驱动规定的运行时。直接丢弃 Future 不等同于合作退出。

## Feature 与 Roplat 集成

默认 feature 为空。Rust 运动、控制、状态和模型接口不依赖 Roplat。

| Feature | 用途 |
|---|---|
| `roplat` | `ControlRhythm`、`AsyncControlRhythm`，以及运动、模型和约束节点适配。 |
| `ffi` / `to_c` | 外部语言接口模块 / C 接口特性入口。 |
| `to_cxx` | C++ bridge 支持。 |
| `to_py` | PyO3 支持和导出的状态类型。 |

在 Roplat 应用中，为本库依赖添加 `features = ["roplat"]`，并使用 `roplat = "0.3.0"`。应用图通过 `#[roplat::system]` 构建：阻塞后端使用 `ControlRhythm`，原生异步后端使用 `AsyncControlRhythm`。适配器把同一套观测和指令接到图中，不会补齐设备驱动缺少的能力。

节点生命周期由创建它的作用域管理。作为节律输入传入的设备，在正常完成时作为输出返回；错误处理不能因为图节点被归还，就假设设备对象也一定能够取回。接入图时可继续阅读[适配层源码与接口注释](src/roplat)及[完整 System 示例](tests/control_rhythm.rs)。

## 继续阅读

- [运动与轨迹接口](src/robot/motion.rs)：目标、稠密轨迹，以及驱动提供的路径和途经点处理。
- [控制契约](src/robot/control.rs)与[原生异步契约](src/robot/async_control.rs)。
- [状态类型](src/robot/state.rs)与[模型映射](src/robot/model.rs)。
- [控制器辅助函数](src/utils/controller)：关节 PD/PID、阻抗、重力补偿等计算；增益和模型数据按具体后端配置。
- [可执行契约示例](tests/control_flow.rs)、[异步示例](tests/async_control.rs)与[外部语言接口示例](examples)。
- [配套文档仓库](https://github.com/Robot-Exp-Platform/robot_behavior_page)。

驱动作者可导入 `robot_behavior::driver::*`，应用可导入 `robot_behavior::behavior::*`。先在 `Robot` 中实现设备生命周期和状态，再逐项实现支持的运动、控制与模型能力。默认生命周期钩子是空操作，部分未实现操作会返回错误；继承默认方法不代表完成了对应设备功能。

## 从源码开发

使用已发布版本时，优先选择 registry 依赖。本仓 manifest 为集成开发记录了固定 Git 来源，包括可选依赖。因此源码构建即使关闭某个 feature，也可能需要 GitHub SSH 访问；它不要求同级存在 `drives` 仓库。

在源码目录执行 `cargo +nightly check --lib` 可检查默认 Rust 库。`tests/` 中的有限 mock 示例验证接口契约，不代表硬件时序结果。整合工作区开发见 [drives](https://github.com/Robot-Exp-Platform/drives)。

## 许可

Apache-2.0，详见 [LICENSE](LICENSE)。
