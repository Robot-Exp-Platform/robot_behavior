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

下方 registry 入门示例使用 `robot_behavior 0.6.1` 和 `roplat_exrobot 0.2.0`。本分支通过固定 Git 来源准备 **0.8.0** 与配套 Roplat 0.5.0；这些准备版本尚未上传 crates.io。依赖构建需要 C++ 工具链（Windows 使用 MSVC Build Tools 和 Windows SDK），库本身使用 nightly Rust 特性。先安装工具链并创建项目：

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

在 Roplat 应用中，为本库依赖添加 `features = ["roplat"]`，并使用匹配的 Roplat 0.5.0 固定 Git 来源（尚未上传 crates.io）。应用图通过 `#[roplat::system]` 构建：阻塞后端使用 `ControlRhythm`，原生异步后端使用 `AsyncControlRhythm`。适配器把同一套观测和指令接到图中，不会补齐设备驱动缺少的能力。

机器人始终由创建层持有；节点和节律通过独立的 `Resources` 通道接收可变借用，业务 Input 和 Output 不承载机器人。

```rust
#[roplat::system]
async fn session(mut robot: MyRobot, mut law: MyLaw) -> RoplatResult<MyRobot> {
    let mut control = ControlRhythm::<MyRobot, JointPositionControl<7>>::new();
    let mut reach = MotionNode::<MyRobot, JointSpace<7>>::new();
    let mut goal = Goal::new();

    &mut robot >> reach.resources.0;
    goal >> reach;
    reach.output?;
    &mut robot >> control.resources.0;
    control >> |state| state >> law;

    Ok(robot)
}
```

节点与机器人的生命周期由创建它们的作用域管理。`MotionNode` 的 `Resources = (R,)`、`Input = S::Target`、`Output = RobotResult<()>`；两个控制节律的 `Resources = (R,)`、`Input = Output = ()`。可变借用持续一次 process 或完整 drive，成功、失败和停止均不需要归还边或 reclaim 钩子。`reach.output?` 显式将业务错误提升为 System 失败。接入图时可继续阅读[适配层源码与接口注释](src/roplat)及[完整 System 示例](tests/control_rhythm.rs)。

## 组合机器人模型与仿真后端

同一种设备可以在 `RobotDescription::SIMULATION_MODELS` 中提供多个模型。
加载器实现 `SimulationBackend`，声明后端标识与实际接受的格式，再通过
`select_robot_model::<R>()` 选择模型。只声明旧 `URDF` 常量的设备仍可由接受
URDF 的后端使用。

```rust
use robot_behavior::{ModelFormat, RobotDescription, SimulationModel};

struct ArmModels;
impl RobotDescription for ArmModels {
    const SIMULATION_MODELS: &'static [SimulationModel] = &[
        SimulationModel::new(ModelFormat::Urdf, "arm/robot.urdf"),
        SimulationModel::new(ModelFormat::Mjcf, "arm/scene.xml").for_backend("mujoco"),
    ];
}
```

选择顺序是：指定后端的模型、通用模型、旧 `URDF`；每层先按后端的格式优先级，
再按设备的声明顺序。`SelectedSimulationModel` 记录选择来源；不匹配时返回
带有后端和接受格式的 `ModelSelectionError`。路径遵循后端的资源搜索规则。
选中的文件加载失败时应报告失败，不应静默换用另一份模型并改变物理条件。

这层接口只选择模型，不自动转换文件或实现仿真器。后端仍需验证模型，并提供
实际支持的关节、控制、传感器和场景能力。操作系统与构建条件由后端安装契约
说明；接受 MJCF 不表示当前电脑已安装或支持 MuJoCo。仿真步进继续使用独立
的 `PhysicsEngine`，不改变真机的运动与控制语义。
[选择测试](tests/simulation_models.rs) 覆盖设备与后端匹配及不匹配的组合，
不启动仿真器。

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
