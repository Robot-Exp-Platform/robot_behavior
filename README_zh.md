# robot_behavior

[English](README.md) | [简体中文](README_zh.md) | [在线文档](../robot_behavior_page/docs/zh/index.md)

`robot_behavior` 是机器人驱动、仿真器和 Roplat 适配层共享的 Rust 行为抽象库。它描述的是机器人“能做什么”：在类型化空间中运动，暴露结构化状态，运行实时控制闭包，并在驱动具备模型能力时提供运动学、Jacobian 和动力学映射。

它不是某个硬件 SDK，也不是完整运动规划框架。它是一个契约 crate，让 Franka、JAKA、Hans、AUBO、仿真器和示例机器人在应用层呈现一致的接口。

## 能力

- 使用 `JointSpace<N>`、`FlangeSpace`、`TcpSpace`、base space 和 whole-body space 描述运动目标。
- 使用 `TorqueControl<N>`、`ArmTorqueControl<N>`、`JointPositionControl<N>`、`CartesianPoseControl<N>`、`BaseVelocityControl` 等类型化通道运行实时控制。
- 使用 `JointState<N>`、`ArmState<N>`、`BaseState`、`QuadrupedState<N>`、`HumanoidState<N>` 读取结构化状态。
- 复用 PD/PID、阻抗控制、重力补偿、computed torque 等控制器闭包。
- 用 `SpaceMap` 表达 FK、IK、Jacobian、动力学等模型能力。
- 让机械臂、四足、人形、移动底盘和仿真器以“能力 + 状态”的方式组合，而不是继承同一个根机器人类型。

## 依赖者

当前 workspace 中使用 `robot_behavior` 的主要 crate 包括：

- `franka-rust`
- `libjaka-rs`
- `libhans-rs`
- `libaubo-rs`
- `rsbullet`
- `roplat_exrobot`
- `roplat_rerun`
- `examples/jaka_dual`

下游实验 workspace 也通过 Cargo patch 使用同一套行为接口，从而避免实验代码绑定到某个具体硬件 crate。

## 基本用法

应用代码通过类型参数选择运动空间：

```rust
use robot_behavior::{JointSpace, Motion, RobotResult};

fn home<R>(robot: &mut R) -> RobotResult<()>
where
    R: robot_behavior::MoveTo<JointSpace<6>>,
{
    robot.move_to::<JointSpace<6>>([0.0; 6])
}
```

实时控制通过 `ControlWith<S>` 表示驱动支持的控制通道，通过 `control_with` 执行闭包：

```rust
use robot_behavior::{Control, RobotResult, TorqueControl};

fn one_torque_command<R>(robot: &mut R) -> RobotResult<()>
where
    R: robot_behavior::ControlWith<TorqueControl<7>>,
{
    robot.control_with::<TorqueControl<7>, _>(|_state, _dt| ([0.0; 7], true))
}
```

控制器 helper 返回普通 `FnMut` 闭包，可直接传入 `control_with`：

```rust
use robot_behavior::{
    Control, RobotResult, TorqueControl,
    utils::controller::joint_traj_pd_control,
};

fn track_traj<R>(robot: &mut R, traj: Vec<[f64; 7]>) -> RobotResult<()>
where
    R: robot_behavior::ControlWith<TorqueControl<7>>,
{
    let controller = joint_traj_pd_control(traj, [80.0; 7], [12.0; 7]);
    robot.control_with::<TorqueControl<7>, _>(controller)
}
```

COPP 轨迹也可以整理成实时控制闭包：

```rust
use robot_behavior::{
    Control, JointPositionControl, RobotResult,
    utils::trajectory::copp_waypoints_joint_position_control,
};

fn follow_waypoints<R>(robot: &mut R, waypoints: &[[f64; 7]]) -> RobotResult<()>
where
    R: robot_behavior::ControlWith<JointPositionControl<7>> + robot_behavior::Joints<7>,
{
    let generator = copp_waypoints_joint_position_control::<R, 7>(waypoints, 1.0)?;
    robot.control_with::<JointPositionControl<7>, _>(generator)
}
```

## 设计脉络

- `Robot`：生命周期和原生状态。
- `MoveTo<S>` / `MoveTraj<S>`：驱动支持的运动空间。
- `ControlWith<S>`：驱动支持的实时控制通道。
- `Arm<N>`、`MobileBase`、`Quadruped<N>`、`Humanoid<N>`：可组合的机器人能力束。
- `StateView<T>`：以 `meas` / `cmd` / `des` 表达 measured、commanded、desired 三类状态视角。
- `SpaceMap`：模型映射统一入口，例如 FK、Jacobian、质量矩阵、重力和科氏力。

`WholeBodyJointSpace<N>` 等 whole-body 运动空间仍用于区分整机关节运动；控制通道则统一复用 `TorqueControl<N>`、`JointPositionControl<N>`、`JointVelocityControl<N>`，避免为相同的输入输出形状重复定义控制类型。

## 控制退出与可选 roplat 适配

驱动实现统一入口 `ControlWith<S>::control_with_flow`。其闭包返回
`ControlStep<Command> = ControlFlow<(), (Command, bool)>`：

- `Continue((command, false))`：发送有效命令，继续周期。
- `Continue((command, true))`：先发送最后一条命令，再正常结束。
- `Break(())`：本周期不发送算法命令，进入设备协议规定的会话收尾。不统一替换成零命令、hold 或急停。

原有 `control_with`、`control_with_async` 保留为便利包装。`control_with_async` 和 `control_with_flow_async` 均延续 0.6 的**阻塞会话 + async 周期闭包**：并不返回会话 Future，也不保证外层同任务其他分支能在会话期间继续轮询。

默认 feature 为空，Robot / ControlWith / 状态 / 模型能力不依赖 roplat。显式启用 `features = ["roplat"]` 才提供 `ControlRhythm` 与三类节点适配。控制节律的 Input 为 `RobotResult<R>`，Yield 为 `(Obs, Duration)`，Feed 为 `(Command, bool)`，完整 drive 返回 `Execution<R>`。域错误和设备错误进入框架错误通道；无有效指令的周期通过 Break 结束。

合作退出都归还 N。设备作为 Input、不在 N 中时，只有正常完成才通过 Output 返回设备本身。生命周期属于创建层；反复进入 drive 不会重置或重新启用外部节点。域单独失败时保留原 RoplatError；设备失败通过 `RoplatError::Io` 包装可 downcast 的 `ControlSessionError`，其字段保留同时发生的域退出和设备错误；驱动双错用 `RobotException::ControlSession` 保留。错误包装仅在失败路径分配。

应用图使用 `#[roplat::system]`；可执行多层域示例见 [control_rhythm 测试](tests/control_rhythm.rs)，AI 开发配合 [roplat-skills](https://github.com/Robot-Exp-Platform/roplat-skills)。不要以手写应用 process 链替代 System 的生命周期和退出管理。

从 drives 根可执行：

```sh
cargo check -p robot_behavior --no-default-features --lib
cargo test -p robot_behavior --features roplat --lib --tests
cargo check -p robot_behavior --all-features --all-targets
cargo bench -p robot_behavior --features roplat --bench control_flow
```

性能样例比较 48、56、1024 字节命令的纯 CPU 成功路径；旧路径是对原控制循环的复现，因为旧适配不能直接对当前核心编译。结果不是实物机器人延迟，也不验证外层 runtime 公平性。Python/C++ 示例是可编译 mock wrapper，不是完整部署包或真机演示。


## 原生异步控制会话

新增 `AsyncControlWith<S>::control_native_async` 返回整个设备会话的 Future，包括异步进入与结束会话。它是单独的驱动能力，不会为仅实现 `ControlWith` 的阻塞驱动自动伪造异步实现；已有 0.6 接口语义保持不变。具体驱动说明其 I/O runtime 要求，行为接口本身不依赖 Tokio 或 roplat。

其回调为 `AsyncControlCallback<Obs, Command>`，`call(&mut self, ...)` 返回 `Send` Future。驱动在整个会话期间借用回调，并完整等待当前调用后再开始下一次；会话返回后，调用方可以取回回调中的状态。普通 `FnMut -> Send Future` 闭包直接适配；需要跨 await 借用自身可变状态的控制器可以实现这个静态分发 trait，无需每周期装箱、创建任务或复制状态。这里不承诺所有 lending async 闭包都能自动满足约束。

启用 `roplat` feature 后，使用 `robot_behavior::roplat::AsyncControlRhythm<R, S>` 将原生会话加入 System。Input/Yield/Feed/Output、创建层生命周期、错误保留和 N 归还规则与原 ControlRhythm 相同。新适配器不创建嵌套 runtime，不在每个周期 spawn 或装箱 Future。原生异步使 I/O 等待能够让出外层 executor；回调中的同步计算仍会占据 executor，直到其主动让出。

成功 Feed 按本周期提交：计算期间收到停止请求，但域仍返回有效 Feed，则发送该指令，下一 callback 前观察停止；若该有效 Feed 的 done=true，则正常完成。没有有效命令的域应返回 Stopped 或 Err。等待设备状态期间如何退出仍取决于驱动的等待及收尾协议，不自动给出统一时限；丢弃会话 Future 不等于合作关闭。

[原生异步测试](tests/async_control.rs) 在 current-thread runtime 下，让周期闭包真正等待同一任务 join 兄弟分支的信号，验证能够共同推进；同时覆盖挂起期间停止、主错与收尾错、非 Clone 节点归还、真实两层 System、整图 Send 编译约束与跨 drive 生命周期。这些有限 mock 测试不代表真机时序或物理安全认证。

```sh
cargo test -p robot_behavior --no-default-features --test async_control
cargo test -p robot_behavior --features roplat --test async_control
CONTROL_BENCH_CYCLES=1000000 cargo bench -p robot_behavior --features roplat --bench native_async_control
```

新增独立 CPU 基准比较 ControlRhythm 与 AsyncControlRhythm，采用相同的 48/56/1024 字节命令计算和 21 组交替先后的配对批次。输出每批平均 ns/周期及最大批均值，不是逐周期 p99、设备延迟，也不证明一直 Ready 的计算会自动公平调度。既有 control_flow 基准保持不变，用于独立回归对照。
