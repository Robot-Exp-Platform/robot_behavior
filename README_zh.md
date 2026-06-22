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
use robot_behavior::{JointSpace, MoveTo, RobotResult};

fn home<R>(robot: &mut R) -> RobotResult<()>
where
    R: MoveTo<JointSpace<6>>,
{
    robot.move_to::<JointSpace<6>>([0.0; 6])
}
```

实时控制通过 `ControlWith<S>` 表示驱动支持的控制通道，通过 `control_with` 执行闭包：

```rust
use robot_behavior::{Control, ControlWith, RobotResult, TorqueControl};

fn hold_zero_torque<R>(robot: &mut R) -> RobotResult<()>
where
    R: ControlWith<TorqueControl<7>>,
{
    robot.control_with::<TorqueControl<7>, _>(|_state, _dt| ([0.0; 7], true))
}
```

控制器 helper 返回普通 `FnMut` 闭包，可直接传入 `control_with`：

```rust
use robot_behavior::{
    Control, ControlWith, RobotResult, TorqueControl,
    controller::joint_traj_pd_control,
};

fn track_traj<R>(robot: &mut R, traj: Vec<[f64; 7]>) -> RobotResult<()>
where
    R: ControlWith<TorqueControl<7>>,
{
    let controller = joint_traj_pd_control(traj, [80.0; 7], [12.0; 7]);
    robot.control_with::<TorqueControl<7>, _>(controller)
}
```

COPP 轨迹也可以整理成实时控制闭包：

```rust
use robot_behavior::{
    Control, ControlWith, JointPositionControl, RobotResult,
    utils::trajectory::copp_waypoints_joint_position_control,
};

fn follow_waypoints<R>(robot: &mut R, waypoints: &[[f64; 7]]) -> RobotResult<()>
where
    R: ControlWith<JointPositionControl<7>> + robot_behavior::Joints<7>,
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
