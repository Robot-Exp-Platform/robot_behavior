# robot_behavior

[English](README.md) | [简体中文](README_zh.md) | [Documentation](../robot_behavior_page/docs/en/index.md)

`robot_behavior` is the shared Rust behavior layer for robot drivers, simulators and Roplat adapters. It defines the common language for "what a robot can do": move in typed spaces, expose structured state, run realtime control closures, and provide kinematics / dynamics maps when a driver has a model.

It is not a hardware SDK and it is not a motion-planning framework. It is the contract crate that lets different backends feel like the same kind of robot from application code.

## What It Does

`robot_behavior` gives downstream crates a common API for:

- Moving robots in typed spaces such as `JointSpace<N>`, `FlangeSpace`, `TcpSpace`, base spaces and whole-body spaces.
- Running realtime control loops through typed channels such as `TorqueControl<N>`, `ArmTorqueControl<N>`, `CartesianPoseControl<N>` and `BaseVelocityControl`.
- Reading structured state through `JointState<N>`, `ArmState<N>`, `BaseState`, `QuadrupedState<N>` and `HumanoidState<N>`.
- Sharing controller skills such as PD/PID tracking, impedance control, gravity compensation and computed-torque control.
- Expressing FK, IK, Jacobian and dynamics as typed `SpaceMap` implementations.
- Letting arms, humanoids, quadrupeds, mobile bases and simulators share reusable behavior without forcing them into one root robot type.

## Why Use It

The main advantage is consistency across very different robots and backends.

- **Typed commands instead of ambiguous arrays**: `[f64; 7]` becomes meaningful only when paired with `JointSpace<7>`, `TorqueControl<7>` or another marker.
- **One application style across drivers**: user code can call `move_to::<JointSpace<N>>()` or `control_with::<TorqueControl<N>, _>()` against any compatible backend.
- **Driver-friendly abstraction**: drivers implement only the spaces and control channels they actually support.
- **Reusable controller closures**: controller helpers return plain `FnMut` closures, so they plug directly into realtime control loops.
- **Robot form is compositional**: an arm, dog or humanoid can be modeled as capabilities plus state, rather than being forced into one rigid inheritance tree.
- **Model APIs are optional**: kinematics and dynamics live behind typed maps, so a simple driver can skip them and a rich driver can expose them cleanly.

## Who Depends On It

In this workspace, `robot_behavior` is used by:

- `franka-rust`: Franka Emika / FR3 driver.
- `libjaka-rs`: JAKA robot driver.
- `libhans-rs`: Hans robot driver.
- `libaubo-rs`: AUBO robot driver.
- `rsbullet`: Bullet-based simulation backend.
- `roplat_exrobot`: example / adapter robots exposed as Roplat nodes.
- `roplat_rerun` and `utils/rerun_urdf`: visualization-related crates.
- `examples/jaka_dual` and other workspace examples.

It is also patched into downstream experiment workspaces so experiments can consume the same behavior interface without depending on a specific hardware crate.

## Core Idea

Application code selects behavior through type-level spaces:

```rust
use robot_behavior::{JointSpace, Motion, MoveTo, RobotResult};

fn home<R>(robot: &mut R) -> RobotResult<()>
where
    R: MoveTo<JointSpace<6>>,
{
    robot.move_to::<JointSpace<6>>([0.0; 6])
}
```

Realtime control is selected through type-level control channels:

```rust
use robot_behavior::{Control, ControlWith, RobotResult, TorqueControl};

fn hold_zero_torque<R>(robot: &mut R) -> RobotResult<()>
where
    R: ControlWith<TorqueControl<7>>,
{
    robot.control_with::<TorqueControl<7>, _>(|_state, _dt| {
        ([0.0; 7], true)
    })
}
```

The channel determines what state the closure receives. For example, `TorqueControl<N>` observes `JointState<N>`, while `ArmTorqueControl<N>` observes full `ArmState<N>` for Cartesian impedance, Jacobians or dynamics-aware control.

## Controller Skills

The controller helpers are intentionally small and composable. They build realtime closures rather than controller objects:

```rust
use robot_behavior::{
    Control, ControlWith, RobotResult, TorqueControl,
    utils::controller::joint_traj_pd_control,
};

fn track_traj<R>(robot: &mut R, traj: Vec<[f64; 7]>) -> RobotResult<()>
where
    R: ControlWith<TorqueControl<7>>,
{
    let controller = joint_traj_pd_control(traj, [80.0; 7], [12.0; 7]);
    robot.control_with::<TorqueControl<7>, _>(controller)
}
```

Available controller families include:

- Joint PD / PID fixed target, dynamic target and trajectory tracking.
- Joint impedance fixed target, dynamic target, trajectory tracking and handle-driven sessions.
- Cartesian impedance with FK / Jacobian model support.
- Gravity compensation.
- Computed-torque tracking.
- Base velocity PID.

## State Model

State is represented as measured / commanded / desired views:

```rust
pub struct StateView<T> {
    pub meas: T,
    pub cmd: T,
    pub des: T,
}
```

For arms, the primary state is:

```rust
pub struct ArmState<const N: usize> {
    pub joint: JointState<N>,
    pub flange: StateView<SpatialSample>,
    pub tcp: Option<StateView<SpatialSample>>,
    pub stiffness: Option<StateView<SpatialSample>>,
    pub load: Option<LoadState>,
}
```

The field names are explicit at the robot-structure level (`joint`, `flange`, `tcp`) and use standard robotics notation inside samples (`q`, `dq`, `tau`).

## For Driver Authors

A typical arm driver implements:

- `Robot` for lifecycle and native state.
- `Joints<N>` and `EndPoint` for limits.
- `MoveTo<S>` and optionally `MoveTraj<S>` for supported motion spaces.
- `ControlWith<S>` for supported realtime channels.
- `Arm<N>` for the unified arm surface.
- Optional `SpaceMap` / model traits for FK, IK, Jacobian and dynamics.

Driver crates should normally import:

```rust
use robot_behavior::driver::*;
```

Application crates should normally import:

```rust
use robot_behavior::behavior::*;
```

## Feature Flags

- `ffi`: FFI module gates.
- `to_py`: PyO3 support.
- `to_cxx`: `cxx` support.
- `to_c`: C-facing gates.

The core Rust behavior API works with default features.

## Status

`robot_behavior` is still evolving with the driver workspace. The current direction is stable at the design level: represent robots as capabilities, typed spaces and reusable controller / model skills. Some trait details may still change as more drivers and robot forms are integrated.
