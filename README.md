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
use robot_behavior::{JointSpace, Motion, RobotResult};

fn home<R>(robot: &mut R) -> RobotResult<()>
where
    R: robot_behavior::MoveTo<JointSpace<6>>,
{
    robot.move_to::<JointSpace<6>>([0.0; 6])
}
```

Realtime control is selected through type-level control channels:

```rust
use robot_behavior::{Control, RobotResult, TorqueControl};

fn one_torque_command<R>(robot: &mut R) -> RobotResult<()>
where
    R: robot_behavior::ControlWith<TorqueControl<7>>,
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
- `roplat`: optional Node, blocking ControlRhythm and native AsyncControlRhythm adapters.

The core Rust behavior API works with default features.

## Status

`robot_behavior` is still evolving with the driver workspace. The current direction is stable at the design level: represent robots as capabilities, typed spaces and reusable controller / model skills. Some trait details may still change as more drivers and robot forms are integrated.

## Control sessions and Roplat

The driver implements `ControlWith<S>::control_with_flow`. Its callback returns
`ControlStep<Command> = ControlFlow<(), (Command, bool)>`:

- `Continue((command, false))`: send the command and continue.
- `Continue((command, true))`: send this final command, then complete normally.
- `Break(())`: send no algorithm command this cycle, then perform device-specific
  session termination. This does not prescribe a zero command, hold or emergency stop.

The existing tuple callback methods `control_with` and `control_with_async` are
wrappers. Both `control_with_async` and `control_with_flow_async` are **blocking
sessions with async per-cycle callbacks**, preserving version 0.6 semantics.
They do not return a session Future or promise that siblings on an outer
executor can run while the session blocks.

Default features are empty: using Robot, ControlWith, motion, state or models
requires no roplat dependency. Enable `features = ["roplat"]` to import
`robot_behavior::roplat::{ControlRhythm, MotionNode, SpaceMapNode, SafetyNode}`.
ControlRhythm accepts `RobotResult<R>`, yields `(Obs, Duration)`, receives
`(Command, bool)`, and returns `Execution<R>`. Domain or device failure is a
framework error; a stopped/failed cycle does not require an invented command.
Every cooperative exit returns N. The robot Input is outside N, so the robot
itself is only returned as Output on completion. The creating scope owns
Lifecycle; repeated drives do not reset or reactivate external nodes.

A domain failure alone retains its original RoplatError. Device failures use
`RoplatError::Io` with a downcastable `ControlSessionError`, retaining both the
domain exit and device error when needed. Drivers retain operation and cleanup
failures with `RobotException::ControlSession`. Allocations for these wrappers
occur only on failure.

Use `#[roplat::system]` for application graphs; see the executable multi-layer
examples in [control_rhythm tests](tests/control_rhythm.rs) and
[roplat-skills](https://github.com/Robot-Exp-Platform/roplat-skills). Implementing
a Node's process method is normal; replacing the application's entire graph
with manual process calls bypasses System semantics.

Finite checks (from the parent drives workspace):

```sh
cargo check -p robot_behavior --no-default-features --lib
cargo test -p robot_behavior --features roplat --lib --tests
cargo check -p robot_behavior --all-features --all-targets
cargo bench -p robot_behavior --features roplat --bench control_flow
```

The benchmark compares CPU-only successful paths with 48, 56 and 1024-byte
commands. Its legacy path reproduces pre-change loop code because that older
adapter cannot compile against the current core. It does not measure physical
robot latency or runtime fairness. Foreign-language examples are compileable
mock wrappers, not complete deployment packages or device demonstrations.

## Independent source checkout

The optional roplat adapter uses the full Git revision pinned in Cargo.toml.
It does not require a sibling roplat checkout or the drives workspace. This is
an internal Git baseline: the crates.io package with the same version number
predates the current core execution API. Repository SSH access must already be
configured; set `CARGO_NET_GIT_FETCH_WITH_CLI=true` to use your existing SSH key
or agent. Do not put credentials in project files.

```sh
CARGO_NET_GIT_FETCH_WITH_CLI=true cargo check --no-default-features --lib
CARGO_NET_GIT_FETCH_WITH_CLI=true cargo check --no-default-features --features roplat --lib
```

Cargo may inspect the pinned source while resolving optional dependencies even
when the feature is off. Feature separation removes core compilation/runtime
dependencies from the default build; it is not an offline download guarantee.


## Native asynchronous control

`AsyncControlWith<S>::control_native_async` returns a **complete session Future**,
including asynchronous entry and termination. It is a separate driver capability:
implementing the blocking `ControlWith` does not automatically implement it, and
none of the version 0.6 blocking methods change meaning. Drivers document their
I/O runtime requirements; the behavior interfaces themselves have no Tokio or
roplat dependency.

```rust
use robot_behavior::{AsyncControl, AsyncControlWith, JointPositionControl, RobotResult};
use std::ops::ControlFlow;

async fn one_command<R>(robot: &mut R, command: [f64; 7]) -> RobotResult<()>
where
    R: AsyncControlWith<JointPositionControl<7>>,
{
    let mut callback = move |_state, _dt| async move {
        ControlFlow::Continue((command, true))
    };
    AsyncControl::control_native_async::<JointPositionControl<7>, _>(robot, &mut callback).await
}
```

The canonical callback is `AsyncControlCallback<Obs, Command>::call(&mut self, ...)
-> impl Future<Output = ControlStep<Command>> + Send`. A driver borrows the callback
for the whole session and awaits every call before starting another. State is
available to its caller after success or error. Ordinary `FnMut -> Send Future`
closures work directly. A controller that needs to borrow its own mutable state
across await can implement the trait on a named struct; this avoids a boxed future
or a copied state value per cycle. It does not promise that every lending async
closure automatically meets the callback contract.

Enable `roplat` and use `robot_behavior::roplat::AsyncControlRhythm<R, S>` for the
native session in a System graph. Its Input/Yield/Feed/Output, creator lifecycle,
error retention and N-return rules match ControlRhythm. It neither constructs a
nested runtime nor spawns or boxes a future per cycle. Native async makes I/O
suspension visible to the caller's executor; synchronous computation inside a
callback can still occupy that executor until it yields.

A successful Feed is committed for the current cycle even if stop was requested
while it was computing. Stop is observed before the next callback; a valid final
Feed with `done = true` completes normally. A domain with no command must return
`Stopped` or `Err`. Stopping while waiting for device input follows the driver's
wait/termination policy, not an automatically imposed timing guarantee. Dropping
the session future is not cooperative shutdown.

[Native tests](tests/async_control.rs) run on a current-thread runtime and include
a callback that can only resume when a same-task join sibling signals it. They
also exercise pending-domain stop, original plus cleanup errors, non-Clone node
return, real nested System execution and the complete graph's Send bound. These
are finite mock checks, not hardware timing or safety validation.

```sh
cargo test -p robot_behavior --no-default-features --test async_control
cargo test -p robot_behavior --features roplat --test async_control
CONTROL_BENCH_CYCLES=1000000 cargo bench -p robot_behavior --features roplat --bench native_async_control
```

The independent native benchmark compares existing ControlRhythm with
AsyncControlRhythm using the same synthetic 48/56/1024-byte command work and 21
alternating paired batches. It reports batch-average nanoseconds per cycle and
the maximum batch mean, not individual-cycle p99, device latency or a claim that
an always-ready callback is fairly scheduled. The older control_flow benchmark
remains unchanged for separate regression comparison.
