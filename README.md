# robot_behavior

[English](README.md) | [简体中文](README_zh.md) | [crates.io](https://crates.io/crates/robot_behavior)

`robot_behavior` is a Rust interface library for writing robot applications against **capabilities**: move to a target, read a structured state, or calculate a command each control cycle. A hardware driver or simulator implements those capabilities; this crate supplies their shared types and contracts.

Use it when you want to reuse application or controller code across compatible backends, or implement a new driver in the same vocabulary. To connect to a robot, also choose its driver. To simulate physics, choose a physics backend such as [RsBullet](https://github.com/Robot-Exp-Platform/rsbullet). This crate alone does neither.

## Design: share the contract, keep the device-specific work in the driver

A joint position and a joint torque can both be `[f64; 6]`, but they mean different things. `JointSpace<6>` identifies a motion target; `JointPositionControl<6>` and `TorqueControl<6>` identify cyclic command channels. These marker types select the observation and command types and keep application calls explicit.

Capabilities are implemented separately. A driver supporting joint motion does not automatically support torque control, inverse kinematics, or native asynchronous I/O. Generic application code requests the traits it needs, rather than assuming every robot offers the same features.

The driver owns transport, timing, state acquisition, and session termination. Your control callback receives an observation and a `Duration`, then produces the next command. Controller helpers are ordinary closures; they do not create a scheduler or make a hardware timing guarantee.

| You want to… | Application API | Backend implements |
|---|---|---|
| Reach a target | `Motion::move_to::<S>` | `MoveTo<S>` |
| Send a sampled trajectory | `Motion::move_traj::<S>` | `MoveTraj<S>` |
| Compute a command every cycle | `Control::control_with::<S, _>` | `ControlWith<S>` |
| Await a complete control session | `AsyncControl::control_native_async::<S, _>` | `AsyncControlWith<S>` |
| Read a backend's native state | `Robot::read_state` | `Robot` |
| Work with arm state or a model | `Arm<N>`, `SpaceMap` and model traits | Only the supported capabilities |

## First example: a complete program without a robot

The current release is **0.6.1**. Dependency builds need a C++ toolchain (on Windows, MSVC Build Tools and the Windows SDK). It uses nightly Rust features, so install a nightly toolchain:

```sh
rustup toolchain install nightly
cargo new behavior-demo --edition 2024
cd behavior-demo
```

Add these dependencies to the generated `Cargo.toml`:

```toml
[dependencies]
robot_behavior = "0.6.1"
roplat_exrobot = "0.2.0"
```

`roplat_exrobot` supplies a console-backed reference robot. It prints commands and returns synthetic observations, so the example needs no device, SDK, robot model, or Roplat runtime. Replace `src/main.rs` with:

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

Run it with `cargo +nightly run`. You will see the reference robot's lifecycle and command messages, followed by `completed 3 control cycles` and shutdown. The final callback returns `true`, so its command is sent and the session finishes. The example demonstrates dispatch and termination; the reference robot does not move or integrate a physical state.

To use a device next, replace `ExRobot` with the appropriate driver's constructor and follow that driver's connection and operating-mode setup. Keep only the channels that driver implements. Matching Rust types make code reusable; they do not make robot limits, frames, timing, or controller gains interchangeable.

## Reading state

`Robot::State` is backend-specific. For portable arm-oriented code, use the driver's `Arm<N>` implementation and the structured state types:

- `JointState<N>` contains `meas`, `cmd`, and `des` views: measured feedback, accepted commands, and desired references.
- A `JointSample<N>` contains optional fields such as `q`, `dq`, and `tau`. `None` means that value was not supplied; it should not be silently treated as zero.
- `ArmState<N>` adds flange state and optional TCP, stiffness-frame, and load information. A native state and this portable view are not necessarily acquired through the same device operation.

Read the backend's state documentation for units, frames, field availability, and freshness. A default value, or the presence of `Some`, is not proof that a new sensor sample has arrived.

## Motion and control sessions

Use a motion call when the backend should execute a target or trajectory. Use a control session when your code must produce each cycle's command. `control_with` receives a callback returning `(command, done)` and blocks until the session ends.

For an exit without an algorithm command, use `control_with_flow` and return a `ControlStep<Command>`:

| Callback result | Meaning |
|---|---|
| `ControlFlow::Continue((command, false))` | Send the command; continue. |
| `ControlFlow::Continue((command, true))` | Send this final command; complete normally. |
| `ControlFlow::Break(())` | Send no algorithm command for this cycle; perform the driver's session termination. |

Termination depends on the device protocol. `Break` does not prescribe a zero command or represent an emergency stop.

There are two different uses of async:

- `control_with_async` / `control_with_flow_async` accept async **callbacks inside a blocking session**. They do not return a session future.
- `control_native_async(&mut callback)` returns the **whole session future**, including its I/O and termination. It requires an `AsyncControlWith<S>` implementation and the runtime specified by the driver. Dropping that future is not cooperative session shutdown.

## Features and Roplat integration

Default features are empty. The Rust motion, control, state, and model interfaces work without Roplat.

| Feature | Purpose |
|---|---|
| `roplat` | `ControlRhythm`, `AsyncControlRhythm`, and motion/model/safety node adapters. |
| `ffi` / `to_c` | Enable the foreign-interface modules / C-facing feature gate. |
| `to_cxx` | Enable the C++ bridge support. |
| `to_py` | Enable PyO3 support and exported state types. |

For a Roplat application, add `features = ["roplat"]` to the dependency and use `roplat = "0.3.0"`. Build the application graph with `#[roplat::system]`; choose `ControlRhythm` for a blocking backend or `AsyncControlRhythm` for a native async backend. The adapters connect the same observations and commands to a graph; they do not add capabilities missing from the device driver.

The creating scope manages node lifecycle. A device passed as a rhythm's input is returned as output on normal completion; failure handling must not assume it is recoverable simply because graph nodes are returned. See the [adapter implementation and API comments](src/roplat) and [complete System examples](tests/control_rhythm.rs) when integrating a graph.

## Going further

- [Motion and trajectory APIs](src/robot/motion.rs): targets, dense trajectories, and driver-provided path/waypoint handling.
- [Control contracts](src/robot/control.rs) and [native async contracts](src/robot/async_control.rs).
- [State types](src/robot/state.rs) and [model mappings](src/robot/model.rs).
- [Controller helpers](src/utils/controller): joint PD/PID, impedance, gravity compensation, and other reusable calculations. Choose gains and required model data for your backend.
- [Executable contract examples](tests/control_flow.rs), [async examples](tests/async_control.rs), and [foreign-interface examples](examples).
- [Companion documentation](https://github.com/Robot-Exp-Platform/robot_behavior_page).

Driver authors can import `robot_behavior::driver::*`; applications can import `robot_behavior::behavior::*`. Implement the device's lifecycle and state in `Robot`, then each supported motion/control/model capability. Default lifecycle hooks are no-ops and some unimplemented operations return errors; inheriting a default is not an implementation of device behavior.

## Working from source

Registry dependencies are the simplest way to use a released version. This repository's manifest also records pinned Git sources for integration development, including optional dependencies. A source checkout may therefore need GitHub SSH access even when an optional feature is disabled; it does not need a sibling `drives` checkout.

From a source checkout, `cargo +nightly check --lib` checks the default Rust library. The tests under `tests/` provide finite mock examples of contracts, not evidence of hardware timing. For workspace development, see [drives](https://github.com/Robot-Exp-Platform/drives).

## License

Apache-2.0; see [LICENSE](LICENSE).
