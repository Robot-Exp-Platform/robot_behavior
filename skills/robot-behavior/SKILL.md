---
name: robot-behavior
description: "Use when working with the robot_behavior Rust crate or downstream robot drivers such as franka-rust, libjaka-rs, libhans-rs, libaubo-rs, rsbullet, or roplat adapters. Covers two user paths: writing application code that consumes an existing robot driver, and implementing or reviewing a new driver against robot_behavior traits including Robot, Motion, MoveTo, MoveTraj, ControlWith, ControlObservation, typed spaces, state models, controller closures, model maps, Roplat integration, examples, and release checks."
---

# Robot Behavior

Use this skill to keep `robot_behavior` work aligned with the current crate design. Treat the crate as a behavior contract layer: it defines shared robot capabilities, state shapes, typed command spaces, control channels, model maps, and Roplat adapters. It is not a vendor SDK, not a full planner, and not a place to hide driver-specific semantics behind fake generic traits.

## First Pass

Before editing or reviewing code, read the current source instead of trusting memory. The API is still evolving.

- Read `robot_behavior/src/lib.rs` for public preludes: `behavior`, `driver`, and `controller`.
- Read `robot_behavior/src/robot/mod.rs`, `motion.rs`, `control.rs`, `observe.rs`, `state.rs`, `spaces.rs`, `arm.rs`, `category.rs`, and `model.rs` before changing traits or downstream impls.
- Read `robot_behavior/src/utils/controller/*` and `src/utils/trajectory.rs` before changing controllers or path generation.
- Read `robot_behavior/src/roplat/*` only when the task mentions Roplat rhythms or nodes.
- Read `robot_behavior/EXAMPLES.md` when adding or auditing driver examples.
- Let current source win over this skill if there is drift.

## Core Model

Keep these concepts separate:

- `Robot` is the root lifecycle and state trait. It owns `type State`, `CONTROL_PERIOD`, `version`, and `read_state`.
- Capability traits describe what a robot can do: `Joints<N>`, `EndPoint`, `Arm<N>`, `MobileBase`, `Quadruped<N>`, `Humanoid<N>`, `MoveTo<S>`, `MoveTraj<S>`, `ControlWith<S>`, and model traits.
- Space marker types give arrays semantic meaning. `[f64; N]` is not enough; pair it with `JointSpace<N>`, `TorqueControl<N>`, `WholeBodyJointSpace<N>`, etc.
- State is split into measured, commanded, and desired views with `StateView<T> { meas, cmd, des }`.
- Controller helpers return plain closures. Do not add controller structs just to name a single closure.
- `behavior::*` is for application users. `driver::*` is for driver authors. Avoid importing `driver::*` in examples or app code unless implementing traits.

## Application Users

Use this path when the user has a driver and wants to move, control, observe, or test a robot.

### Imports And Bounds

Prefer:

```rust
use robot_behavior::behavior::*;
```

Write trait bounds against capabilities, not concrete drivers:

```rust
fn home<R>(robot: &mut R) -> RobotResult<()>
where
    R: MoveTo<JointSpace<7>>,
{
    robot.move_to::<JointSpace<7>>([0.0; 7])
}
```

Use the concrete driver type only for connection, vendor-specific configuration, and hardware-only APIs.

### Lifecycle And State

Use `Robot` methods for lifecycle and state:

- `init`, `enable`, `disable`, `shutdown`, `stop`, `reset`, `pause`, `resume`
- `is_moving` and `waiting_for_finish` when a driver gives them real semantics
- `read_state` for the driver-native full state
- `Arm::state` for normalized `ArmState<N>` when the robot is an arm

Expect `RobotResult<T>` and propagate failures with `?`.

### Motion

Use typed spaces to select the meaning of the target:

```rust
robot.move_to::<JointSpace<7>>(joint_target)?;
robot.move_to::<FlangeSpace>(pose)?;
robot.move_to::<TcpSpace>(pose)?;
```

Current semantics:

- `move_to` is blocking: it returns after the target motion completes or fails.
- `move_to_async` returns an inert future. Use it only when the driver has a real async backend.
- `move_to_sync` is deprecated; do not introduce new uses.
- `move_traj`, `move_path`, and `move_waypoints` are trajectory entry points. If waiting semantics matter, prefer the available `_sync` wrappers or verify the driver behavior.
- `MotionFile` loads JSON `Vec<S::Target>` for `move_traj_from_file` and `move_waypoints_from_file`.

Common motion spaces:

- `JointSpace<N>`: joint position target `[f64; N]`
- `FlangeSpace` / `TcpSpace`: `Pose`
- `BasePoseSpace`, `BaseVelocitySpace`: mobile/floating base commands
- `WholeBodyJointSpace<N>`, `WholeBodyVelocitySpace<N>`, `WholeBodyTorqueSpace<N>`
- `GaitSpace`, `FootSpace<LEG>`, `HandSpace<HAND>`
- `Relative<S>` and `Inertial<S>` wrappers when a driver supports those frame semantics

### Realtime Control

Use `Control::control_with::<S, _>(closure)` for a blocking control session:

```rust
robot.control_with::<TorqueControl<7>, _>(|state, dt| {
    let q = state.meas.q.unwrap_or([0.0; 7]);
    let done = dt.as_secs_f64() > 0.0 && q.iter().all(|v| v.is_finite());
    ([0.0; 7], done)
})?;
```

The closure signature is:

```rust
FnMut(S::Obs, Duration) -> (S::Command, bool)
```

The `bool` is `done`. Return `true` to end the session. Keep commands bounded and make every example terminate.

Built-in control channels:

- `JointPositionControl<N>`: observes `JointState<N>`, commands `[f64; N]`
- `JointVelocityControl<N>`: observes `JointState<N>`, commands `[f64; N]`
- `TorqueControl<N>`: observes `JointState<N>`, commands `[f64; N]`
- `ArmTorqueControl<N>`: observes `ArmState<N>`, commands `[f64; N]`
- `CartesianPoseControl<N>`: observes `ArmState<N>`, commands `Pose`
- `CartesianVelocityControl<N>`: observes `ArmState<N>`, commands `[f64; 6]`
- `BaseVelocityControl`: observes `BaseState`, commands `[f64; 6]`
- `BalanceControl`: observes `BaseState`, commands `[f64; 6]`

Use `control_with_async` only when the per-cycle controller closure itself needs `async`. The session still blocks until completion; the default implementation runs one async future per control cycle through the blocking `control_with` loop.

### Controllers And Target Sources

Import reusable controllers with:

```rust
use robot_behavior::controller::*;
```

Use fixed-target helpers only for control tasks. For normal fixed-position movement, prefer `move_to`.

Useful controller families:

- `joint_pd_control`, `joint_pd_tracking_control`, `joint_traj_pd_control`
- `joint_pid_control`, `joint_pid_tracking_control`, `joint_traj_pid_control`
- `base_velocity_pid_control`, `base_velocity_pid_tracking_control`
- `joint_impedance_control`, `joint_traj_impedance_control`, `joint_impedance_tracking_control`, `joint_impedance_session`
- `cartesian_impedance_control`, `cartesian_traj_impedance_control`, `cartesian_impedance_tracking_control`, `cartesian_impedance_session`
- `gravity_compensation_control`
- `computed_torque_control`, `computed_torque_tracking_control`

Use `utils::path_generate::*` when you need target-source closures:

- `constant_joint_target_fn`, `joint_traj_target_fn`, `joint_path_target_fn`
- `constant_pose_target_fn`, `pose_traj_target_fn`, `pose_path_target_fn`

Use `utils::trajectory::*` when you need COPP-backed joint planning or position-control closure generation:

- `plan_waypoints_traj_via_copp`
- `plan_path_traj_via_copp`
- `joint_traj_position_control`
- `copp_waypoints_joint_position_control`
- `copp_path_joint_position_control`

### Observation

Use `ControlObservation` to register robot-level observers around a control session:

```rust
robot
    .before(|state, dt| {
        let _ = (state, dt);
    })
    .after(|state, dt| {
        let _ = (state, dt);
    })
    .control_with::<JointPositionControl<7>, _>(controller)?;
```

Observers receive `&Robot::State`, not `S::Obs`. Keep them lightweight; do not put heavy logging, blocking I/O, or allocation in realtime paths unless the driver documents that it is safe.

### Models

Use model traits only when the driver/model implements them:

- `ForwardKinematics<N>`: `JointSpace<N> -> FlangeSpace`
- `InverseKinematics<N>`: `FlangeSpace -> JointSpace<N>`
- `JacobianModel<N>`: `JointSpace<N> -> JacobianSpace<N>`
- `DynamicsModel<N>`: mass, Coriolis/centrifugal, and gravity maps

Use `SpaceMap<From, To>` / `TypedSpaceMap<From, To>` for generic mappings instead of inventing ad hoc names.

### Examples

When adding application examples, follow `EXAMPLES.md` naming and scope. Moving examples should state the expected physical effect clearly enough for operator review.

Do not mix Roplat examples into native driver example sets unless the task explicitly asks for Roplat integration.

## Driver Authors

Use this path when implementing a real robot, simulator wrapper, or native SDK binding.

### Imports And Shape

Prefer:

```rust
use robot_behavior::driver::*;
```

Design the driver so one robot object owns the exclusive control/communication resource. Do not let state reads, motion commands, and realtime control silently create competing sockets, sessions, or hardware handles.

Use vendor-native APIs for one-off hardware features such as collision thresholds, grippers, IO, brakes, or firmware settings. Add a shared trait only after the behavior repeats across drivers.

### Implement In This Order

1. `Robot`
2. `Joints<N>` and/or `EndPoint`
3. normalized state conversion (`JointState<N>`, `ArmState<N>`, `BaseState`, etc.)
4. capability bundle (`Arm<N>`, `MobileBase`, `Quadruped<N>`, or `Humanoid<N>`) if appropriate
5. `MoveTo<S>` for each supported motion space
6. `MoveTraj<S>` for each supported trajectory space
7. `ControlWith<S>` for each supported realtime control channel
8. `ControlObservation` if the backend can run observer hooks during control sessions
9. model maps and Roplat adapters only when needed

### Robot And State

Choose `Robot::State` as the full native state or a driver-owned normalized full state. Preserve information instead of squeezing it prematurely into a control-space observation.

Implement:

- `const CONTROL_PERIOD: f64`
- `fn version() -> String`
- `fn read_state(&mut self) -> RobotResult<Self::State>`

Override lifecycle methods only when the backend has a real operation for them. Default no-op lifecycle methods are acceptable for unsupported lifecycle hooks.

Use SI units at the `robot_behavior` boundary:

- joint position: radians
- joint velocity: rad/s
- acceleration: rad/s^2
- torque: N*m
- Cartesian position: meters
- spatial velocity: `[vx, vy, vz, wx, wy, wz]`
- spatial wrench: `[fx, fy, fz, tx, ty, tz]`

Convert vendor units at the driver boundary.

### Limits And Capability Bundles

Implement `Joints<N>` for fixed-DoF articulated robots. Declare `JOINT_MIN` and `JOINT_MAX`; add velocity, acceleration, jerk, torque, and torque-rate bounds when known.

Implement `EndPoint` for Cartesian-capable arms or end effectors. Override only meaningful bounds.

Implement `Arm<N>` when the robot is a serial arm and can support arm-level state, load configuration, joint/flange accessors, and limit tuning APIs. Do not make `Arm` the root model for non-arm robots; use category traits or smaller capabilities.

For legged or whole-body robots, use `MobileBase`, `Quadruped<N>`, `Humanoid<N>`, whole-body spaces, gait spaces, foot/hand spaces, and shared joint control channels.

### Motion Implementation

Implement `MoveTo<S>` per supported space:

```rust
impl MoveTo<JointSpace<7>> for MyRobot {
    fn move_to(&mut self, target: [f64; 7]) -> RobotResult<()> {
        // blocking target motion
        Ok(())
    }
}
```

Rules:

- Make `move_to` block until completion or error.
- Funnel high-level spaces downward when that is natural, e.g. `TcpSpace -> FlangeSpace -> JointSpace -> hardware`.
- Implement `move_to_async` only with a real async backend. The returned future should do no work until polled.
- If async is unsupported, rely on the default unsupported error.

Implement `MoveTraj<S>` when a backend can consume trajectories. Today the trait requires `move_traj`, `move_path`, and `move_waypoints`; if path or waypoint planning is unsupported, implement those methods to return `RobotException::UnprocessableInstructionError` with a clear message.

Use `utils::trajectory` or a vendor planner only when the driver actually wires it in. Do not fake interpolation silently.

### ControlWith Implementation

Implement `ControlWith<S>` one control channel at a time. Each impl fixes an observation type and command type through `S: ControlSpace<Self>`.

Required behavior:

- Run a blocking control session.
- Call the user closure once per control period with fresh observation and elapsed `Duration`.
- Send or apply the returned command for the current cycle.
- Stop and return `Ok(())` when `done = true`, or return the driver error.
- Do not store a scoped `control_with` closure after the call returns.
- Do not open a separate backend that conflicts with `read_state` or motion.

Implement `hold_command(obs)` as a continuity fallback for that channel. Do not name it or treat it as a physical safety system. Safety belongs to command filtering, vendor safety configuration, limits, collision behavior, and low-level driver checks.

When supporting `ControlObservation`, store `ControlObserver<Self::State>` on the robot and run `before` / `after` hooks around each cycle with the full `Robot::State`.

Override `control_with_async` only when the backend can preserve the same blocking session semantics more directly than the default per-cycle `block_on` adapter.

### Simulators

Do not force a step-driven simulator into normal `ControlWith` if that would lie about semantics. A simulator that controls many entities often needs registered controllers and a simulation-level `step`.

Acceptable designs:

- Implement normal `ControlWith` only for a blocking single-robot session.
- Provide simulator-native `control_with` / registration APIs for queued or registered semantics.
- Provide simulator-specific Roplat rhythms that distribute state to registered robot controllers, collect commands, and step the world.

Keep the type-level control-space style where useful, but do not claim standard `ControlWith` if the session is not blocking and does not own the robot resource for its duration.

### Model Maps

Use `SpaceMap<From, To>` for typed model mappings:

```rust
impl SpaceMap<JointSpace<7>, FlangeSpace> for MyModel {
    type Input = [f64; 7];
    type Output = Pose;

    fn map(&self, input: Self::Input) -> RobotResult<Self::Output> {
        // FK
        Ok(Pose::default())
    }
}
```

Use helper traits when the model has reliable data:

- `ArmForwardKinematics<N>` from DH data
- `ArmInverseKinematics<N>` for iterative or analytic IK
- `ForwardKinematics<N>` / `InverseKinematics<N>` through `SpaceMap`
- `JacobianModel<N>`
- `DynamicsModel<N>`

Do not add fake FK, IK, Jacobian, or dynamics just to satisfy an example. Omit the trait or return a clear unsupported error.

### FFI And Bindings

Treat Rust as the source of truth. Update `.pyi`, `.hpp`, C/C++/Python bindings, and feature-gated FFI only after the Rust API and downstream driver semantics are settled.

If the task is not about FFI, avoid touching FFI files while changing core traits or driver examples.

## Roplat Integration

Use `robot_behavior::roplat::*` only for Roplat tasks.

Current generic adapters:

- `MotionNode<R, S>`: input `(RobotResult<R>, S::Target)`, output `RobotResult<R>`, runs one blocking `MoveTo<S>` and returns the robot resource.
- `SpaceMapNode<M, From, To>`: wraps a typed `SpaceMap`.
- `SafetyNode<F, Command>`: transforms `(Command, bool)` frames.
- `ControlRhythm<R, S>`: input `RobotResult<R>`, output `RobotResult<R>`, yields `(S::Obs, Duration)` and accepts `(S::Command, bool)` through the Roplat operation domain.

Do not add generic message structs when a tuple cleanly expresses the data. Do not add mpsc forwarding or background threads in generic adapters unless the upstream Roplat contract requires it.

## Validation

For `robot_behavior` itself:

```powershell
cd robot_behavior
cargo fmt
cargo check --all-targets --offline
cargo package --allow-dirty --no-verify --list --offline
```

For downstream drivers from the workspace root:

```powershell
cargo fmt
cargo check -p <crate> --all-targets --offline
cargo package -p <crate> --allow-dirty --no-verify --list --offline
```

For skills:

```powershell
python C:\Users\yanji\.codex\skills\.system\skill-creator\scripts\quick_validate.py robot_behavior\skills\robot-behavior
```

Check `git status` for the touched subcrate. Do not commit the root `drives` repository unless explicitly asked.

## Common Mistakes

- Do not use `robot_behavior::driver::*` in application examples; it can bring implementation traits into scope and create method ambiguity.
- Do not wrap a blocking backend and call it native async.
- Do not store non-static scoped `control_with` closures in another thread.
- Do not implement unsupported traits with no-op behavior that looks successful.
- Do not duplicate reusable controller logic inside a driver when `robot_behavior::controller` already provides the closure.
- Do not put vendor-only APIs into shared traits after one implementation.
- Do not use `Arm` as the conceptual root for every robot form.
- Do not hide simulator registered/queued semantics behind standard blocking `ControlWith`.
