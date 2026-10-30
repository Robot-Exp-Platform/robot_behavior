# roplat_behavior

[English](README.md) | [简体中文](README_zh.md) | [Migrating from robot_behavior](MIGRATION.md)

> [!NOTE]
> **0.9 is not on crates.io yet.** These crates continue `robot_behavior` 0.8 under new names and depend on `roplat_msgs` from the roplat core, which is also unpublished. Until both are released, build from source (see [Working from source](#working-from-source)). The last `robot_behavior` release, 0.8.1, only points here.

`roplat_behavior` is a Rust interface library for writing robot applications against **capabilities**: move to a target, read a structured state, or calculate a command each control cycle. A hardware driver or simulator implements those capabilities; this repository supplies their shared contracts and the algorithms built on them.

| Crate | Contents |
|---|---|
| `roplat_behavior` | Capability contracts (`Robot`, motion spaces, control channels, model maps; gripper, force/torque, IMU and camera contracts), arm / base / legged state, motion limits as data (`MotionLimits`, `with_limits`), relative motion (`Relative<S>`), optional roplat adapters. |
| `roplat_behavior_ffi` | Python (`py`) and C++ (`cxx`) bindings of the contracts for driver packages used without the roplat runtime: the `py_*` / `cxx_*` macros and the data classes. |
| `roplat_world` | Contracts for the worlds robots run in: simulator backends and model selection, physics engines, renderers, world entities, the shared asset directory; `SimulationRhythm` with feature `roplat`. |
| `roplat_kinematics` | Denavit–Hartenberg forward kinematics, Jacobians and iterative inverse kinematics at the flange (`FLANGE`, `ik_solve_converged`); `dh_kinematics!` connects an arm to the model maps. `ArmKineCache::from_chain` takes a URDF chain from `roplat_description` instead of a DH table, `chain_ik` solves its inverse kinematics (any joint kinds, within URDF limits), `chain_ik_solutions` collects the distinct solutions passing a validity check (e.g. collision), nearest first, `UrdfKinematics` makes the chain a `ForwardKinematics` and `JacobianModel`, and `UrdfDynamics` makes a URDF tree a `DynamicsModel` for the `roplat_control` torque controllers. |
| `roplat_control` | Joint PD/PID, base-velocity PID, joint and Cartesian impedance, gravity compensation, computed torque, the target sources tracking controllers take (`target`), operator teleoperation (`cartesian_teleop_control`: deadzone, dead-man button, input timeout, acceleration limit), Cartesian servoing (`CartesianServo`: a tip twist to joint velocity or position commands, slowing only when approaching a singularity, stopping at joint limits, within joint velocity limits, with an optional proximity factor), following a timed tool-pose trajectory on the Cartesian pose channel (`CartesianTrajectoryFollower`, `cartesian_line`), real-time thread priority. |
| `roplat_trajectory` | Joint and Cartesian interpolation (linear, trapezoid, S-curve, fourth-order) and COPP time scaling on limits as data (`JointLimits`, `CartesianLimits`, `PathBounds`), depending only on `roplat_msgs` and copp; `plan_joint_path` finds collision-free joint-space waypoints (RRT-Connect, caller-supplied collision check), and `simplify_path` and `smooth_path` drop the waypoints it does not need and round its corners, checking every new segment, as OMPL's path simplifier does; `plan_cartesian_path` moves the tip in straight lines through given poses, solving each step with the caller's seeded inverse kinematics and stopping before an unreachable pose, a joint jump or an invalid segment. |
| `roplat_description` | One robot model read from URDF: kinematic tree, inertia, visual/collision geometry, joint limits, chain forward kinematics and Jacobian, the frames of every link in the tree and whole-tree inverse dynamics (gravity compensation), shared by kinematics, simulation, visualization and limit checks. Depends only on `roplat_msgs`. |
| `roplat_frames` | Frame tree of a robot cell: compile-time frame ids, static transforms from URDF and calibration, hand-eye calibration of a camera mount (`calibration::calibrate_hand_eye`, eye in hand or eye to hand), time-buffered joint and dynamic transforms looked up at a `Stamp` without allocating; optional `JointStateNode` (feature `roplat`); with feature `scene`, `scene::place_objects` moves planning-scene objects to their tracked frames at a stamp and `scene::add_sensed_points` adds a depth reading at the camera pose of its stamp. [Design (zh)](roplat_frames/docs/design_zh.md). |
| `roplat_safety` | Safety gates for control graphs: joint limits, rate limits, sample freshness, a cycle watchdog and a shared stop latch. A refused command fails the cycle, so the device receives `Break` and no command. [Design (zh)](roplat_safety/docs/design_zh.md). |
| `roplat_scene` | Collision checking for motion planning: a robot from its URDF (meshes as convex hulls), objects in the world and objects attached to links, allowed contacts (adjacent links from the start) and a padding. `PlanningScene` answers `plan_joint_path`'s validity check and gives the nearest pair; collision geometry by parry3d-f64, no native dependency. Also named configurations (`NamedPoses`) and tool placement (`EndEffector`), the parts of an SRDF that planning code needs, built in code. Sensed geometry too: `points::depth_to_points` back-projects a depth image through `CameraInfo` intrinsics, and `add_points` adds the points as occupied voxels the planners avoid. Concave objects such as bins and shelves keep their hollows: `add_concave_object` (a mesh file) and `add_trimesh` (triangles built in code) split them into convex parts by V-HACD instead of one hull. |
| `roplat_grasp` | Grasp candidates for boxes, cylinders and spheres for a parallel gripper, best first, and `plan_pick`: a free path to the pre-grasp pose, a straight approach and a straight lift with the object attached, trying candidates in order and saying why each failed. `GraspCheck` judges whether the closed fingers hold the object from the opening any gripper reports (held, empty, unexpected width, or disputed by the driver's own flag), and `carried_mass` confirms the load after the lift from a wrist force sensor. |
| `roplat_diagnostics` | Runtime observation: rate, period and latency probes with mean, p99 and worst values; `FrequencyCheck` and `LatencyCheck` judging them into OK / WARN / ERROR / STALE statuses (thresholds as ROS's `FrequencyStatus`); an `Updater` and a prefix-group `Aggregator`. All on a `roplat_msgs::Clock`, so simulation and replay time work. With `roplat`, pass-through `ProbeNode` / `LatencyNode` feed a check read outside the graph. |

Sample and geometry types (`Pose`, `JointSample`, `SpatialSample`, `LoadState`, `Twist`, …) come from [`roplat_msgs`](https://github.com/Robot-Exp-Platform/roplat) and are re-exported by `roplat_behavior`. They have one representation each and a fixed native layout, so the same values cross process and language boundaries. None of these crates depends on the roplat runtime unless you enable the `roplat` feature.

Use them when you want to reuse application or controller code across compatible backends, or implement a new driver in the same vocabulary. To connect to a robot, also choose its driver. To simulate physics, choose a physics backend such as [RsBullet](https://github.com/Robot-Exp-Platform/rsbullet).

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

The example uses `roplat_behavior` with the console reference robot `roplat_exrobot` from the [drives](https://github.com/Robot-Exp-Platform/drives) workspace. It uses nightly Rust features, and dependency builds need a C++ toolchain (on Windows, MSVC Build Tools and the Windows SDK):

```toml
[dependencies]
roplat_behavior = "0.9.0"
roplat_exrobot = "0.5.0"
```

```rust
use roplat_behavior::{
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

Run it with `cargo +nightly run`. You will see the reference robot's lifecycle and command messages, followed by `completed 3 control cycles` and shutdown. The final callback returns `true`, so its command is sent and the session finishes. The reference robot does not move or integrate a physical state.

To use a device next, replace `ExRobot` with the appropriate driver's constructor and follow that driver's connection and operating-mode setup. Matching Rust types make code reusable; they do not make robot limits, frames, timing, or controller gains interchangeable.

## Reading state

`Robot::State` is backend-specific. For portable arm-oriented code, use the driver's `Arm<N>` implementation and the structured state types:

- `JointState<N>` contains `meas`, `cmd`, and `des` views: measured feedback, accepted commands, and desired references.
- A `JointSample<N>` carries `q`, `dq`, `ddq`, `tau` and `dtau`, each read through an accessor such as `sample.q()`. `None` means that value was not supplied; it is never a fabricated zero. Producers use `JointSample::EMPTY.with_q(q)` or `sample.set_q(Some(q))`.
- `ArmState<N>` adds flange state and optional TCP, stiffness-frame, and load information. A native state and this portable view are not necessarily acquired through the same device operation.

Read the backend's state documentation for units, frames, field availability, and freshness. The presence of a value is not proof that a new sensor sample has arrived.

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

## Kinematics, controllers and trajectories

An arm with a Denavit–Hartenberg table implements `roplat_kinematics::ArmForwardKinematics<N>` (and `ArmInverseKinematics<N>` for the iterative solvers). `dh_kinematics!(MyArm, 6)` then implements the flange-pose and Jacobian maps, and `dh_kinematics!(MyArm, 6, inverse)` also the inverse map, so generic code can use `ForwardKinematics`, `JacobianModel` and `InverseKinematics` from `roplat_behavior`.

`roplat_kinematics` answers "which joint positions reach this pose" and connects DH and URDF models to the contract's model maps; `roplat_description` answers "what is the model at these joint positions" (frames, Jacobians, dynamics, limits) without the contract.

`roplat_control` returns ordinary closures for `control_with`. `roplat_trajectory` plans on `JointLimits` and the control period; an unbounded limit is `f64::INFINITY`, and the planners need finite velocity and acceleration bounds. Choose gains and model data for your backend; they do not make timing guarantees.

`JointTrajectoryFollower` plays timed `JointTrajectoryPoint`s on the joint-position channel. It reads time from a `roplat_msgs::Clock`, not from the cycle `Duration`, so the same follower runs on wall or simulation time; `JointTrajectoryFollower::cycle_timed` with `step_by(dt, …)` times it by the cycle durations a driver's callback reports. Path and goal tolerances refuse with a `TrackingError` rather than altering the command; with the `roplat` feature of `roplat_trajectory` the follower is a node, and `follower?` in a control domain ends the session with `Break`.

`roplat_control::CartesianTrajectoryFollower` does the same for tool poses on `CartesianPoseControl`: straight-line position and spherical orientation interpolation between timed `CartesianWaypoint`s, path and goal tolerances in metres and radians against the measured flange pose, and `cartesian_line` for a straight minimum-jerk move sampled at the control period. With the `roplat` feature of `roplat_control` it is a node too, like the joint follower.

## Features and Roplat integration

Default features are empty. The Rust motion, control, state, and model interfaces work without Roplat.

| Feature of `roplat_behavior` | Purpose |
|---|---|
| `roplat` | `ControlRhythm`, `AsyncControlRhythm`, and motion/model node adapters. |
| `conformance` | Contract checks for driver tests. |

Python and C++ bindings are `roplat_behavior_ffi` (features `py`, `cxx`); `SimulationRhythm` is `roplat_world` (feature `roplat`).

For a Roplat application, add `features = ["roplat"]` and depend on the matching `roplat = "0.5"`. Build the application graph with `#[roplat::system]`; choose `ControlRhythm` for a blocking backend or `AsyncControlRhythm` for a native async backend. The adapters connect the same observations and commands to a graph; they do not add capabilities missing from the device driver.

The robot stays owned by its creating scope. Nodes and rhythms receive a mutable reference through their separate `Resources` channel; business Input and Output never need to carry it.

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

The creating scope manages node and robot lifecycle. `MotionNode` declares `Resources = (R,)`, `Input = S::Target`, and `Output = RobotResult<()>`; both control rhythms declare `Resources = (R,)` with `Input = Output = ()`. The borrow lasts for one process or complete drive, including failure and stop paths. The `?` on `reach.output` explicitly promotes its business error to a System failure. See the [adapter implementation](roplat_behavior/src/roplat) and [complete System examples](roplat_behavior/tests/control_rhythm.rs).

## Combining robot models and simulator backends

`RobotDescription::SIMULATION_MODELS` can offer different models of the same
device. A loader implementing `SimulationBackend` declares its identifier and
accepted formats; `select_robot_model::<R>()` selects the matching asset before
loading it. Existing descriptions that only provide `URDF` remain supported by
loaders accepting URDF.

```rust
use roplat_behavior::{ModelFormat, RobotDescription, SimulationModel};

struct ArmModels;
impl RobotDescription for ArmModels {
    const SIMULATION_MODELS: &'static [SimulationModel] = &[
        SimulationModel::new(ModelFormat::Urdf, "arm/robot.urdf"),
        SimulationModel::new(ModelFormat::Mjcf, "arm/scene.xml").for_backend("mujoco"),
    ];
}
```

Selection prefers a backend-specific model, then a portable declaration, then
legacy `URDF`. Within a tier, backend format preference comes before declaration
order. The returned `SelectedSimulationModel` records the source; an unsupported
combination returns `ModelSelectionError` naming the backend and formats. Paths
follow that backend's asset/search-path rules. A selected file's loading failure
must be reported, rather than silently choosing different physics.

This is model selection, not automatic conversion or simulator implementation.
The backend still validates the model and provides its actual joint, control,
sensor, and scene capabilities. Platform/build availability belongs to each
backend's installation contract; accepting MJCF does not imply that MuJoCo is
installed or supported on the current machine. `PhysicsEngine` continues to own
stepping separately from the robot's existing motion and control traits. The
[selection tests](roplat_world/tests/simulation_models.rs) demonstrate compatible and
incompatible device/backend combinations without starting a simulator.

The optional `roplat_world::SimulationRhythm<E>` owns any compatible `PhysicsEngine`
and drives finite unit ticks in a real `#[roplat::system]` graph. Use
`SimulationRhythm::logical(engine).with_max_steps(n)` for accelerated execution,
or `SimulationRhythm::new(engine, period)` for wall-clock pacing. Physics dt
belongs to the engine; pacing never changes it. Budgets reset per drive without
resetting the world. A step error retains its source, stops the domain, and
prevents a downstream tick for the failed call. Cooperative stopping cancels
idle waits but waits for an active domain to return its owned nodes.

This shared adapter depends on no Bullet or MuJoCo package. Its Tokio dependency
is enabled only with the `roplat` feature; default behavior builds remain free
of that framework/runtime dependency. The backend retains responsibility for
state freshness, command ordering, physical semantics and resource cleanup.
The adapter does not reset an engine on entry or call its consuming `shutdown`
method on exit; use the backend's RAII cleanup or recover it with `into_engine()`.

## Going further

- [Motion and trajectory APIs](roplat_behavior/src/robot/motion.rs), [control contracts](roplat_behavior/src/robot/control.rs) and [native async contracts](roplat_behavior/src/robot/async_control.rs).
- [State types](roplat_behavior/src/robot/state.rs) and [model mappings](roplat_behavior/src/robot/model.rs).
- [Kinematics](roplat_kinematics/src), [controllers](roplat_control/src) and [trajectories](roplat_trajectory/src).
- [Executable contract examples](roplat_behavior/tests/control_flow.rs), [async examples](roplat_behavior/tests/async_control.rs), and [foreign-interface examples](roplat_behavior/examples).
- [What changed from robot_behavior 0.8](MIGRATION.md).

Driver authors can import `roplat_behavior::driver::*`; applications can import `roplat_behavior::behavior::*`. Implement the device's lifecycle and state in `Robot`, then each supported motion/control/model capability. Default lifecycle hooks are no-ops and some unimplemented operations return errors; inheriting a default is not an implementation of device behavior.

## Working from source

This is a Cargo workspace of the crates above; each crate declares its own version. Until `roplat_msgs` is published, the root manifest patches `roplat` and `roplat_msgs` to a sibling checkout of the roplat core, so clone both side by side:

```text
Robot-Exp-Platform/
├── roplat/
└── roplat_behavior/
```

`cargo +nightly test --workspace` runs the default tests; add `--features roplat_behavior/roplat` for the adapter tests. The tests provide finite mock examples of contracts, not evidence of hardware timing. For driver integration, see [drives](https://github.com/Robot-Exp-Platform/drives).

## License

Apache-2.0; see [LICENSE](LICENSE).
