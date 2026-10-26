# robot_behavior (renamed: roplat_behavior)

[English](README.md) | [简体中文](README_zh.md)

> [!IMPORTANT]
> **`robot_behavior` is renamed `roplat_behavior` and will not be updated again.** 0.8.1 is the final release: its API is identical to 0.8.0, and only this README, the crate documentation and the metadata changed. New development continues in [Robot-Exp-Platform/roplat_behavior](https://github.com/Robot-Exp-Platform/roplat_behavior) (this repository, renamed).
>
> As of 2026-10-26, `roplat_behavior` 0.9 and the `roplat_msgs` crate it depends on are **not yet on crates.io**. Keep using `robot_behavior = "0.8"` until they are published, or build them from source; the change is reviewed in [roplat_behavior#17](https://github.com/Robot-Exp-Platform/roplat_behavior/pull/17).

The rest of this page is the migration guide from 0.8 to 0.9.

`robot_behavior` is renamed `roplat_behavior` and split into four crates with one version. The contracts and their semantics are unchanged: typed motion spaces and control channels, `StateView`'s measured / accepted-command / desired views, `ControlStep`'s termination, the split between blocking sessions and native async sessions, and `Resources = (R,)` borrowing in the roplat adapters. What changes is where things live, and the sample and pose types, which now come from `roplat_msgs`.

As of 2026-10-26 neither `roplat_behavior` 0.9 nor `roplat_msgs` is on crates.io; build from source until they are released.

## Dependencies

```toml
[dependencies]
# robot_behavior = "0.8"
roplat_behavior = "0.9.0"
roplat_kinematics = "0.9.0"   # DH kinematics, IK
roplat_control = "0.9.0"      # PID, impedance, computed torque
roplat_trajectory = "0.9.0"   # interpolation, COPP time scaling
```

Replace `robot_behavior::` with `roplat_behavior::` in paths. Features are unchanged (`roplat`, `ffi`, `to_c`, `to_cxx`, `to_py`), and `roplat` now asks for `roplat = "0.5.1"`.

## Where items moved

| 0.8 path | 0.9 path |
|---|---|
| `robot_behavior::{Robot, MoveTo, ControlWith, JointSpace, Arm, SpaceMap, …}` | `roplat_behavior::…` (unchanged names) |
| `Pose`, `JointSample`, `JointState`, `SpatialSample`, `ContactSample`, `StateView`, `LoadState` | `roplat_msgs`, re-exported by `roplat_behavior` |
| `DhParam`, `dh_param!`, `mdh_param!`, `ArmKineCache`, `ArmForwardKinematics`, `ArmInverseKinematics`, `IKMethod`, `CommonStop`, `AnalyticFamily`, `Link`, `Joint`, `JointType`, `ArmDynamics`, `combine_array` | `roplat_kinematics` |
| `robot_behavior::controller::*` and the root re-exports (`joint_pd_control`, `joint_impedance_control`, `gravity_compensation_control`, …), `set_realtime_priority`, `is_hardware_realtime`, `RealtimeBehavior` | `roplat_control` |
| `robot_behavior::utils::path_generate::*` | `roplat_trajectory::path_generate::*` |
| `plan_waypoints_traj_via_copp`, `plan_path_traj_via_copp`, `plan_s_t_via_copp`, `plan_cartesian_path_traj_via_time_scaling`, `CartesianTimeScalingOptions` | `roplat_trajectory` |
| `Iso3`, `JVec`, `JMat`, `Jaco` | `roplat_behavior` (unchanged) |
| nalgebra aliases `Twist`, `Wrench` (6-vectors) | `roplat_kinematics::SpatialVec`; the names `Twist` / `Wrench` are now the `roplat_msgs` structs |
| `OverrideOnce`, `RobotBound`, `ArmBound`, `limit`, `homo_to_isometry`, `to_radians_array`, … | `roplat_behavior` (unchanged) |

Removed: the uncompiled `src/robot_old` and `src/utils/inverse.rs` sources.

## Pose

`Pose` is one struct, `{ position: Vector3, orientation: Quaternion }` with the quaternion stored `x, y, z, w`, instead of five enum variants. The accessors keep their names: `euler()`, `quat()`, `quat_array()`, `homo()`, `axis_angle()`, `position()`.

| 0.8 | 0.9 |
|---|---|
| `Pose::Euler(p, rpy)` | `Pose::from_euler(p, rpy)` |
| `Pose::Quat(isometry)` | `Pose::from_isometry(isometry)` or `isometry.into()` |
| `Pose::Homo(m)` | `Pose::from_homo(m)` |
| `Pose::AxisAngle(p, axis, angle)` | `Pose::from_axis_angle(p, axis, angle)` |
| `Pose::Position(p)` | `Pose::from_position(p)` |
| `match pose { Pose::Homo(m) => … }` | `pose.homo()` |
| `pose_a * pose_b` | unchanged (composition, now always through quaternions) |
| `pose_a / pose_b` | removed; it summed `2^(a - b)` over the position and had no callers |

The array `From` conversions keep their meaning (`[f64; 3]` position, `[f64; 6]` Euler, `[f64; 7]` with its zero / unit / axis-angle rule, `[f64; 16]` homogeneous, slices and `Vec`). `Vec<f64>::from(pose)` always yields `[x, y, z, qx, qy, qz, qw]`; in 0.8 the quaternion variant produced only six numbers.

Serialized poses: JSON written by 0.8 (`{"Euler": …}`, `{"Quat": …}`, `{"Homo": …}`, `{"AxisAngle": …}`, `{"Position": …}`) still deserializes. New output is `{"position": …, "orientation": …}`.

The C, C++ and Python bindings return poses in the quaternion form; their constructors still accept every form. A C/C++ `Quat` input is now read as a quaternion directly instead of through the seven-number heuristic, which could misread a quaternion whose norm was not exactly 1.

## Samples

Fields that may be absent are tracked by a validity mask behind accessors, so the samples have a fixed layout.

| 0.8 | 0.9 |
|---|---|
| `sample.q` (`Option<[f64; N]>`) | `sample.q()` |
| `JointSample { q: Some(q), dq: Some(dq), ..Default::default() }` | `JointSample::EMPTY.with_q(q).with_dq(dq)` |
| `sample.q = Some(q)` / `= None` | `sample.set_q(Some(q))` / `set_q(None)` |
| `spatial.pose`, `spatial.vel`, `spatial.acc`, `spatial.wrench` | `spatial.pose()`, `spatial.twist()`, `spatial.accel()`, `spatial.wrench()` returning `Pose`, `Twist`, `Accel`, `Wrench` |
| `vel: Some([f64; 6])` | `.with_twist(array.into())`; read back with `.twist().map(<[f64; 6]>::from)` |
| `contact.on`, `contact.f`, `contact.p`, `contact.n` | `contact.is_active()`, `force()`, `point()`, `normal()` (as `Vector3`) |
| `LoadState { m, x, i }` | `LoadState { mass, center_of_mass, inertia }` |

Defaults are unchanged: `JointSample::default()` has nothing present; `SpatialSample::default()` and `ContactSample::default()` have every field present with identity / zero values, as before. `SpatialSample::EMPTY` and `ContactSample::EMPTY` have nothing present.

`JacobianModel::twist` returns the `Twist` struct instead of a nalgebra 6-vector.

## Kinematics

A type implementing `ArmForwardKinematics<N>` no longer gets the `SpaceMap` flange-pose and Jacobian maps automatically; the blanket impls would have tied the contract crate to the DH implementation. Add one line next to the DH table:

```rust
roplat_kinematics::dh_kinematics!(MyArm, 6);           // FK + Jacobian maps
roplat_kinematics::dh_kinematics!(MyArm, 6, inverse);  // also the IK map
```

Calls such as `<MyArm as ArmInverseKinematics<6>>::ik_solve` work as before.

## Python and C++

The Python module is `roplat_behavior` (was `robot_behavior`); its classes and fields are unchanged, including `LoadState(m, x, i)`. The C++ header is `roplat_behavior.hpp`, namespace `roplat_behavior`.

## Nightly

`roplat_behavior` no longer needs `generic_const_exprs`; it uses only `async_trait_bounds`. `roplat_kinematics` still uses `generic_const_exprs` for its `N + 1` frame cache.
