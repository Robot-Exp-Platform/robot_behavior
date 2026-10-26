# 从 robot_behavior 0.8 迁移到 roplat_behavior 0.9

[English](MIGRATION.md)

`robot_behavior` 改名为 `roplat_behavior`，并拆成共用一个版本的四个包。契约及其语义不变：类型化的运动空间与控制通道，`StateView` 的测量 / 已接受指令 / 期望三种视图，`ControlStep` 的收尾，阻塞会话与原生异步会话的区分，以及 roplat 适配中 `Resources = (R,)` 的借用。变化在于各部分的位置，以及采样和位姿类型改由 `roplat_msgs` 提供。

截至 2026-10-26，`roplat_behavior` 0.9 与 `roplat_msgs` 都还没有发布到 crates.io；发布之前请从源码构建。

## 依赖

```toml
[dependencies]
# robot_behavior = "0.8"
roplat_behavior = "0.9.0"
roplat_kinematics = "0.9.0"   # DH 运动学、逆解
roplat_control = "0.9.0"      # PID、阻抗、计算力矩
roplat_trajectory = "0.9.0"   # 插值、COPP 时间缩放
```

路径中的 `robot_behavior::` 改为 `roplat_behavior::`。feature 不变（`roplat`、`ffi`、`to_c`、`to_cxx`、`to_py`），`roplat` 现在要求 `roplat = "0.6.0-rc.1"`。

## 各项的新位置

| 0.8 路径 | 0.9 路径 |
|---|---|
| `robot_behavior::{Robot, MoveTo, ControlWith, JointSpace, Arm, SpaceMap, …}` | `roplat_behavior::…`（名字不变） |
| `Pose`、`JointSample`、`JointState`、`SpatialSample`、`ContactSample`、`StateView`、`LoadState` | `roplat_msgs`，由 `roplat_behavior` 重新导出 |
| `DhParam`、`dh_param!`、`mdh_param!`、`ArmKineCache`、`ArmForwardKinematics`、`ArmInverseKinematics`、`IKMethod`、`CommonStop`、`AnalyticFamily`、`Link`、`Joint`、`JointType`、`ArmDynamics`、`combine_array` | `roplat_kinematics` |
| `robot_behavior::controller::*` 及根部的重新导出（`joint_pd_control`、`joint_impedance_control`、`gravity_compensation_control` 等）、`set_realtime_priority`、`is_hardware_realtime`、`RealtimeBehavior` | `roplat_control` |
| `robot_behavior::utils::path_generate::*` | `roplat_trajectory::path_generate::*` |
| `plan_waypoints_traj_via_copp`、`plan_path_traj_via_copp`、`plan_s_t_via_copp`、`plan_cartesian_path_traj_via_time_scaling`、`CartesianTimeScalingOptions` | `roplat_trajectory` |
| `Iso3`、`JVec`、`JMat`、`Jaco` | `roplat_behavior`（不变） |
| nalgebra 别名 `Twist`、`Wrench`（六维向量） | `roplat_kinematics::SpatialVec`；`Twist` / `Wrench` 这两个名字现在指 `roplat_msgs` 的结构体 |
| `OverrideOnce`、`RobotBound`、`ArmBound`、`limit`、`homo_to_isometry`、`to_radians_array` 等 | `roplat_behavior`（不变） |

删除了从未参与编译的 `src/robot_old` 与 `src/utils/inverse.rs`。

## 位姿

`Pose` 从五个 enum 变体改为一个结构体 `{ position: Vector3, orientation: Quaternion }`，四元数按 `x, y, z, w` 存储。访问方法名字不变：`euler()`、`quat()`、`quat_array()`、`homo()`、`axis_angle()`、`position()`。

| 0.8 | 0.9 |
|---|---|
| `Pose::Euler(p, rpy)` | `Pose::from_euler(p, rpy)` |
| `Pose::Quat(isometry)` | `Pose::from_isometry(isometry)` 或 `isometry.into()` |
| `Pose::Homo(m)` | `Pose::from_homo(m)` |
| `Pose::AxisAngle(p, axis, angle)` | `Pose::from_axis_angle(p, axis, angle)` |
| `Pose::Position(p)` | `Pose::from_position(p)` |
| `match pose { Pose::Homo(m) => … }` | `pose.homo()` |
| `pose_a * pose_b` | 不变（复合，现在一律经四元数计算） |
| `pose_a / pose_b` | 删除；它对位置求 `2^(a - b)` 之和，且没有调用方 |

数组形式的 `From` 转换含义不变（`[f64; 3]` 位置，`[f64; 6]` 欧拉角，`[f64; 7]` 仍按零向量 / 单位四元数 / 轴角规则区分，`[f64; 16]` 齐次矩阵，以及切片和 `Vec`）。`Vec<f64>::from(pose)` 一律得到 `[x, y, z, qx, qy, qz, qw]`；0.8 的四元数变体只给出 6 个数。

序列化：0.8 写出的 JSON（`{"Euler": …}`、`{"Quat": …}`、`{"Homo": …}`、`{"AxisAngle": …}`、`{"Position": …}`）仍可读取，新输出为 `{"position": …, "orientation": …}`。

C、C++、Python 绑定以四元数形式返回位姿，构造时仍接受所有形式。C/C++ 的 `Quat` 输入现在直接按四元数读取，不再经过 7 个数的判别规则；旧规则在范数不恰好为 1 时可能把四元数误读成轴角。

## 采样

可能缺失的字段改为位掩码加访问器，采样因此有固定布局。

| 0.8 | 0.9 |
|---|---|
| `sample.q`（`Option<[f64; N]>`） | `sample.q()` |
| `JointSample { q: Some(q), dq: Some(dq), ..Default::default() }` | `JointSample::EMPTY.with_q(q).with_dq(dq)` |
| `sample.q = Some(q)` / `= None` | `sample.set_q(Some(q))` / `set_q(None)` |
| `spatial.pose`、`spatial.vel`、`spatial.acc`、`spatial.wrench` | `spatial.pose()`、`spatial.twist()`、`spatial.accel()`、`spatial.wrench()`，返回 `Pose`、`Twist`、`Accel`、`Wrench` |
| `vel: Some([f64; 6])` | `.with_twist(array.into())`；读取用 `.twist().map(<[f64; 6]>::from)` |
| `contact.on`、`contact.f`、`contact.p`、`contact.n` | `contact.is_active()`、`force()`、`point()`、`normal()`（`Vector3`） |
| `LoadState { m, x, i }` | `LoadState { mass, center_of_mass, inertia }` |

默认值不变：`JointSample::default()` 不含任何字段；`SpatialSample::default()` 与 `ContactSample::default()` 与以前一样含全部字段，取单位位姿和零值。`SpatialSample::EMPTY` 与 `ContactSample::EMPTY` 不含任何字段。

`JacobianModel::twist` 返回 `Twist` 结构体，不再是 nalgebra 六维向量。

## 运动学

实现 `ArmForwardKinematics<N>` 的类型不再自动获得 `SpaceMap` 的法兰位姿与雅可比映射；这些 blanket impl 会把契约包绑在 DH 实现上。在 DH 参数表旁加一行：

```rust
roplat_kinematics::dh_kinematics!(MyArm, 6);           // 正运动学与雅可比映射
roplat_kinematics::dh_kinematics!(MyArm, 6, inverse);  // 同时实现逆解映射
```

`<MyArm as ArmInverseKinematics<6>>::ik_solve` 这类调用照旧可用。

## Python 与 C++

Python 模块名为 `roplat_behavior`（原为 `robot_behavior`）；类与字段不变，包括 `LoadState(m, x, i)`。C++ 头文件为 `roplat_behavior.hpp`，命名空间为 `roplat_behavior`。

## Nightly

`roplat_behavior` 不再需要 `generic_const_exprs`，只用 `async_trait_bounds`。`roplat_kinematics` 的 `N + 1` 帧缓存仍需要 `generic_const_exprs`。
