# roplat_kinematics

[English](README.md) | [简体中文](README_zh.md)

[`roplat_behavior`](../roplat_behavior/README_zh.md) 机械臂的运动学，来自 DH 参数表或 URDF 链，并接到契约的模型映射上。

属于 [roplat_behavior 工作区](../README_zh.md)。0.9 尚未发布到 crates.io，请按[从源码开发](../README_zh.md#从源码开发)从源码构建。

## 内容

- 以法兰为准的 DH 正运动学、雅可比与迭代逆运动学（`FLANGE`、`ik_solve_converged`）。
- `dh_kinematics!` 把 DH 机械臂接入模型映射。
- 从 URDF 读出的机械臂不需要 DH 表：
  - `ArmKineCache::from_chain` 用 [`roplat_description`](../roplat_description/README_zh.md) 的链构造同样的缓存。
  - `chain_ik` 在 URDF 限位内求其逆运动学，支持各类关节。
  - `chain_ik_solutions` 收集通过有效性检查（如碰撞检查）的不同解，按与参考位形的距离排序。
  - `UrdfKinematics` 把该链作为 `ForwardKinematics` 与 `JacobianModel`，与 `dh_kinematics!` 对 DH 机械臂所做的相同。
- `UrdfDynamics` 把 URDF 树作为 `DynamicsModel`，供 [`roplat_control`](../roplat_control/README_zh.md) 的力矩控制器使用。

## DH 机械臂

带 DH 参数表的机械臂实现 `ArmForwardKinematics<N>`（迭代求解器还需要 `ArmInverseKinematics<N>`）。`dh_kinematics!(MyArm, 6)` 随后实现法兰位姿与雅可比映射，`dh_kinematics!(MyArm, 6, inverse)` 还实现逆映射，泛型代码就能通过 `roplat_behavior` 的 `ForwardKinematics`、`JacobianModel`、`InverseKinematics` 使用它。

## 运动学还是描述？

`roplat_kinematics` 回答“要到达这个位姿，关节该取什么值”，并把 DH 与 URDF 两种模型接到契约的模型映射上；`roplat_description` 回答“在这组关节值下模型是什么样”（各坐标系、雅可比、动力学、限值），不依赖契约。

本包不使用 `generic_const_exprs`：新一代 trait 求解器不支持它，nightly 2026-10-20 在依赖它的下游包上会崩溃。

## 许可证

Apache-2.0，见 [LICENSE](../LICENSE)。
