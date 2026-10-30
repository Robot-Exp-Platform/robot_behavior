# roplat_description

[English](README.md) | [简体中文](README_zh.md)

从 URDF 读出的同一份机器人模型，供运动学、仿真、可视化与限位检查共用。

属于 [roplat_behavior 工作区](../README_zh.md)。0.9 尚未发布到 crates.io，请按[从源码开发](../README_zh.md#从源码开发)从源码构建。

## 内容

- `RobotModel`：从 URDF 读出的运动学树，包括连杆（惯量、可视与碰撞几何）和关节（原点、轴、限位）。
- `Chain`：从树中截出的一条基座到末端的路径，用于正运动学、逐关节限值及其几何雅可比。
- `RobotModel::link_frames` 一次给出整棵树各连杆的位姿，供可视化使用。
- `RobotModel::inverse_dynamics` 给出一段运动所需的关节力，含重力补偿。
- 位姿使用 `roplat_msgs::Pose`；限位可转换为 `roplat_msgs::JointLimits`。

网格文件名按原样保存（`package://…`、相对路径），由使用方解析，与 `roplat_behavior::SimulationModel` 的路径相同。DH 参数仍是 [`roplat_kinematics`](../roplat_kinematics/README_zh.md) 中的另一种来源，其 `ArmKineCache::from_chain` 也接受 `Chain`。

本包只依赖 `roplat_msgs`（解析用 `urdf-rs`），不依赖契约，因此 [`roplat_scene`](../roplat_scene/README_zh.md)、[`roplat_frames`](../roplat_frames/README_zh.md)、[`roplat_grasp`](../roplat_grasp/README_zh.md) 与 [`roplat_perception`](../roplat_perception/README_zh.md) 建立在它之上而不需要 `roplat_behavior`。

## 许可证

Apache-2.0，见 [LICENSE](../LICENSE)。
