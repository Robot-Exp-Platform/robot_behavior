# roplat_description

[English](README.md) | [简体中文](README_zh.md)

One robot model read from URDF, shared by kinematics, simulation, visualization and limit checks.

Part of the [roplat_behavior workspace](../README.md). 0.9 is not on crates.io yet; build it from source as [Working from source](../README.md#working-from-source) describes.

## Contents

- `RobotModel`: the kinematic tree read from URDF, with links (inertia, visual and collision geometry) and joints (origin, axis, limits).
- `Chain`: one base-to-tip path cut out of the tree, for forward kinematics, per-joint bounds and its geometric Jacobian.
- `RobotModel::link_frames` places every link of the tree at once, for visualization.
- `RobotModel::inverse_dynamics` gives the joint efforts for a motion, gravity compensation included.
- Poses use `roplat_msgs::Pose`; limits convert to `roplat_msgs::JointLimits`.

Mesh file names are kept as written (`package://…`, relative paths); resolving them is the consumer's job, as for `roplat_behavior::SimulationModel` paths. DH parameters remain a separate source in [`roplat_kinematics`](../roplat_kinematics/README.md), whose `ArmKineCache::from_chain` also accepts a `Chain`.

The crate depends only on `roplat_msgs` (with `urdf-rs` for parsing), not on the contracts, so [`roplat_scene`](../roplat_scene/README.md), [`roplat_frames`](../roplat_frames/README.md), [`roplat_grasp`](../roplat_grasp/README.md) and [`roplat_perception`](../roplat_perception/README.md) build on it without `roplat_behavior`.

## License

Apache-2.0; see [LICENSE](../LICENSE).
