# roplat_kinematics

[English](README.md) | [简体中文](README_zh.md)

Kinematics for [`roplat_behavior`](../roplat_behavior/README.md) arms, from a Denavit–Hartenberg table or a URDF chain, connected to the contract's model maps.

Part of the [roplat_behavior workspace](../README.md). 0.9 is not on crates.io yet; build it from source as [Working from source](../README.md#working-from-source) describes.

## Contents

- Denavit–Hartenberg forward kinematics, Jacobians and iterative inverse kinematics at the flange (`FLANGE`, `ik_solve_converged`).
- `dh_kinematics!` connects a DH arm to the model maps.
- An arm read from URDF needs no DH table:
  - `ArmKineCache::from_chain` builds the same cache from a [`roplat_description`](../roplat_description/README.md) chain.
  - `chain_ik` solves its inverse kinematics for any joint kinds, within the URDF limits.
  - `chain_ik_solutions` collects the distinct solutions that pass a validity check (a collision check, for example), nearest first.
  - `UrdfKinematics` makes the chain a `ForwardKinematics` and `JacobianModel`, as `dh_kinematics!` does for a DH arm.
- `UrdfDynamics` makes a URDF tree a `DynamicsModel` for the [`roplat_control`](../roplat_control/README.md) torque controllers.

## A DH arm

An arm with a Denavit–Hartenberg table implements `ArmForwardKinematics<N>` (and `ArmInverseKinematics<N>` for the iterative solvers). `dh_kinematics!(MyArm, 6)` then implements the flange-pose and Jacobian maps, and `dh_kinematics!(MyArm, 6, inverse)` also the inverse map, so generic code can use `ForwardKinematics`, `JacobianModel` and `InverseKinematics` from `roplat_behavior`.

## Kinematics or description?

`roplat_kinematics` answers "which joint positions reach this pose" and connects DH and URDF models to the contract's model maps; `roplat_description` answers "what is the model at these joint positions" (frames, Jacobians, dynamics, limits) without the contract.

The crate does not use `generic_const_exprs`: the next-generation trait solver does not support it, and nightly 2026-10-20 crashes on downstream crates that depend on it.

## License

Apache-2.0; see [LICENSE](../LICENSE).
