# roplat_behavior_ffi

[English](README.md) | [简体中文](README_zh.md)

Python and C++ bindings of the [`roplat_behavior`](../roplat_behavior/README.md) capability contracts, for driver packages used without the roplat runtime.

Part of the [roplat_behavior workspace](../README.md). 0.9 is not on crates.io yet; build it from source as [Working from source](../README.md#working-from-source) describes.

## Contents

- Feature `py`: Python data classes and the `py_*` macros that wrap a driver for Python: `py_robot!`, `py_arm!`, `py_joint_motion!`, `py_flange_move!`, `py_flange_motion!`, the control macros (`py_joint_position_control!`, `py_joint_torque_control!`, `py_cartesian_pose_control!`, …) and others.
- Feature `cxx`: the cxx bridge types and the `cxx_*` macros for C++ (`cxx_robot_api!`, `cxx_arm_motion_api!`, `cxx_arm_bridge!`).

A driver enables `py` or `cxx` and calls the macros on its robot type. They expand to `pyo3` / `cxx` code in the driver, so the driver depends on the same `pyo3` and `cxx` versions as this crate. A driver's own `pyo3` dependency turns on `extension-module` for its Python package; this crate needs only the macros' features.

The bindings follow the contract: when `roplat_behavior` changes its minor version, this crate releases the matching one, and `pyo3` or `cxx` upgrades wait for such a release.

## Features

| Feature | Purpose |
|---|---|
| `py` | Python classes and the `py_*` macros (`pyo3`). |
| `cxx` | The cxx bridge types and the `cxx_*` macros. |

Default features are empty.

## Examples

- [`examples/py.rs`](examples/py.rs) (`--features py`) wraps a mock robot for Python. It is not a device connection or a Python extension package.
- [`examples/cxx.rs`](examples/cxx.rs) (`--features cxx`) is a lifecycle bridge. A standalone C++ application also needs generated glue.

## License

Apache-2.0; see [LICENSE](../LICENSE).
