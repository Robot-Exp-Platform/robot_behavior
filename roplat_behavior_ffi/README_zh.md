# roplat_behavior_ffi

[English](README.md) | [简体中文](README_zh.md)

[`roplat_behavior`](../roplat_behavior/README_zh.md) 能力契约的 Python 与 C++ 绑定，供不依赖 roplat 运行时的驱动包使用。

属于 [roplat_behavior 工作区](../README_zh.md)。0.9 尚未发布到 crates.io，请按[从源码开发](../README_zh.md#从源码开发)从源码构建。

## 内容

- feature `py`：Python 数据类，以及把驱动包装给 Python 的 `py_*` 宏：`py_robot!`、`py_arm!`、`py_joint_motion!`、`py_flange_move!`、`py_flange_motion!`、各控制宏（`py_joint_position_control!`、`py_joint_torque_control!`、`py_cartesian_pose_control!` 等）及其他。
- feature `cxx`：cxx 桥接类型，以及面向 C++ 的 `cxx_*` 宏（`cxx_robot_api!`、`cxx_arm_motion_api!`、`cxx_arm_bridge!`）。

驱动打开 `py` 或 `cxx`，在自己的机器人类型上调用这些宏。宏在驱动中展开为 `pyo3`／`cxx` 代码，因此驱动依赖的 `pyo3` 与 `cxx` 版本须与本包相同。驱动自己的 `pyo3` 依赖为其 Python 包打开 `extension-module`；本包只需要宏所用的 feature。

绑定跟随契约：`roplat_behavior` 升 minor 版本时，本包发布对应版本；`pyo3` 或 `cxx` 的升级也等到这样的版本一起发布。

## Feature

| Feature | 用途 |
|---|---|
| `py` | Python 类与 `py_*` 宏（`pyo3`）。 |
| `cxx` | cxx 桥接类型与 `cxx_*` 宏。 |

默认 feature 为空。

## 示例

- [`examples/py.rs`](examples/py.rs)（`--features py`）把一个模拟机器人包装给 Python。它不连接设备，也不是 Python 扩展包。
- [`examples/cxx.rs`](examples/cxx.rs)（`--features cxx`）是生命周期桥接。独立的 C++ 应用还需要生成的胶水代码。

## 许可证

Apache-2.0，见 [LICENSE](../LICENSE)。
