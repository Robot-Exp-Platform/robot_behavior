# Readme

[English](README.md) | [简体中文](README_cn.md)

<p align = "center">
    <strong>
        <a href="https://robot-exp-platform.github.io/robot_behavior_page/">Documentation</a>
    </strong>
</p>

---

At present, the interface definitions among minor versions are still unstable. It is expected that the first stable version will be implemented before 2026.

This library is part of the [Universal Robot Driver Project](https://github.com/Robot-Exp-Platform/robot_behavior)! We are committed to providing Rust driver support for more robotic platforms! **Unifying driver interfaces across different robot models, reducing the learning curve for robotics, and delivering more efficient robot control solutions!**

This library is a general robot driven feature library, used to describe the characteristics of robot behavior. It provides some common feature descriptors and implementations for use by other robot driver libraries. At the same time, the signature database also implements automatic derivation macros for common interfaces, which can be used to derive secure interface implementations.

We aim to ensure consistent behavior of driver libraries across various operating platforms, as well as compatibility among different driver libraries. We hope that through the use of this library, we can minimize the learning curve associated with robot operations and achieve seamless integration.

## The principles of interface design

- **Complete interface description**. During the usage of each interface, the behavior executed by that interface should be clearly and completely expressed
- **Semantic consistency**: Function parameters/return values and function names should have consistent semantics
- **Consistent behavior**: The behavior of the interface should remain consistent across different driver libraries

## How to use the robot driven by this library

[robot_behavior](https://robot-exp-platform.github.io/robot_behavior_page/)。

## How to drive a robot driver

### How to implement the driver to python

open the feature "to_py" , and then use the macros provided in the library to implement the driver. The macros will automatically generate the corresponding Python interface for you.

```rust
#[cfg(feature = "to_py")]
{
    use pyo3::types::{PyAnyMethods, PyModule, PyModuleMethods};
    use robot_behavior::*;

    struct ExRobot;

    #[pyo3::pyclass]
    struct PyExRobot(ExRobot);

    // Implement the necessary traits for PyExRobot
    py_robot_behavior!(PyExRobot(ExRobot));
    py_arm_behavior!(PyExRobot<{0}>(ExRobot));
    py_arm_param!(PyExRobot<{0}>(ExRobot));

    // Implement the preplanned motion traits for PyExRobot
    py_arm_preplanned_motion_impl!(PyExRobot<{0}>(ExRobot));
    py_arm_preplanned_motion!(PyExRobot<{0}>(ExRobot));
    py_arm_preplanned_motion_ext!(PyExRobot<{0}>(ExRobot));

    // Implement the streaming motion traits for PyExRobot
    #[pyo3::pyclass]
    struct ExRobotHandle;
    py_arm_streaming_motion!(PyExRobot<{0}>(ExRobot) -> ExRobotHandle);
    py_arm_streaming_motion_ext!(PyExRobot<{0}>(ExRobot));

    // Implement the real-time control traits for PyExRobot
    py_arm_real_time_control!(PyExRobot<{0}>(ExRobot));
    py_arm_real_time_control_ext!(PyExRobot<{0}>(ExRobot));

    #[pyo3::pymodule]
    fn ex_robot(m: &pyo3::Bound<'_, PyModule>) -> pyo3::PyResult<()> {
        m.add_class::<PyExRobot>()?;
        m.add_class::<PyPose>()?;
        m.add_class::<PyArmState>()?;
        m.add_class::<LoadState>()?;
        Ok(())
    }
}
```
