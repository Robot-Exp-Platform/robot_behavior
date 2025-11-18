# Readme

[English](README.md) | [简体中文](README_cn.md)

<p align = "center">
    <strong>
        <a href="https://robot-exp-platform.github.io/robot_behavior_page/">Documentation</a>
    </strong>
</p>

---

目前小版本间的接口定义尚不稳定，预期在2026年前实现第一个稳定版本

本库是[通用机器人驱动计划](https://github.com/Robot-Exp-Platform/robot_behavior)中的一员！我们立志于为更多的机器人平台提供 Rust 语言及更多语言的驱动支持！**统一不同型号的机器人驱动接口，降低机器人学习成本，提供更高效的机器人控制方案！**

本库是通用机器人驱动的特征库，用于描述机器人行为的特征。它提供了一些通用的特征描述符和实现，供其他机器人驱动库使用。同时特征库也为常见的接口实现了自动派生宏，用于派生安全的接口实现。

我们正在力求确保驱动库在不同的操作平台上行为的一致性和不同驱动库之间的兼容性。我们希望通过这个库，减少机器人操作之间的学习成本，达到一门通而门门通的效果。

## 接口设计原则

- **描述完整**，每个接口在使用过程中应当完整明确的表达该接口执行的行为
- **语义一致**，函数参数/返回值与函数名应当具有一致的语义
- **行为一致**，接口的行为应当在不同的驱动库中保持一致

## 如何使用本库驱动的机器人？

由本库派生的机器人均满足相同的接口规范。文档见 [robot_behavior](https://robot-exp-platform.github.io/robot_behavior_page/)。

对于不同的机器人，我们通常建议以 `robot` 作为对象名，在之后的说明中，均以 `robot` 作为实例化后的机器人对象。

一个简单的移动机器人的样例如下：

```rust
robot.move_to(MotionType::Joint([0.;6]))?;
robot.move_to(MotionType::Cartensian(Pose::Euler([0;3],[0.;3])))?;
```

当然我们还准备了一些简化的函数，如以下的函数具备和上述代码相同的功能：

```rust
robot.move_joint([0.;6])?;
robot.move_cartesian_euler([0;3],[0.;3])?;
```

整体来说，我们将机器人接口分为 3 类：预规划接口、流式接口、闭包接口（实时控制接口），这三类接口分别描述:轨迹发布时已知；轨迹发布时未知，但是运行时发布；轨迹发布时未知，运行时通过闭包计算得到控制量三种情形。基本涵盖了所有的控制方法。

我们正在努力实现更多机器人以及为机器人提供更加简单易懂的接口实现方法，尽情期待！

你可以在 [robots](https://robot-exp-platform.github.io/robot_behavior_page/) 中找到当前已经实现了哪些机器人，如果你实现了机器人，也可以联系我们更新。

## 如何为一个机器人实现驱动？

为你的机器人实现一个结构体，然后尝试为他实现 [`robot_behavior::robot::arm`](./src/robot/arm.rs) 中的一系列特征吧！

我们提供了过程中需要的一系列函数，如规划函数等等，供您任取。

### 如何将驱动封装为 Python 接口？

打开 "to_py" 特性，然后使用库中提供的宏来实现驱动。宏将自动为您生成相应的 Python 接口。

```rust
#[cfg(feature = "to_py")]
{
    use pyo3::types::{PyModule, PyModuleMethods};
    use robot_behavior::*;

    struct ExRobot;

    #[pyo3::pyclass]
    struct PyExRobot(ExRobot);

    py_robot_behavior!(PyExRobot(ExRobot));
    py_arm_behavior!(PyExRobot<{0}>(ExRobot));
    py_arm_param!(PyExRobot<{0}>(ExRobot));
    py_arm_preplanned_motion_impl!(PyExRobot<{0}>(ExRobot));
    py_arm_preplanned_motion!(PyExRobot<{0}>(ExRobot));
    py_arm_preplanned_motion_ext!(PyExRobot<{0}>(ExRobot));

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
