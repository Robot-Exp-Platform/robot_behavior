# roplat_behavior

[English](README.md) | [简体中文](README_zh.md) | [从 robot_behavior 迁移](MIGRATION_zh.md)

> [!NOTE]
> **0.9 尚未发布到 crates.io。** 这些包以新名字延续 `robot_behavior` 0.8，并依赖 roplat 核心中同样尚未发布的 `roplat_msgs`。两者发布之前请从源码构建（见[从源码开发](#从源码开发)）。`robot_behavior` 的最后一个版本 0.8.1 只用于指向这里。

`roplat_behavior` 是一个 Rust 机器人接口库，让应用围绕**能力**编写：到达目标、读取结构化状态，或在每个控制周期计算下一条指令。硬件驱动或仿真后端实现这些能力，本仓库提供共同的接口契约以及建立在契约之上的算法。

| 包 | 内容 |
|---|---|
| `roplat_behavior` | 能力契约（`Robot`、运动空间、控制通道、模型映射；夹爪、力/力矩、IMU 与相机契约）、机械臂/底盘/腿足状态、以数据表达的运动限值（`MotionLimits`、`with_limits`）、相对运动（`Relative<S>`），可选的 roplat 适配。 |
| `roplat_behavior_ffi` | 契约的 Python（`py`）与 C++（`cxx`）绑定，供不依赖 roplat 运行时的驱动包使用：`py_*`／`cxx_*` 宏与数据类。 |
| `roplat_world` | 机器人所处世界的契约：仿真后端与模型选择、物理引擎、渲染器、世界实体、共享资产目录；`roplat` 特性下有 `SimulationRhythm`。 |
| `roplat_kinematics` | 以法兰为准的 DH 正运动学、雅可比与迭代逆运动学（`FLANGE`、`ik_solve_converged`）；`dh_kinematics!` 把机械臂接入模型映射。`ArmKineCache::from_chain` 可用 `roplat_description` 的 URDF 链代替 DH 表，`chain_ik` 在 URDF 限位内求其逆运动学（支持各类关节），`chain_ik_solutions` 收集通过有效性检查（如碰撞）的不同解，按与参考位形的距离排序，`UrdfKinematics` 把该链作为 `ForwardKinematics` 与 `JacobianModel`；`UrdfDynamics` 把 URDF 树作为 `DynamicsModel`，供 `roplat_control` 的力矩控制器使用。 |
| `roplat_control` | 关节 PD/PID、底盘速度 PID、关节与笛卡尔阻抗、重力补偿、计算力矩、跟踪控制器所用的目标源（`target`）、操作员遥操作（`cartesian_teleop_control`：死区、使能按键、输入超时、加速度限制）、笛卡尔伺服（`CartesianServo`：末端速度旋量转为关节速度或位置命令，只在趋近奇异时减速，到关节限位即停，不超关节速度上限，可加接近障碍的减速系数）、在笛卡尔位姿通道上跟随定时的工具位姿轨迹（`CartesianTrajectoryFollower`、`cartesian_line`、`cartesian_arc`）、实时线程优先级。 |
| `roplat_trajectory` | 以数据表达限值（`JointLimits`、`CartesianLimits`、`PathBounds`）的关节与笛卡尔插值（直线、梯形、S 曲线、四次曲线）和 COPP 时间缩放，只依赖 `roplat_msgs` 与 copp；`plan_joint_path` 在关节空间求无碰撞路径点（RRT-Connect，碰撞检查由调用方提供），`simplify_path` 与 `smooth_path` 去掉不需要的路径点并圆滑拐角，每段新路径都经过检查，与 OMPL 的路径简化器相同；`plan_joint_motion`、`simplify_motion` 与 `smooth_motion` 做同样的事，但对每段直线运动整体检查，不再逐点采样；`plan_cartesian_path` 让末端沿直线依次经过给定位姿，每步用调用方带种子的逆解求关节位形，在不可达、关节跳变或关节段无效之前停下；`plan_cartesian_motion` 对每一步整体检查。 |
| `roplat_description` | 从 URDF 读出的同一份机器人模型：运动学树、惯量、可视/碰撞几何、关节限位、链式正运动学与雅可比、整棵树各连杆的位姿与逆动力学（含重力补偿），供运动学、仿真、可视化与限位检查共用；只依赖 `roplat_msgs`。 |
| `roplat_frames` | 机器人工作单元的坐标树：编译期确定的帧 id，来自 URDF 与标定的静态变换，相机安装位置的手眼标定（`calibration::calibrate_hand_eye`，眼在手上或眼在手外），以及从图像得到标定板位姿（`target::detect_chessboard` 以亚像素精度找出棋盘格角点，各视角下顺序一致；`target::target_pose` 求标定板位姿），按 `CameraInfo` 描述的镜头畸变（`lens::distort`、`lens::undistort` 与 `lens::Rectify`，支持 plumb_bob、rational polynomial 与鱼眼模型，相当于 `image_proc` 的校正），按 `Stamp` 缓冲并查询的关节与动态变换，查询不分配内存；可选的 `JointStateNode`（feature `roplat`）；开启 feature `scene` 后，`scene::place_objects` 把规划场景中的物体移到其跟踪坐标系在某一时刻的位姿，`scene::add_sensed_points` 按图像时刻的相机位姿加入深度读数。见[设计](roplat_frames/docs/design_zh.md)。 |
| `roplat_safety` | 控制图的安全门：关节限位、限速、样本新鲜度、周期看门狗与共享停止锁存。被拒绝的命令使本周期失败，设备收到 `Break`、不发命令。见[设计](roplat_safety/docs/design_zh.md)。 |
| `roplat_scene` | 运动规划的碰撞检查：由 URDF 构造的机器人（网格取凸包）、世界中的物体与附着在连杆上的物体、允许接触对（相邻连杆默认允许）与安全距离。`PlanningScene` 回答 `plan_joint_path` 的有效性检查，并给出最近的一对；碰撞几何由 parry3d-f64 计算，无原生依赖。 另有命名位姿（`NamedPoses`）与工具安装位置（`EndEffector`），即规划代码需要的 SRDF 内容，在代码中构造。 也可加入传感得到的几何：`points::depth_to_points` 按 `CameraInfo` 内参把深度图反投影为点，`points::register_depth` 把深度图重投影到旁边的彩色相机（即 `depth_image_proc/register`），`add_points` 把点作为占据体素加入场景，供规划避开。 料箱、货架等凹形物体可保留内部空间：`add_concave_object`（网格文件）与 `add_trimesh`（代码中给出的三角形）用 V-HACD 把它们分成若干凸块，而不是取一个凸包。 `motion_contact` 与 `motion_is_free` 用保守推进整体检查一段关节直线运动，两个采样点之间的薄障碍不会被跳过。 `decompose_link` 以同样方式分解凹形连杆的网格；`without_robot_points` 在传感点加入场景前去掉落在机器人或其所持物体上的点（即 MoveIt 的自身滤波）。 `meshes` 把每个连杆与物体给成世界系中的三角网格，供可视化使用（即 RViz 的规划场景显示）。 `set_pair_padding` 让某一对连杆或物体使用自己的安全间隙，取代场景的统一间隙，位形检查与整段运动检查都按它计算。 `occupancy::OccupancyMap` 像 MoveIt 的 OctoMap 一样随扫描更新传感障碍（每个体素保存对数几率，命中处标记占据，光线经过处清除），`set_occupancy` 把占据体素放进场景。 |
| `roplat_grasp` | 平行夹爪对长方体、圆柱、球体的抓取候选（由优到劣），以及 `plan_pick`：到预抓取位姿的无碰撞路径、直线接近、带着物体直线抬起；按顺序尝试候选，并说明每个失败的原因；`plan_pick_motion` 做同样的规划，但每一段都用 `PlanningScene::motion_is_free` 整体检查。 `GraspCheck` 依据任何夹爪都会报告的开口宽度判断是否抓住（抓住、空抓、宽度不符，或与驱动自身的判断不一致），`carried_mass` 在抬起后由腕部力传感器确认负载。 |
| `roplat_perception` | 面向操作的点云处理，即 ROS 用 PCL 搭的桌面流程：体素降采样、按长方体裁剪、去除孤立点；RANSAC 求主平面（可要求在 `up` 的倾角内水平）与欧氏聚类；在平面上拟合长方体（轮廓的最小面积矩形）或直立圆柱（圆拟合），结果是 `roplat_grasp::grasps` 与 `PlanningScene::add_object` 直接接受的 `Geometry` 与位姿；`segment_tabletop` 一次完成全部步骤，给出桌面和其上的每个物体；点到点 ICP 求已知模型的位姿；表面法向与曲率（`estimate_normals`，即 PCL 的 `NormalEstimation`），以及沿法向生长的光滑区域（`smooth_regions`，即 PCL 的 `RegionGrowing`），可把在棱处相接的表面分开。输入是普通的 `[f64; 3]` 点，无原生依赖。 |
| `roplat_diagnostics` | 运行观测：频率、周期与延迟探针，给出均值、p99 与最坏值；`FrequencyCheck`、`LatencyCheck` 把它们判为 OK／WARN／ERROR／STALE（阈值同 ROS 的 `FrequencyStatus`）；`Updater` 与按名称前缀分组的 `Aggregator`。全部基于 `roplat_msgs::Clock`，仿真与回放时间同样适用。开启 `roplat` 后，直通的 `ProbeNode`／`LatencyNode` 在图内喂给检查，由图外读取。 |

采样与几何类型（`Pose`、`JointSample`、`SpatialSample`、`LoadState`、`Twist` 等）来自 roplat 核心的 [`roplat_msgs`](https://github.com/Robot-Exp-Platform/roplat)，由 `roplat_behavior` 重新导出。每个类型只有一种表示和固定的原生布局，同一份数据可以跨进程、跨语言传递。除非打开 `roplat` feature，这些包都不依赖 roplat 运行时。

如果你希望在兼容的后端之间复用应用与控制器，或为新驱动建立一致的接口，可以使用它们。连接真机还需要选择对应驱动；物理仿真需要 [RsBullet](https://github.com/Robot-Exp-Platform/rsbullet) 等后端。

## 设计理念：统一接口，把设备差异留在驱动中

关节位置与关节力矩都可能是 `[f64; 6]`，但含义不同。`JointSpace<6>` 表示运动目标空间，`JointPositionControl<6>` 与 `TorqueControl<6>` 表示周期控制通道。这些类型选择对应的观测和指令类型，让应用明确表达操作意图。

各项能力独立实现。支持关节运动的驱动，不会自动获得力矩控制、逆运动学或原生异步 I/O。泛型应用声明需要的 trait，不必假定所有机器人都提供相同功能。

驱动负责通信、周期组织、状态采集和会话收尾。控制闭包接收观测值与 `Duration`，计算下一条指令。控制器辅助函数返回普通闭包，不自行创建调度器，也不提供硬件实时性保证。

| 想完成的操作 | 应用入口 | 后端实现 |
|---|---|---|
| 到达一个目标 | `Motion::move_to::<S>` | `MoveTo<S>` |
| 发送采样轨迹 | `Motion::move_traj::<S>` | `MoveTraj<S>` |
| 每周期计算指令 | `Control::control_with::<S, _>` | `ControlWith<S>` |
| 等待完整异步控制会话 | `AsyncControl::control_native_async::<S, _>` | `AsyncControlWith<S>` |
| 读取后端原生状态 | `Robot::read_state` | `Robot` |
| 使用机械臂状态或模型 | `Arm<N>`、`SpaceMap` 等模型 trait | 后端实际支持的能力 |

## 第一个程序：不连接机器人

示例使用 `roplat_behavior` 和 [drives](https://github.com/Robot-Exp-Platform/drives) 工作区中的控制台参考机器人 `roplat_exrobot`。库使用 nightly Rust 特性，依赖构建需要 C++ 工具链（Windows 使用 MSVC Build Tools 和 Windows SDK）：

```toml
[dependencies]
roplat_behavior = "0.9.0"
roplat_exrobot = "0.5.0"
```

```rust
use roplat_behavior::{
    Control, JointPositionControl, JointSpace, Motion, Robot, RobotResult,
};
use roplat_exrobot::ExRobot;

fn main() -> RobotResult<()> {
    let mut arm = ExRobot::<6>::new();
    arm.init()?;
    arm.move_to::<JointSpace<6>>([0.1; 6])?;

    let mut cycles = 0;
    arm.control_with::<JointPositionControl<6>, _>(|_state, _dt| {
        cycles += 1;
        ([0.1; 6], cycles == 3)
    })?;

    println!("completed {cycles} control cycles");
    arm.shutdown()
}
```

用 `cargo +nightly run` 运行。程序打印参考机器人的生命周期和指令，输出 `completed 3 control cycles` 后关闭。最后一次回调返回 `true`，这条指令发送后会话结束。参考机器人不会运动，也不积分物理状态。

接下来连接设备时，把 `ExRobot` 换成对应驱动的构造方式，并按该驱动的说明配置连接和运行模式。相同的 Rust 类型让代码可以复用，但不会让限位、坐标系、周期或控制增益变得通用。

## 读取状态

`Robot::State` 由后端决定。编写通用机械臂代码时，使用驱动的 `Arm<N>` 实现和结构化状态类型：

- `JointState<N>` 包含 `meas`、`cmd`、`des` 三个视图：测量反馈、已接受指令和期望参考。
- `JointSample<N>` 带有 `q`、`dq`、`ddq`、`tau`、`dtau`，通过 `sample.q()` 这类访问器读取。`None` 表示该值没有提供，绝不是伪造的零值。生产者使用 `JointSample::EMPTY.with_q(q)` 或 `sample.set_q(Some(q))`。
- `ArmState<N>` 增加法兰状态，以及可选的 TCP、刚度坐标系和负载信息。原生状态与这个通用视图不一定来自同一次设备读取。

单位、坐标系、字段是否可用以及数据新鲜度，以后端的状态说明为准。字段有值并不证明刚收到了新的传感器样本。

## 运动与控制会话

需要后端执行一个目标或一段轨迹时，使用运动调用；需要由你的代码计算每个周期的指令时，使用控制会话。`control_with` 的回调返回 `(command, done)`，调用会阻塞到会话结束。

如果需要在没有算法指令的情况下退出，使用 `control_with_flow` 并返回 `ControlStep<Command>`：

| 回调结果 | 含义 |
|---|---|
| `ControlFlow::Continue((command, false))` | 发送指令，继续。 |
| `ControlFlow::Continue((command, true))` | 发送这条最终指令，正常结束。 |
| `ControlFlow::Break(())` | 本周期不发送算法指令，执行驱动的会话收尾。 |

收尾方式取决于设备协议。`Break` 不规定发送零指令，也不代表急停。

async 有两种不同用法：

- `control_with_async` / `control_with_flow_async` 在**阻塞会话中使用 async 回调**，不返回会话 future。
- `control_native_async(&mut callback)` 返回**整个会话的 future**，包括 I/O 与收尾。它要求后端实现 `AsyncControlWith<S>`，并使用驱动规定的运行时。丢弃这个 future 不等于合作式关闭会话。

## 运动学、控制器与轨迹

带 DH 参数表的机械臂实现 `roplat_kinematics::ArmForwardKinematics<N>`（迭代求解器还需要 `ArmInverseKinematics<N>`）。`dh_kinematics!(MyArm, 6)` 随后实现法兰位姿与雅可比映射，`dh_kinematics!(MyArm, 6, inverse)` 还实现逆映射，泛型代码就能通过 `roplat_behavior` 的 `ForwardKinematics`、`JacobianModel`、`InverseKinematics` 使用它。

`roplat_kinematics` 回答“要到达这个位姿，关节该取什么值”，并把 DH 与 URDF 两种模型接到契约的模型映射上；`roplat_description` 回答“在这组关节值下模型是什么样”（各坐标系、雅可比、动力学、限值），不依赖契约。

`roplat_control` 返回可直接交给 `control_with` 的普通闭包。`roplat_trajectory` 按 `JointLimits` 与控制周期规划；无界写 `f64::INFINITY`，规划要求速度、加速度有界。增益与模型数据需按后端选择；它们不提供时序保证。

`roplat_control::CartesianTrajectoryFollower` 在 `CartesianPoseControl` 上播放定时的 `CartesianWaypoint`：相邻路点之间位置走直线、姿态做球面插值；路径与终点容差以米和弧度计，与测得的法兰位姿比较，超出时返回错误而不修改命令。`cartesian_line` 生成按控制周期采样的直线最小加加速度运动，`cartesian_arc` 生成经过一个途经点的圆弧运动（相当于 Pilz `CIRC` 的 interim 点方式）。开启 `roplat_control` 的 `roplat` feature 后，它也是一个节点，与关节轨迹跟随相同。

## Feature 与 Roplat 集成

默认 feature 为空。Rust 的运动、控制、状态和模型接口不依赖 Roplat。

| `roplat_behavior` 的 feature | 用途 |
|---|---|
| `roplat` | `ControlRhythm`、`AsyncControlRhythm`，以及运动/模型节点适配器。 |
| `conformance` | 供驱动测试使用的契约检查。 |

Python 与 C++ 绑定在 `roplat_behavior_ffi`（特性 `py`、`cxx`）；`SimulationRhythm` 在 `roplat_world`（特性 `roplat`）。

编写 Roplat 应用时，为依赖添加 `features = ["roplat"]`，并依赖匹配的 `roplat = "0.5"`。用 `#[roplat::system]` 组织应用图；阻塞式后端使用 `ControlRhythm`，原生异步后端使用 `AsyncControlRhythm`。适配器把相同的观测和指令接入图中，不会为驱动补出缺失的能力。

机器人始终归创建层所有。节点和节律通过独立的 `Resources` 通道获得可变引用，业务 Input 和 Output 不需要携带机器人。

```rust
#[roplat::system]
async fn session(mut robot: MyRobot, mut law: MyLaw) -> RoplatResult<MyRobot> {
    let mut control = ControlRhythm::<MyRobot, JointPositionControl<7>>::new();
    let mut reach = MotionNode::<MyRobot, JointSpace<7>>::new();
    let mut goal = Goal::new();

    &mut robot >> reach.resources.0;
    goal >> reach;
    reach.output?;
    &mut robot >> control.resources.0;
    control >> |state| state >> law;

    Ok(robot)
}
```

创建层管理节点和机器人的生命周期。`MotionNode` 声明 `Resources = (R,)`、`Input = S::Target`、`Output = RobotResult<()>`；两种控制节律都声明 `Resources = (R,)`，`Input = Output = ()`。借用覆盖一次 process 或完整的 drive，包括失败与停止路径。`reach.output?` 显式把业务错误提升为 System 失败。接入图时参见[适配器实现](roplat_behavior/src/roplat)和[完整 System 示例](roplat_behavior/tests/control_rhythm.rs)。

## 组合机器人模型与仿真后端

同一种设备可以在 `RobotDescription::SIMULATION_MODELS` 中提供多个模型。
加载器实现 `SimulationBackend`，声明后端标识与实际接受的格式，再通过
`select_robot_model::<R>()` 选择模型。只声明旧 `URDF` 常量的设备仍可由接受
URDF 的后端使用。

```rust
use roplat_behavior::{ModelFormat, RobotDescription, SimulationModel};

struct ArmModels;
impl RobotDescription for ArmModels {
    const SIMULATION_MODELS: &'static [SimulationModel] = &[
        SimulationModel::new(ModelFormat::Urdf, "arm/robot.urdf"),
        SimulationModel::new(ModelFormat::Mjcf, "arm/scene.xml").for_backend("mujoco"),
    ];
}
```

选择顺序是：指定后端的模型、通用模型、旧 `URDF`；每层先按后端的格式优先级，
再按设备的声明顺序。`SelectedSimulationModel` 记录选择来源；不匹配时返回
带有后端和接受格式的 `ModelSelectionError`。路径遵循后端的资源搜索规则。
选中的文件加载失败时应报告失败，不应静默换用另一份模型并改变物理条件。

这层接口只选择模型，不自动转换文件或实现仿真器。后端仍需验证模型，并提供
实际支持的关节、控制、传感器和场景能力。操作系统与构建条件由后端安装契约
说明；接受 MJCF 不表示当前电脑已安装或支持 MuJoCo。仿真步进继续使用独立
的 `PhysicsEngine`，不改变真机的运动与控制语义。
[选择测试](roplat_world/tests/simulation_models.rs) 覆盖设备与后端匹配及不匹配的组合，
不启动仿真器。

可选的 `roplat_world::SimulationRhythm<E>` 为任意兼容的
`PhysicsEngine` 提供有限步数与墙钟节律，并在真实 `#[roplat::system]` 图中运行。
`logical(engine).with_max_steps(n)` 按引擎速度推进；`new(engine, period)`
只限制墙钟节奏，不改变物理步长。每次驱动重新计算步数预算，但不会重置世界。
引擎报错保留错误来源、停止后续图执行；合作退出可打断等待，仍会等待正在执行的
子域归还其节点。状态刷新、命令顺序及物理语义由具体后端负责。

这层适配不依赖 Bullet 或 MuJoCo，Tokio 仅随 `roplat` feature 启用。
它不会代替后端调用消耗所有权的 `shutdown`；后端可使用 RAII 清理，或由应用
通过 `into_engine()` 取回引擎并显式关闭。

## 继续阅读

- [运动与轨迹接口](roplat_behavior/src/robot/motion.rs)、[控制契约](roplat_behavior/src/robot/control.rs)、[原生异步契约](roplat_behavior/src/robot/async_control.rs)。
- [状态类型](roplat_behavior/src/robot/state.rs)与[模型映射](roplat_behavior/src/robot/model.rs)。
- [运动学](roplat_kinematics/src)、[控制器](roplat_control/src)、[轨迹](roplat_trajectory/src)。
- [契约示例](roplat_behavior/tests/control_flow.rs)、[异步示例](roplat_behavior/tests/async_control.rs)和[外部接口示例](roplat_behavior/examples)。
- [相对 robot_behavior 0.8 的变化](MIGRATION_zh.md)。

驱动作者可以导入 `roplat_behavior::driver::*`，应用可以导入 `roplat_behavior::behavior::*`。先在 `Robot` 中实现设备生命周期与状态，再分别实现支持的运动、控制和模型能力。默认生命周期钩子是空操作，部分未实现操作会返回错误；继承默认实现不等于实现了设备行为。

## 从源码开发

本仓库是上述各包组成的 Cargo 工作区，每个包各自声明版本。`roplat_msgs` 发布之前，根清单把 `roplat` 与 `roplat_msgs` 指向同级的 roplat 核心检出，因此需要并排克隆：

```text
Robot-Exp-Platform/
├── roplat/
└── roplat_behavior/
```

`cargo +nightly test --workspace` 运行默认测试；加 `--features roplat_behavior/roplat` 运行适配器测试。这些测试是契约的有限模拟示例，不代表硬件时序。驱动集成见 [drives](https://github.com/Robot-Exp-Platform/drives)。

## 许可证

Apache-2.0，见 [LICENSE](LICENSE)。
