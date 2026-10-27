# roplat_safety：安全层第一版（E07）

状态：第一版，未发布。范围按首批场景（机械臂操作）收窄：关节位置／速度／力矩通道与笛卡尔速度通道；限位、限速、新鲜度门、看门狗，以及急停传播中不需要改契约的部分。不改 roplat 核心，不改 `ControlWith`、`ControlStep`、`ControlRhythm` 等现有契约。

## 1. 原则：只检查，只拒绝

安全构件检查命令，不合格就拒绝，不把命令改成别的值。

- **拒绝怎样到达驱动。** 每个门输出 `Result<_, SafetyViolation>`，图里写 `gate?`。拒绝使本周期的域失败，`ControlRhythm` 收到 `Err`，回调返回 `Break`：本周期不发命令，驱动进入会话收尾，drive 的结果就是这个违例。这与 task-g 中 `StepGuard` 的做法一致，也正是 `ControlStep` 契约里"算不出命令"的唯一表达。
- **不夹取。** 门若把越限命令夹到边界再发出，发出的就是门伪造的命令。饱和是控制器设计的一部分，应放在上游；饱和后的命令照样要过门。libk1 里 `position_command_from_state`（夹取）加 `validate_command`（拒绝）就是这两层，本库只沉淀后一层。
- **不发 `hold_command`。** 它是连续性回退，契约特意没有叫它 safe。
- **`Break` 不是急停**，只是本次会话按驱动协议收尾。急停见第 6 节。

`SafetyViolation` 经 `RoplatError::Io(io::Error::other(..))` 传递，与 `ControlSessionError` 的做法相同，核心不需要新变体。`SafetyViolation::find` 能穿过 `ParallelError`、`LifecycleError` 和 `ControlSessionError`（驱动收尾也失败的情况）找回原值。

## 2. 放在哪里

新 crate `roplat_safety`，放在 roplat_behavior 工作区，不并入 `roplat_control`：

- `roplat_control` 是控制器，负责算命令；安全门负责验证命令。分开放，饱和与拒绝不会混在一个节点里。
- 依赖只有 `roplat_behavior`（通道与观测类型）和 `roplat_msgs`（`JointLimits`、`Stamp`、`Clock`）。feature `roplat` 提供节点，与 `roplat_behavior` 一样默认关闭；不开时纯检查不依赖运行时，驱动也能直接用（像 libk1 在驱动内做门那样）。

## 3. 构件

| 纯检查 | 节点 | 拒绝条件 |
|---|---|---|
| `JointPositionCheck` | `CommandGuard<_>` | 越出位置范围；相邻命令隐含速度超过 `dq_max`、速度变化超过 `ddq_max`；可选：命令与测量位置之差超过跟踪误差上限（覆盖 `StepGuard` 的用途，防止被挡住的机械臂命令越走越远） |
| `JointVelocityCheck` | `CommandGuard<_>` | `|dq|` 超过 `dq_max`；变化超过 `ddq_max`；从测量位置出发，以该速度走一个周期再按 `ddq_max` 刹停仍越出位置范围（回到范围的方向总放行） |
| `JointTorqueCheck` | `CommandGuard<_>` | `|τ|` 超过 `tau_max`；变化超过力矩变化率上限（默认不限，可取 `Joints::TORQUE_DOT_BOUND`） |
| `CartesianVelocityCheck` | `CommandGuard<_>` | 线速度、角速度范数超限；二者变化率超限 |
| `AgeLimit` | `FreshnessGate` | 没有样本；时钟未指定或与门的时钟不同；时间戳在门的"现在"之后；超龄 |
| `Watchdog` | `CycleWatchdog` | 相邻两个周期的间隔超时（按门的时钟，或按设备报告的 `dt`） |
| `SafetyLatch` | `LatchGate`、`Trip` | 锁存已跳闸 |

所有比较都写成 NaN 拒绝：命令或限值是 NaN 都不放行。限值为 `+∞` 表示不检查；`Joints` 默认的 `f64::MAX` 实际上也等于不限。`joint_limits::<R, N>()` 从驱动的 `Joints` 常量取限值；URDF 来源用 `roplat_description` 的 `Chain::joint_limits`。

## 4. 限速：参考是上一条命令与 `dt`

限速比较本周期命令与上一条**被接受**的命令，按 Yield 中的 `dt` 换算成速率。会话第一周期没有上一条命令，参考取自：

| 通道 | 第一周期的参考 | 理由 |
|---|---|---|
| 关节位置 | 测量位置、测量速度 | 物理事实；驱动的 `cmd` 可能是上一个会话、甚至另一个通道留下的 |
| 关节速度 | 测量速度 | 同上 |
| 关节力矩 | 显式起点，默认 0 | 力矩命令的含义随驱动而变（Franka 的命令不含重力补偿），不能拿测量力矩作参考 |
| 笛卡尔速度 | 测量法兰 twist，或显式起点 | Franka 的状态里没有测量 twist |

需要测量值而观测里没有（或不是有限数）时拒绝（`Unobserved`），对应限值不限时不需要测量。`dt = 0` 时只放行与上一条相同的命令。

**时间基准。** 默认用 Yield 的 `dt`，即 `ControlWith` 文档所说的本周期时长。各驱动目前并不一致（见主会话 C17）：Franka 在第一周期前报告 0，限速会因此拒绝任何变化，测量噪声也不例外；roplat_exrobot 报告会话累计时间，限速会越来越宽。`CommandGuard::with_period(period)` 改用设备的命令周期作基准，不再依赖 Yield 的 `dt`。命令本来就在一个设备周期内执行，这样也更保守。周期是否按时到达交给看门狗检查。契约的含义定下来以后，检查本身不用改。

**参考只在一次会话内有效。** 守卫接受 `done = true` 的命令或拒绝时清空参考；会话因外部停止或其他节点失败而结束时，守卫并不知道。节点看不到 drive 的边界，`dt` 也不能用来判断（rsbullet 第一周期同样报告完整周期）。因此规则是：一次会话用一个实例。同一节律多次 drive 时，在外层周期头段新建守卫（roplat 中"本周期创建的对象属于本周期"），或在复用前调用 `reset()`。

## 5. 新鲜度门与看门狗

**新鲜度门**基于 `roplat_msgs` 的 `Stamp` 和 `Clock`。不同时钟的时刻不可比较，`nanos_since` 返回 `None`，此时直接拒绝，不做猜测。输入可以是图内的 `Stamped<T>`，也可以是 Latest 读出的 `Option<Stamped<T>>`；输出值本身。它与核心 `comm::sample` 的 `Freshness` 不是一回事：后者按序号判断"有没有新版本"，用字符串时钟 `StreamTime`；两套时间词汇有待 E01 与数据流 T01 统一。

**看门狗与节律周期。** `CycleWatchdog` 在控制周期内运行，比较本周期与上周期的间隔，同时检查设备报告的 `dt`（设备可能报告主机时钟看不到的丢周期）。超过 `timeout` 就拒绝本周期，即使命令已经算好也不发出。第一周期只布防，与守卫一样，一次会话一个实例。用仿真时钟（`ManualClock`）时，宿主机卡顿不推进仿真时间，看门狗也就不会触发。这是对的，因为仿真中的物理同样没有前进。

**它管不到永不结束的周期。** 节点阻塞时看门狗节点根本不会运行。`ExecutionContext` 的停止是协作式的：`ControlRhythm` 只在下一次回调前查看停止标志，外部线程调用 `request_stop` 也打断不了阻塞中的周期。这种情况只能靠设备自己的通信看门狗（如 Franka 的丢包超时）。因此 `timeout` 应设在名义周期以上（例如 2～3 倍），同时低于设备看门狗，让软件先给出受控的 `Break`。

跨域心跳不另设构件：对心跳样本的 `Stamp` 用新鲜度门检查即可。

## 6. 急停传播

**已实现（不改契约）：**

- `SafetyLatch`：进程内的锁存停止，clone 后共享同一个状态。`Trip` 节点把任一门的拒绝写入锁存；各域的 `LatchGate` 在下一周期拒绝。锁存跨会话保持，只有操作者调用 `clear()` 才解除。也可以由操作者输入节点直接 `trip()`。
- 同一 System 内的并行兄弟域，在一个分支失败时本来就会被请求协作停止；锁存补上三种情况：独立域之间、跨 drive、人工触发。
- 会话失败、`ControlRhythm` 归还机器人之后，System 尾段可以用普通 Rust 调用 `robot.emergency_stop()`。

它是软件停止，延迟一个周期，不能代替设备的急停回路。

**提议（需要改契约，未实现，待作者裁定）：**

1. 驱动提供可跨线程调用的急停句柄（`Send + Sync`，在会话开始前取得）。会话期间 `ControlRhythm` 独占 `&mut robot`，`Robot::emergency_stop(&mut self)` 在会话中无法调用。
2. 观测携带设备状态（`RobotStatus` 的 `EMERGENCY_STOP`、故障码），图内的门就能把硬件急停或故障写入锁存。现有 `JointState`、`ArmState` 没有状态字段。
3. 区分受控收尾与保护性停止：`ControlStep` 增加第三种出口，或让驱动对安全拒绝选择另一种收尾（例如 Stop Category 1）。目前所有 `Break` 都走驱动的正常收尾。
4. `ExecutionContext` 记录停止原因，兄弟域可以分辨安全停止与正常结束。

## 7. 用法

```rust
control >> |sample| {
    stop?;                        // LatchGate
    sample.1 >> watchdog;         // CycleWatchdog
    watchdog?;
    sample >> servo;
    sample >> guard.input.0;      // CommandGuard<JointPositionCheck<7>>
    servo >> guard.input.1;
    guard >> trip;                // Trip：拒绝也停住其他域
    trip?
};
```

`ArmState` 不是 `Copy`。按 DSL 的规则，要把观测同时交给控制器和守卫，需要一个显式复制的节点（见 `tests/control_graph.rs` 的 `Tee`）。

## 8. 验证与未覆盖

- `tests/control_graph.rs`：14 项。13 项运行真实的 `#[roplat::system]` 图，测试机器人实现了四个通道的 `ControlWith`。其中 11 项是拒绝路径：位置越限、位置步长超速、制动距离不足、力矩超限、力矩跳变、笛卡尔超速、目标超龄、跨时钟、无样本、周期迟到、锁存跨域。每一项都确认设备会话收到 `Break`、违例命令从未发出、drive 返回该违例。另外 2 项是全部放行和 `done` 后参考重置。最后 1 项直接调用节点，验证 `with_period` 能处理第一周期 `dt = 0`。
- `tests/checks.rs`：纯检查的 15 个用例，覆盖 NaN 命令与限值、`dt = 0`、参考来源与清空、跟踪误差、时钟规则、看门狗布防与解除、锁存，以及 `find` 穿透包装错误。
- 未覆盖：rsbullet 仿真（`RsBulletDevice` 尚未合并）、参考机器人（本轮不连真机）、`AsyncControlRhythm`（源码同样把 `Err` 变成 `Break`，未写测试）、hard 节律。

## 9. 待作者裁定

1. 错误传递沿用 `RoplatError::Io`（同 `ControlSessionError`），还是在核心加一个带类型的变体。
2. 参考的生命期：现在是"节点加文档规则"；另一种做法是会话级包装节律（包住 `ControlRhythm`，drive 开始时清空，并在节律层检查最终 Feed，图里无法绕过），代价是检查移出了图。
3. 是否在 `roplat_control` 提供饱和节点；核心是否提供通用的复制节点（`Tee`）。
4. 第 6 节的四项契约提议。
5. 控制回调 `dt` 的含义（C17）定下后，`with_period` 是否保留。
6. crate 名称与位置。
