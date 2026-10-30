# roplat_diagnostics

[English](README.md) | [简体中文](README_zh.md)

运行观测：事情发生得多频繁、数据有多迟，以及这是否正常。它承担 `ros2 topic hz`／`delay` 与 `diagnostic_updater`／`diagnostic_aggregator` 的角色。

属于 [roplat_behavior 工作区](../README_zh.md)。0.9 尚未发布到 crates.io，请按[从源码开发](../README_zh.md#从源码开发)从源码构建。

## 内容

- **探针**负责测量。`RateProbe` 保存最近若干次事件的间隔，给出其 `Stats`（频率、均值、离散度、p99、最坏值）；`LatencyProbe` 对样本被使用时的年龄做同样的统计。
- **检查**负责判断。`FrequencyCheck` 与 `LatencyCheck` 把探针结果变成带 `Level`（OK、WARN、ERROR、STALE）与说明的 `Status`，阈值同 ROS 的 `FrequencyStatus`：没有事件为错误，频率超出期望范围加容差为警告，静默超过超时为过期。
- **汇总。** `Updater` 轮询具名的检查；`Aggregator` 按名称前缀分组，取最差的级别。

一切都以 `roplat_msgs::Clock` 给出的 `Stamp` 计量，同样的检查可用于墙钟、仿真或回放时间。探针是不依赖运行时的普通值；开启 `roplat` feature 后，`ProbeNode` 与 `LatencyNode` 原样传递图中数据，同时喂给一个检查，图外代码通过 `ProbeHandle` 读取。

## 示例

```rust
use std::time::Duration;
use roplat_diagnostics::{FrequencyCheck, Level};
use roplat_msgs::Stamp;

// A 100 Hz loop, judged within 10 %, stale after 50 ms of silence.
let mut check = FrequencyCheck::new("arm/state", 100.0..=100.0, 0.1, Duration::from_millis(50));
for k in 0..20 {
    check.tick(&Stamp::new(0, k * 10_000_000));
}
let status = check.status(&Stamp::new(0, 200_000_000));
assert_eq!(status.level, Level::Ok);
// Nothing for 0.2 s: stale.
assert_eq!(check.status(&Stamp::new(0, 400_000_000)).level, Level::Stale);
```

## Feature

| Feature | 用途 |
|---|---|
| `roplat` | `ProbeNode` 与 `LatencyNode`，喂给检查的直通节点；探针与检查本身不需要运行时。 |

默认 feature 为空。

## 许可证

Apache-2.0，见 [LICENSE](../LICENSE)。
