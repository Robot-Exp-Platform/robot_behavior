# roplat_diagnostics

[English](README.md) | [简体中文](README_zh.md)

Runtime observation: how often things happen, how late data is, and whether that is healthy. It covers the roles of `ros2 topic hz` / `delay` and of `diagnostic_updater` / `diagnostic_aggregator`.

Part of the [roplat_behavior workspace](../README.md). 0.9 is not on crates.io yet; build it from source as [Working from source](../README.md#working-from-source) describes.

## Contents

- **Probes** measure. `RateProbe` keeps the last intervals between events and gives their `Stats` (rate, mean, spread, p99, worst); `LatencyProbe` does the same for the age of samples when they are used.
- **Checks** judge. `FrequencyCheck` and `LatencyCheck` turn probes into a `Status` with a `Level` (OK, WARN, ERROR, STALE) and a message, with the thresholds ROS's `FrequencyStatus` uses: no events is an error, a rate outside the expected band with tolerance is a warning, silence longer than a timeout is stale.
- **Collection.** An `Updater` polls named checks; an `Aggregator` groups statuses by name prefix, the worst level winning.

Everything is measured in `Stamp`s from a `roplat_msgs::Clock`, so the same checks run on wall, simulation or replay time. Probes are plain values with no runtime; with the `roplat` feature, `ProbeNode` and `LatencyNode` pass graph data through unchanged while feeding a check that code outside the graph reads through a `ProbeHandle`.

## Example

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

## Features

| Feature | Purpose |
|---|---|
| `roplat` | `ProbeNode` and `LatencyNode`, pass-through nodes that feed a check; the probes and checks themselves need no runtime. |

Default features are empty.

## License

Apache-2.0; see [LICENSE](../LICENSE).
