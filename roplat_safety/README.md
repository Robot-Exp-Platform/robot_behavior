# roplat_safety

[English](README.md) | [简体中文](README_zh.md)

Safety gates for [`roplat_behavior`](../roplat_behavior/README.md) control channels: joint limits, rate limits, sample freshness, a cycle watchdog and a shared stop latch.

Part of the [roplat_behavior workspace](../README.md). 0.9 is not on crates.io yet; build it from source as [Working from source](../README.md#working-from-source) describes.

## How a gate acts

A gate checks and refuses; it never repairs a command into another one. Saturating a command is controller design and belongs upstream, where the saturated command is then checked like any other.

A refusal is a `SafetyViolation`. In a control domain `gate?` turns it into a failed cycle, so the device session receives `Break` and no command, which is the semantics the `ControlStep` contract already gives a domain that cannot produce one.

## Contents

- Command checks per channel, each within its range and rate limits from one command to the next: `JointPositionCheck`, `JointVelocityCheck`, `JointTorqueCheck` and `CartesianVelocityCheck`.
- Sample freshness (`AgeLimit`, `Sampled`) and a cycle watchdog (`Watchdog`).
- A stop latch (`SafetyLatch`) that domains share.
- Feature `roplat`: the nodes `CommandGuard`, `FreshnessGate`, `CycleWatchdog`, `LatchGate` and `Trip`. A `Trip` trips its latch on a refusal, so the domains behind a `LatchGate` stop too.

The checks are plain types usable without the roplat runtime. The [design (zh)](docs/design_zh.md) gives the design, its trade-offs and what is left open.

## Features

| Feature | Purpose |
|---|---|
| `roplat` | Nodes and error transport; the checks themselves need no runtime. |

Default features are empty.

## License

Apache-2.0; see [LICENSE](../LICENSE).
