//! Runtime observation: how often things happen, how late data is, and
//! whether that is healthy.
//!
//! This covers the roles of `ros2 topic hz` / `delay` and of
//! `diagnostic_updater` / `diagnostic_aggregator`:
//!
//! - **Probes** measure. [`RateProbe`] keeps the last intervals between
//!   events and gives their [`Stats`] (rate, mean, spread, p99, worst);
//!   [`LatencyProbe`] does the same for the age of samples when they are
//!   used.
//! - **Checks** judge. [`FrequencyCheck`] and [`LatencyCheck`] turn probes
//!   into a [`Status`] with a [`Level`] and a message, with the thresholds
//!   ROS's `FrequencyStatus` uses: no events is an error, a rate outside
//!   the expected band with tolerance is a warning, silence longer than a
//!   timeout is stale.
//! - **Collection.** An [`Updater`] polls named checks; an [`Aggregator`]
//!   groups statuses by name prefix, the worst level winning.
//!
//! Everything is measured in [`Stamp`]s from a
//! [`Clock`](roplat_msgs::Clock), so the same checks run on wall,
//! simulation or replay time. Probes are plain values with no runtime;
//! with the `roplat` feature, [`ProbeNode`] and [`LatencyNode`] pass graph
//! data through unchanged while feeding a check that code outside the graph
//! reads through a [`ProbeHandle`].
//!
//! ```
//! use std::time::Duration;
//! use roplat_diagnostics::{FrequencyCheck, Level};
//! use roplat_msgs::Stamp;
//!
//! // A 100 Hz loop, judged within 10 %, stale after 50 ms of silence.
//! let mut check = FrequencyCheck::new("arm/state", 100.0..=100.0, 0.1, Duration::from_millis(50));
//! for k in 0..20 {
//!     check.tick(&Stamp::new(0, k * 10_000_000));
//! }
//! let status = check.status(&Stamp::new(0, 200_000_000));
//! assert_eq!(status.level, Level::Ok);
//! // Nothing for 0.2 s: stale.
//! assert_eq!(check.status(&Stamp::new(0, 400_000_000)).level, Level::Stale);
//! ```

mod check;
#[cfg(feature = "roplat")]
mod node;
mod probe;
mod status;

pub use check::{FrequencyCheck, LatencyCheck};
#[cfg(feature = "roplat")]
pub use node::{LatencyNode, ProbeHandle, ProbeNode};
pub use probe::{LatencyProbe, RateProbe, Stats};
pub use roplat_msgs::Stamp;
pub use status::{Aggregator, Level, Status, Updater};
