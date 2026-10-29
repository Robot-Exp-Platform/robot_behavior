//! Safety gates for `roplat_behavior` control channels: joint limits, rate
//! limits, sample freshness, a cycle watchdog and a shared stop latch.
//!
//! A gate checks and refuses; it never repairs a command into another one.
//! Saturating a command is controller design and belongs upstream, where the
//! saturated command is then checked like any other. A refusal is a
//! [`SafetyViolation`]; in a control domain `gate?` turns it into a failed
//! cycle, so the device session receives `Break` and no command, which is
//! the semantics the `ControlStep` contract already gives a domain that cannot
//! produce one.
//!
//! The checks are plain types usable without the roplat runtime; the
//! `roplat` feature adds the nodes in `node`. The design, its
//! trade-offs and what is left open are in `docs/design_zh.md`.

mod bound;
mod cartesian;
mod fresh;
mod joint;
mod latch;
#[cfg(feature = "roplat")]
pub mod node;
mod violation;
mod watchdog;

use std::time::Duration;

pub use cartesian::CartesianVelocityCheck;
pub use fresh::{AgeLimit, Sampled};
pub use joint::{JointPositionCheck, JointTorqueCheck, JointVelocityCheck};
pub use latch::SafetyLatch;
#[cfg(feature = "roplat")]
pub use node::{CommandGuard, CycleWatchdog, FreshnessGate, LatchGate, Trip};
pub use violation::{Axis, Quantity, SafetyViolation};
pub use watchdog::Watchdog;

/// Checks the commands of one control channel against the previous command
/// and the cycle's observation.
pub trait CommandCheck {
    type Obs;
    type Command;

    /// Checks `command` for a cycle that observed `obs` after `dt`. An
    /// accepted command becomes the reference for the next check; a refusal
    /// clears the reference.
    fn check(
        &mut self,
        obs: &Self::Obs,
        dt: Duration,
        command: &Self::Command,
    ) -> Result<(), SafetyViolation>;

    /// Forgets the previous command; the next check starts from the
    /// observation, as at the start of a session.
    fn reset(&mut self);
}
