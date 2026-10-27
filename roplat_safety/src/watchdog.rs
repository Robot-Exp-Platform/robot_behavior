//! Gap between feeds on one clock.

use std::time::Duration;

use roplat_msgs::Stamp;

use crate::SafetyViolation;

/// Trips when more than `timeout` passes between two feeds.
///
/// Like the K1 heartbeat watchdog it reports, and leaves the physical
/// response to whoever consumes the violation. A tripped watchdog disarms;
/// the next feed starts a new watch.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Watchdog {
    timeout: Duration,
    last: Option<Stamp>,
}

impl Watchdog {
    pub const fn new(timeout: Duration) -> Self {
        Self { timeout, last: None }
    }

    pub const fn timeout(&self) -> Duration {
        self.timeout
    }

    /// The gap since the last feed, read at `now`; `Ok` when unarmed.
    pub fn check(&self, now: &Stamp) -> Result<(), SafetyViolation> {
        let Some(last) = &self.last else {
            return Ok(());
        };
        let nanos = now
            .nanos_since(last)
            .ok_or(SafetyViolation::ClockMismatch { sample: last.clock, gate: now.clock })?;
        let gap = Duration::from_nanos(nanos.unsigned_abs());
        if nanos < 0 {
            return Err(SafetyViolation::FromFuture { ahead: gap });
        }
        if gap > self.timeout {
            return Err(SafetyViolation::Overrun { gap, max: self.timeout });
        }
        Ok(())
    }

    /// Checks the gap, then records a feed at `now`.
    pub fn feed(&mut self, now: Stamp) -> Result<(), SafetyViolation> {
        let result = self.check(&now);
        self.last = result.is_ok().then_some(now);
        result
    }

    pub fn reset(&mut self) {
        self.last = None;
    }
}
