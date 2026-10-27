//! Sample age on one clock.

use std::time::Duration;

use roplat_msgs::{Stamp, Stamped, clock};

use crate::SafetyViolation;

/// Refuses samples older than `max_age`, stamped on another clock, or
/// stamped ahead of `now`.
///
/// Instants on different clocks do not compare, so a sample from another
/// clock, or one whose producer left the clock unspecified, is refused
/// rather than guessed at.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AgeLimit {
    pub max_age: Duration,
}

impl AgeLimit {
    pub const fn new(max_age: Duration) -> Self {
        Self { max_age }
    }

    /// The age of a sample stamped `stamp`, read at `now`.
    pub fn check(&self, stamp: &Stamp, now: &Stamp) -> Result<Duration, SafetyViolation> {
        let mismatch = || SafetyViolation::ClockMismatch { sample: stamp.clock, gate: now.clock };
        if stamp.clock == clock::UNSPECIFIED {
            return Err(mismatch());
        }
        let nanos = now.nanos_since(stamp).ok_or_else(mismatch)?;
        let age = Duration::from_nanos(nanos.unsigned_abs());
        if nanos < 0 {
            return Err(SafetyViolation::FromFuture { ahead: age });
        }
        if age > self.max_age {
            return Err(SafetyViolation::Stale { age, max_age: self.max_age });
        }
        Ok(age)
    }
}

/// What a freshness gate reads: a stamped value, or the `Option` a Latest
/// read gives before anything has been published.
pub trait Sampled {
    type Value;

    fn into_sample(self) -> Option<Stamped<Self::Value>>;
}

impl<T> Sampled for Stamped<T> {
    type Value = T;

    fn into_sample(self) -> Option<Stamped<T>> {
        Some(self)
    }
}

impl<T> Sampled for Option<Stamped<T>> {
    type Value = T;

    fn into_sample(self) -> Option<Stamped<T>> {
        self
    }
}
