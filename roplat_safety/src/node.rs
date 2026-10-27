//! Thin roplat nodes over the checks.
//!
//! Each refusing node outputs `Result<_, SafetyViolation>`. Written as
//! `node?` in a control domain, a refusal fails the cycle: `ControlRhythm`
//! sends no command for it, breaks the device session and returns the
//! violation as the drive's error.

use std::{marker::PhantomData, time::Duration};

use roplat::{Lifecycle, Node, RoplatError};
use roplat_msgs::Clock;

use crate::{AgeLimit, CommandCheck, SafetyLatch, SafetyViolation, Sampled, Watchdog};

/// Passes the controller's `(command, done)` on only if its check accepts it.
///
/// Input is the control Yield `(obs, dt)` and that Feed. The check's
/// reference lasts one session: an accepted `done` or a refusal clears it. A
/// session that ends otherwise (a stop, another node's failure) leaves it, so
/// a guard reused for a later session is built anew in the enclosing cycle or
/// [`reset`](Self::reset) first.
///
/// Rates are measured over the Yield's `dt` unless a fixed period is set.
#[derive(Clone, Debug)]
pub struct CommandGuard<C> {
    check: C,
    period: Option<Duration>,
}

impl<C: CommandCheck> CommandGuard<C> {
    pub fn new(check: C) -> Self {
        Self { check, period: None }
    }

    /// Measures rates over the device's command period instead of the
    /// Yield's `dt`, which drivers do not yet report alike: Franka reports
    /// zero before the first cycle, roplat_exrobot the session time.
    pub fn with_period(mut self, period: Duration) -> Self {
        self.period = Some(period);
        self
    }

    pub fn check(&self) -> &C {
        &self.check
    }

    pub fn reset(&mut self) {
        self.check.reset();
    }
}

impl<C> Lifecycle for CommandGuard<C> {
    type Error = RoplatError;
}

impl<C> Node for CommandGuard<C>
where
    C: CommandCheck + Send + Sync,
    C::Obs: Send,
    C::Command: Send,
{
    type Resources = ();
    type Input = ((C::Obs, Duration), (C::Command, bool));
    type Output = Result<(C::Command, bool), SafetyViolation>;

    async fn process(&mut self, ((obs, dt), (command, done)): Self::Input, _: ()) -> Self::Output {
        self.check
            .check(&obs, self.period.unwrap_or(dt), &command)?;
        if done {
            self.check.reset();
        }
        Ok((command, done))
    }
}

/// Passes a sample's value on only if it is fresh on the gate's clock; see
/// [`AgeLimit`]. Reads a `Stamped<T>`, or the `Option` from a Latest.
#[derive(Clone, Debug)]
pub struct FreshnessGate<I, C> {
    clock: C,
    limit: AgeLimit,
    _input: PhantomData<fn(I)>,
}

impl<I, C: Clock> FreshnessGate<I, C> {
    pub fn new(clock: C, max_age: Duration) -> Self {
        Self { clock, limit: AgeLimit::new(max_age), _input: PhantomData }
    }
}

impl<I, C> Lifecycle for FreshnessGate<I, C> {
    type Error = RoplatError;
}

impl<I, C> Node for FreshnessGate<I, C>
where
    I: Sampled + Send,
    I::Value: Send,
    C: Clock,
{
    type Resources = ();
    type Input = I;
    type Output = Result<I::Value, SafetyViolation>;

    async fn process(&mut self, sample: I, _: ()) -> Self::Output {
        let sample = sample.into_sample().ok_or(SafetyViolation::NoSample)?;
        self.limit.check(&sample.header.stamp, &self.clock.now())?;
        Ok(sample.value)
    }
}

/// Refuses a cycle that comes more than `timeout` after the previous one,
/// on its clock or by the device's own `dt`.
///
/// Input is the cycle `dt` of the control Yield. Running inside the cycle, it
/// stops a late cycle from commanding; a cycle that never finishes is the
/// device's own watchdog's to catch. The first cycle arms it, so it lasts one
/// session like [`CommandGuard`].
#[derive(Clone, Debug)]
pub struct CycleWatchdog<C> {
    clock: C,
    watchdog: Watchdog,
}

impl<C: Clock> CycleWatchdog<C> {
    pub fn new(clock: C, timeout: Duration) -> Self {
        Self { clock, watchdog: Watchdog::new(timeout) }
    }

    pub fn reset(&mut self) {
        self.watchdog.reset();
    }
}

impl<C> Lifecycle for CycleWatchdog<C> {
    type Error = RoplatError;
}

impl<C: Clock> Node for CycleWatchdog<C> {
    type Resources = ();
    type Input = Duration;
    type Output = Result<(), SafetyViolation>;

    async fn process(&mut self, dt: Duration, _: ()) -> Self::Output {
        let max = self.watchdog.timeout();
        if dt > max {
            self.watchdog.reset();
            return Err(SafetyViolation::Overrun { gap: dt, max });
        }
        self.watchdog.feed(self.clock.now())
    }
}

/// Refuses every cycle while its latch is tripped.
#[derive(Clone, Debug)]
pub struct LatchGate {
    latch: SafetyLatch,
}

impl LatchGate {
    pub fn new(latch: SafetyLatch) -> Self {
        Self { latch }
    }
}

impl Lifecycle for LatchGate {
    type Error = RoplatError;
}

impl Node for LatchGate {
    type Resources = ();
    type Input = ();
    type Output = Result<(), SafetyViolation>;

    async fn process(&mut self, (): (), _: ()) -> Self::Output {
        self.latch.check()
    }
}

/// Trips its latch on a refusal and passes the result on unchanged, so the
/// domains behind a [`LatchGate`] stop too.
#[derive(Clone, Debug)]
pub struct Trip<T> {
    latch: SafetyLatch,
    _value: PhantomData<fn() -> T>,
}

impl<T> Trip<T> {
    pub fn new(latch: SafetyLatch) -> Self {
        Self { latch, _value: PhantomData }
    }
}

impl<T> Lifecycle for Trip<T> {
    type Error = RoplatError;
}

impl<T: Send> Node for Trip<T> {
    type Resources = ();
    type Input = Result<T, SafetyViolation>;
    type Output = Result<T, SafetyViolation>;

    async fn process(&mut self, result: Self::Input, _: ()) -> Self::Output {
        if let Err(violation) = &result {
            self.latch.trip(violation.to_string());
        }
        result
    }
}
