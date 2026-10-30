//! Checks: probes judged against expectations.

use std::{ops::RangeInclusive, time::Duration};

use roplat_msgs::Stamp;

use crate::{LatencyProbe, Level, RateProbe, Status};

/// Is something happening at the expected rate? The thresholds follow
/// ROS's `FrequencyStatus`:
///
/// - no interval measured yet: [`Level::Error`], "no events";
/// - silence longer than `stale_after`: [`Level::Stale`];
/// - mean rate below `min × (1 − tolerance)` or above
///   `max × (1 + tolerance)`: [`Level::Warn`];
/// - otherwise [`Level::Ok`].
///
/// The status reports the rate and the worst and p99 intervals, which show
/// jitter and overruns that a mean rate hides.
#[derive(Clone, Debug)]
pub struct FrequencyCheck {
    name: String,
    expected: RangeInclusive<f64>,
    tolerance: f64,
    stale_after: Duration,
    probe: RateProbe,
}

impl FrequencyCheck {
    /// `expected` in Hz; `tolerance` as a fraction (0.1 is 10 %). The
    /// window is 100 intervals; see [`with_window`](Self::with_window).
    pub fn new(
        name: &str,
        expected: RangeInclusive<f64>,
        tolerance: f64,
        stale_after: Duration,
    ) -> Self {
        Self {
            name: name.to_owned(),
            expected,
            tolerance,
            stale_after,
            probe: RateProbe::new(100),
        }
    }

    pub fn with_window(mut self, window: usize) -> Self {
        self.probe = RateProbe::new(window);
        self
    }

    pub fn tick(&mut self, at: &Stamp) {
        self.probe.tick(at);
    }

    pub fn probe(&self) -> &RateProbe {
        &self.probe
    }

    pub fn reset(&mut self) {
        self.probe.reset();
    }

    /// The verdict at `now`.
    pub fn status(&self, now: &Stamp) -> Status {
        let Some(stats) = self.probe.stats() else {
            return Status::new(&self.name, Level::Error, "no events")
                .with("events", self.probe.events());
        };
        let rate = stats.rate();
        let with_values = |status: Status| {
            status
                .with("rate_hz", format!("{rate:.2}"))
                .with("mean_period", format!("{:?}", stats.mean))
                .with("p99_period", format!("{:?}", stats.p99))
                .with("max_period", format!("{:?}", stats.max))
                .with("events", self.probe.events())
        };
        if let Some(silence) = self.probe.silence(now)
            && silence > self.stale_after
        {
            return with_values(Status::new(
                &self.name,
                Level::Stale,
                format!("nothing for {silence:?}"),
            ));
        }
        let low = self.expected.start() * (1.0 - self.tolerance);
        let high = self.expected.end() * (1.0 + self.tolerance);
        let (level, message) = if rate < low {
            (Level::Warn, format!("rate {rate:.1} Hz below {low:.1} Hz"))
        } else if rate > high {
            (Level::Warn, format!("rate {rate:.1} Hz above {high:.1} Hz"))
        } else {
            (Level::Ok, format!("rate {rate:.1} Hz"))
        };
        with_values(Status::new(&self.name, level, message))
    }
}

/// Is data fresh when used? Judges the p99 age of samples: above `warn`
/// is [`Level::Warn`], above `error` is [`Level::Error`]; no samples is
/// [`Level::Stale`].
#[derive(Clone, Debug)]
pub struct LatencyCheck {
    name: String,
    warn: Duration,
    error: Duration,
    probe: LatencyProbe,
}

impl LatencyCheck {
    pub fn new(name: &str, warn: Duration, error: Duration) -> Self {
        Self {
            name: name.to_owned(),
            warn,
            error,
            probe: LatencyProbe::new(100),
        }
    }

    pub fn with_window(mut self, window: usize) -> Self {
        self.probe = LatencyProbe::new(window);
        self
    }

    /// Records a sample stamped `stamp` used at `now`.
    pub fn record(&mut self, stamp: &Stamp, now: &Stamp) {
        self.probe.record(stamp, now);
    }

    pub fn probe(&self) -> &LatencyProbe {
        &self.probe
    }

    pub fn reset(&mut self) {
        self.probe.reset();
    }

    pub fn status(&self) -> Status {
        let Some(stats) = self.probe.stats() else {
            return Status::new(&self.name, Level::Stale, "no samples");
        };
        let level = if stats.p99 > self.error {
            Level::Error
        } else if stats.p99 > self.warn {
            Level::Warn
        } else {
            Level::Ok
        };
        Status::new(&self.name, level, format!("p99 age {:?}", stats.p99))
            .with("mean_age", format!("{:?}", stats.mean))
            .with("max_age", format!("{:?}", stats.max))
            .with("samples", stats.count)
            .with("rejected", self.probe.rejected())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: i64 = 1_000_000;

    fn at(ms: i64) -> Stamp {
        Stamp::new(0, ms * MS)
    }

    fn ticked(periods_ms: &[i64]) -> FrequencyCheck {
        let mut check = FrequencyCheck::new("loop", 100.0..=100.0, 0.1, Duration::from_millis(50));
        let mut t = 0;
        check.tick(&at(t));
        for p in periods_ms {
            t += p;
            check.tick(&at(t));
        }
        check
    }

    #[test]
    fn frequency_levels() {
        // No intervals yet.
        assert_eq!(ticked(&[]).status(&at(0)).level, Level::Error);
        // 100 Hz.
        let ok = ticked(&[10; 20]);
        let status = ok.status(&at(200));
        assert_eq!(status.level, Level::Ok);
        assert_eq!(status.value("rate_hz"), Some("100.00"));
        // 80 Hz is below 90 Hz.
        assert_eq!(ticked(&[12, 13, 12, 13]).status(&at(50)).level, Level::Warn);
        // 125 Hz is above 110 Hz.
        assert_eq!(ticked(&[8; 10]).status(&at(80)).level, Level::Warn);
        // Silent for 100 ms after the last tick at 200 ms.
        assert_eq!(ok.status(&at(300)).level, Level::Stale);
    }

    #[test]
    fn frequency_reports_overruns_a_mean_hides() {
        // 19 periods of 9.5 ms and one of 19.5 ms: mean 10 ms, worst 19.5.
        let mut periods = vec![9_500_000; 19];
        periods.push(19_500_000);
        let mut check = FrequencyCheck::new("loop", 100.0..=100.0, 0.1, Duration::from_millis(50));
        let mut t = 0;
        check.tick(&Stamp::new(0, t));
        for p in periods {
            t += p;
            check.tick(&Stamp::new(0, t));
        }
        let status = check.status(&Stamp::new(0, t));
        assert_eq!(status.level, Level::Ok);
        assert_eq!(status.value("max_period"), Some("19.5ms"));
    }

    #[test]
    fn latency_levels_use_p99() {
        let mut check = LatencyCheck::new(
            "camera",
            Duration::from_millis(20),
            Duration::from_millis(50),
        );
        assert_eq!(check.status().level, Level::Stale);
        for k in 0..100 {
            check.record(&at(k * 10), &at(k * 10 + 5));
        }
        assert_eq!(check.status().level, Level::Ok);
        // Two late samples in the last 100 move p99 to 30 ms.
        check.record(&at(2000), &at(2030));
        check.record(&at(2010), &at(2040));
        assert_eq!(check.status().level, Level::Warn);
        check.record(&at(2020), &at(2100));
        check.record(&at(2030), &at(2110));
        assert_eq!(check.status().level, Level::Error);
    }
}
