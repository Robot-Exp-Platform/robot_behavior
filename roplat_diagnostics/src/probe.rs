//! Probes: windows of intervals or ages and their statistics.

use std::{collections::VecDeque, time::Duration};

use roplat_msgs::Stamp;

/// Statistics of a window of durations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stats {
    pub count: usize,
    pub mean: Duration,
    pub min: Duration,
    pub max: Duration,
    /// Population standard deviation.
    pub std_dev: Duration,
    /// 99th percentile, nearest rank.
    pub p99: Duration,
}

impl Stats {
    /// `None` for an empty window.
    pub fn of(window: impl IntoIterator<Item = Duration>) -> Option<Self> {
        let mut nanos: Vec<f64> = window.into_iter().map(|d| d.as_nanos() as f64).collect();
        if nanos.is_empty() {
            return None;
        }
        let count = nanos.len();
        let mean = nanos.iter().sum::<f64>() / count as f64;
        let variance = nanos.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / count as f64;
        nanos.sort_by(f64::total_cmp);
        // Nearest rank: the smallest value with at least 99 % at or below it.
        let rank = ((0.99 * count as f64).ceil() as usize).clamp(1, count);
        let duration = |x: f64| Duration::from_nanos(x.round().max(0.0) as u64);
        Some(Self {
            count,
            mean: duration(mean),
            min: duration(nanos[0]),
            max: duration(nanos[count - 1]),
            std_dev: duration(variance.sqrt()),
            p99: duration(nanos[rank - 1]),
        })
    }

    /// Events per second for a window of intervals.
    pub fn rate(&self) -> f64 {
        let mean = self.mean.as_secs_f64();
        if mean > 0.0 {
            1.0 / mean
        } else {
            f64::INFINITY
        }
    }
}

/// The last `window` intervals between events.
#[derive(Clone, Debug)]
pub struct RateProbe {
    capacity: usize,
    intervals: VecDeque<Duration>,
    last: Option<Stamp>,
    /// Events whose stamp was on another clock or earlier than the last.
    rejected: u64,
    events: u64,
}

impl RateProbe {
    /// Keeps the last `window` intervals (at least 1).
    pub fn new(window: usize) -> Self {
        let capacity = window.max(1);
        Self {
            capacity,
            intervals: VecDeque::with_capacity(capacity),
            last: None,
            rejected: 0,
            events: 0,
        }
    }

    /// Records an event at `at`. An event on another clock than the first,
    /// or earlier than the previous one, is counted in
    /// [`rejected`](Self::rejected) and otherwise ignored.
    pub fn tick(&mut self, at: &Stamp) {
        if let Some(last) = self.last {
            match at.nanos_since(&last) {
                Some(nanos) if nanos >= 0 => {
                    if self.intervals.len() == self.capacity {
                        self.intervals.pop_front();
                    }
                    self.intervals.push_back(Duration::from_nanos(nanos as u64));
                }
                _ => {
                    self.rejected += 1;
                    return;
                }
            }
        }
        self.last = Some(*at);
        self.events += 1;
    }

    /// Statistics of the intervals in the window; `None` before two events.
    pub fn stats(&self) -> Option<Stats> {
        Stats::of(self.intervals.iter().copied())
    }

    /// The last accepted event.
    pub fn last(&self) -> Option<Stamp> {
        self.last
    }

    /// Time since the last accepted event, if `now` is on its clock.
    pub fn silence(&self, now: &Stamp) -> Option<Duration> {
        let nanos = now.nanos_since(&self.last?)?;
        Some(Duration::from_nanos(nanos.max(0) as u64))
    }

    /// Accepted events since creation or [`reset`](Self::reset).
    pub fn events(&self) -> u64 {
        self.events
    }

    pub fn rejected(&self) -> u64 {
        self.rejected
    }

    pub fn reset(&mut self) {
        self.intervals.clear();
        self.last = None;
        self.rejected = 0;
        self.events = 0;
    }
}

/// The last `window` ages of samples: the time from a sample's stamp to
/// when it was used.
#[derive(Clone, Debug)]
pub struct LatencyProbe {
    capacity: usize,
    ages: VecDeque<Duration>,
    rejected: u64,
}

impl LatencyProbe {
    pub fn new(window: usize) -> Self {
        let capacity = window.max(1);
        Self {
            capacity,
            ages: VecDeque::with_capacity(capacity),
            rejected: 0,
        }
    }

    /// Records a sample stamped `stamp` used at `now`. A stamp on another
    /// clock, or later than `now`, is counted in
    /// [`rejected`](Self::rejected).
    pub fn record(&mut self, stamp: &Stamp, now: &Stamp) {
        match now.nanos_since(stamp) {
            Some(nanos) if nanos >= 0 => {
                if self.ages.len() == self.capacity {
                    self.ages.pop_front();
                }
                self.ages.push_back(Duration::from_nanos(nanos as u64));
            }
            _ => self.rejected += 1,
        }
    }

    pub fn stats(&self) -> Option<Stats> {
        Stats::of(self.ages.iter().copied())
    }

    pub fn rejected(&self) -> u64 {
        self.rejected
    }

    pub fn reset(&mut self) {
        self.ages.clear();
        self.rejected = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: i64 = 1_000_000;

    #[test]
    fn stats_of_a_known_window() {
        let ms = Duration::from_millis;
        let s = Stats::of([ms(10), ms(10), ms(10), ms(30)]).unwrap();
        assert_eq!(s.count, 4);
        assert_eq!(s.mean, ms(15));
        assert_eq!(s.min, ms(10));
        assert_eq!(s.max, ms(30));
        // Deviations -5, -5, -5, 15 ms: variance 75 ms², sigma ~8.66 ms.
        assert!((s.std_dev.as_secs_f64() - 75.0_f64.sqrt() / 1000.0).abs() < 1e-6);
        assert_eq!(s.p99, ms(30));
        assert!((s.rate() - 1000.0 / 15.0).abs() < 1e-9);
        assert!(Stats::of([]).is_none());
    }

    #[test]
    fn rate_probe_keeps_a_window_and_rejects_bad_stamps() {
        let mut probe = RateProbe::new(3);
        assert!(probe.stats().is_none());
        for t in [0, 10, 20, 30, 50] {
            probe.tick(&Stamp::new(1, t * MS));
        }
        // Window of the last 3 intervals: 10, 10, 20 ms.
        let s = probe.stats().unwrap();
        assert_eq!(s.count, 3);
        assert_eq!(s.max, Duration::from_millis(20));
        // Backwards in time and another clock are rejected.
        probe.tick(&Stamp::new(1, 40 * MS));
        probe.tick(&Stamp::new(2, 60 * MS));
        assert_eq!(probe.rejected(), 2);
        assert_eq!(probe.events(), 5);
        assert_eq!(
            probe.silence(&Stamp::new(1, 80 * MS)),
            Some(Duration::from_millis(30))
        );
        assert_eq!(probe.silence(&Stamp::new(2, 80 * MS)), None);
    }

    #[test]
    fn latency_probe_measures_age_at_use() {
        let mut probe = LatencyProbe::new(10);
        probe.record(&Stamp::new(0, 100 * MS), &Stamp::new(0, 103 * MS));
        probe.record(&Stamp::new(0, 110 * MS), &Stamp::new(0, 115 * MS));
        // A stamp from the future is rejected.
        probe.record(&Stamp::new(0, 130 * MS), &Stamp::new(0, 120 * MS));
        let s = probe.stats().unwrap();
        assert_eq!(
            (s.min, s.max),
            (Duration::from_millis(3), Duration::from_millis(5))
        );
        assert_eq!(probe.rejected(), 1);
    }
}
