//! Pass-through nodes that feed a check from inside a graph.
//!
//! The node owns nothing the graph needs: it returns its input unchanged,
//! so it can sit on any edge. The check lives behind a [`ProbeHandle`]
//! that code outside the graph (a status task, a Rerun logger, a test)
//! reads while the graph runs.

use std::{
    marker::PhantomData,
    sync::{Arc, Mutex, MutexGuard},
};

use roplat::{Lifecycle, Node, RoplatError};
use roplat_msgs::{Clock, Stamped};

use crate::{FrequencyCheck, LatencyCheck};

/// Shared access to the check a node feeds.
#[derive(Debug)]
pub struct ProbeHandle<K> {
    check: Arc<Mutex<K>>,
}

impl<K> Clone for ProbeHandle<K> {
    fn clone(&self) -> Self {
        Self { check: Arc::clone(&self.check) }
    }
}

impl<K> ProbeHandle<K> {
    fn new(check: K) -> Self {
        Self { check: Arc::new(Mutex::new(check)) }
    }

    /// The check, locked; a panic while it was held does not hide it.
    pub fn lock(&self) -> MutexGuard<'_, K> {
        self.check
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }
}

/// Ticks a [`FrequencyCheck`] on its clock each time a value passes.
#[derive(Debug)]
pub struct ProbeNode<T, C> {
    clock: C,
    handle: ProbeHandle<FrequencyCheck>,
    _value: PhantomData<fn(T) -> T>,
}

impl<T, C: Clock> ProbeNode<T, C> {
    /// The node and the handle to its check.
    pub fn new(clock: C, check: FrequencyCheck) -> (Self, ProbeHandle<FrequencyCheck>) {
        let handle = ProbeHandle::new(check);
        (
            Self { clock, handle: handle.clone(), _value: PhantomData },
            handle,
        )
    }
}

impl<T, C> Lifecycle for ProbeNode<T, C> {
    type Error = RoplatError;
}

impl<T: Send, C: Clock> Node for ProbeNode<T, C> {
    type Resources = ();
    type Input = T;
    type Output = T;

    async fn process(&mut self, value: T, _: ()) -> T {
        let now = self.clock.now();
        self.handle.lock().tick(&now);
        value
    }
}

/// Records how old each stamped sample is when it passes, into a
/// [`LatencyCheck`] on its clock. The sample's stamp must be on the same
/// clock; one that is not is counted as rejected.
#[derive(Debug)]
pub struct LatencyNode<T, C> {
    clock: C,
    handle: ProbeHandle<LatencyCheck>,
    _value: PhantomData<fn(T) -> T>,
}

impl<T, C: Clock> LatencyNode<T, C> {
    pub fn new(clock: C, check: LatencyCheck) -> (Self, ProbeHandle<LatencyCheck>) {
        let handle = ProbeHandle::new(check);
        (
            Self { clock, handle: handle.clone(), _value: PhantomData },
            handle,
        )
    }
}

impl<T, C> Lifecycle for LatencyNode<T, C> {
    type Error = RoplatError;
}

impl<T: Send, C: Clock> Node for LatencyNode<T, C> {
    type Resources = ();
    type Input = Stamped<T>;
    type Output = Stamped<T>;

    async fn process(&mut self, sample: Stamped<T>, _: ()) -> Stamped<T> {
        let now = self.clock.now();
        self.handle.lock().record(&sample.header.stamp, &now);
        sample
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use roplat_msgs::{Header, ManualClock};

    use super::*;
    use crate::Level;

    #[tokio::test(flavor = "current_thread")]
    async fn probe_node_passes_values_and_ticks_on_its_clock() {
        let clock = Arc::new(ManualClock::simulation());
        let check = FrequencyCheck::new("loop", 100.0..=100.0, 0.1, Duration::from_millis(50));
        let (mut node, handle) = ProbeNode::<u32, _>::new(Arc::clone(&clock), check);
        for k in 0..30 {
            assert_eq!(node.process(k, ()).await, k);
            clock.advance(Duration::from_millis(10));
        }
        let status = handle.lock().status(&clock.now());
        assert_eq!(status.level, Level::Ok, "{status}");
        // The clock moves on with nothing passing: stale.
        clock.advance(Duration::from_millis(100));
        assert_eq!(handle.lock().status(&clock.now()).level, Level::Stale);
    }

    #[tokio::test(flavor = "current_thread")]
    async fn latency_node_measures_sample_age() {
        let clock = Arc::new(ManualClock::simulation());
        let check = LatencyCheck::new(
            "camera",
            Duration::from_millis(20),
            Duration::from_millis(50),
        );
        let (mut node, handle) = LatencyNode::<f64, _>::new(Arc::clone(&clock), check);
        for _ in 0..10 {
            let header = Header { stamp: clock.now(), ..Header::default() };
            clock.advance(Duration::from_millis(30));
            let out = node.process(Stamped::new(header, 1.5), ()).await;
            assert_eq!(out.value, 1.5);
        }
        let status = handle.lock().status();
        assert_eq!(status.level, Level::Warn, "{status}");
        assert_eq!(status.value("samples"), Some("10"));
    }
}
