use std::future::Future;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use crossbeam_queue::ArrayQueue;
use roplat::RoplatError;
use roplat::rhythm::Rhythm;

use crate::{
    ArmImpedance, ArmRealtimeControl, ArmState, ControlType, MotionType, Pose, RobotException,
};

// ── ArmControlRhythm ──────────────────────────────────────────────────────
// Wraps ArmRealtimeControl::control_with_closure as a Rhythm source.
// Each rhythm tick corresponds to one realtime control cycle.
// Yield = (ArmState<N>, Duration),  Feed = (ControlType<N>, bool)

const CONTROL_QUEUE_CAPACITY: usize = 8;
const MAX_COMMAND_AGE_TICKS: u64 = (CONTROL_QUEUE_CAPACITY - 1) as u64;
const CONTROL_MAILBOX_POLL_INTERVAL: Duration = Duration::from_micros(50);
const CONTROL_CALLBACK_STALL_TIMEOUT: Duration = Duration::from_millis(100);

/// One command application observed at the realtime callback boundary.
#[derive(Debug, Clone)]
pub struct ControlApplicationSample<const N: usize> {
    pub source_tick: u64,
    pub application_tick: u64,
    pub state: ArmState<N>,
    pub period: Duration,
    pub command: ControlType<N>,
    pub done: bool,
    pub transport_stop: bool,
    pub monotonic_ns: u64,
}

/// Preallocated, non-blocking audit path for actual callback applications.
pub struct ControlApplicationLog<const N: usize> {
    samples: Arc<ArrayQueue<ControlApplicationSample<N>>>,
    dropped: Arc<AtomicU64>,
}

impl<const N: usize> Clone for ControlApplicationLog<N> {
    fn clone(&self) -> Self {
        Self {
            samples: Arc::clone(&self.samples),
            dropped: Arc::clone(&self.dropped),
        }
    }
}

impl<const N: usize> ControlApplicationLog<N> {
    pub fn with_capacity(capacity: usize) -> Result<Self, RobotException> {
        if capacity == 0 {
            return Err(RobotException::CommandException(
                "control application log capacity must be positive".to_string(),
            ));
        }
        Ok(Self {
            samples: Arc::new(ArrayQueue::new(capacity)),
            dropped: Arc::new(AtomicU64::new(0)),
        })
    }

    pub fn try_pop(&self) -> Option<ControlApplicationSample<N>> {
        self.samples.pop()
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    pub fn dropped_samples(&self) -> u64 {
        self.dropped.load(Ordering::Acquire)
    }

    fn try_push(&self, sample: ControlApplicationSample<N>) {
        if self.samples.push(sample).is_err() {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }
}

struct ControlObservation<const N: usize> {
    source_tick: u64,
    state: ArmState<N>,
    period: Duration,
}

#[derive(Clone, Copy)]
struct TimedControl<const N: usize> {
    source_tick: u64,
    command: ControlType<N>,
    done: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
enum ControlStopReason {
    None = 0,
    DoneApplied = 1,
    ObservationQueueOverflow = 2,
    CommandQueueOverflow = 3,
    StaleCommand = 4,
    InvalidCommandTick = 5,
    CallbackStalled = 6,
}

impl ControlStopReason {
    fn from_u8(value: u8) -> Self {
        match value {
            0 => Self::None,
            1 => Self::DoneApplied,
            2 => Self::ObservationQueueOverflow,
            3 => Self::CommandQueueOverflow,
            4 => Self::StaleCommand,
            5 => Self::InvalidCommandTick,
            6 => Self::CallbackStalled,
            _ => Self::CommandQueueOverflow,
        }
    }
}

struct ControlMailbox<const N: usize> {
    observations: Arc<ArrayQueue<ControlObservation<N>>>,
    commands: Arc<ArrayQueue<TimedControl<N>>>,
    stop_reason: Arc<AtomicU8>,
    callback_finished: Arc<AtomicBool>,
}

impl<const N: usize> Clone for ControlMailbox<N> {
    fn clone(&self) -> Self {
        Self {
            observations: Arc::clone(&self.observations),
            commands: Arc::clone(&self.commands),
            stop_reason: Arc::clone(&self.stop_reason),
            callback_finished: Arc::clone(&self.callback_finished),
        }
    }
}

impl<const N: usize> ControlMailbox<N> {
    fn new() -> Self {
        Self {
            observations: Arc::new(ArrayQueue::new(CONTROL_QUEUE_CAPACITY)),
            commands: Arc::new(ArrayQueue::new(CONTROL_QUEUE_CAPACITY)),
            stop_reason: Arc::new(AtomicU8::new(ControlStopReason::None as u8)),
            callback_finished: Arc::new(AtomicBool::new(false)),
        }
    }

    fn request_stop(&self, reason: ControlStopReason) {
        let _ = self.stop_reason.compare_exchange(
            ControlStopReason::None as u8,
            reason as u8,
            Ordering::AcqRel,
            Ordering::Acquire,
        );
    }

    fn finish_callback(&self, reason: ControlStopReason) {
        self.request_stop(reason);
        self.callback_finished.store(true, Ordering::Release);
    }

    fn stop_reason(&self) -> ControlStopReason {
        ControlStopReason::from_u8(self.stop_reason.load(Ordering::Acquire))
    }

    fn callback_finished(&self) -> bool {
        self.callback_finished.load(Ordering::Acquire)
    }

    fn publish_command(&self, command: TimedControl<N>) -> bool {
        if self.commands.push(command).is_ok() {
            true
        } else {
            self.request_stop(ControlStopReason::CommandQueueOverflow);
            false
        }
    }

    fn result(&self) -> Result<(), RobotException> {
        match self.stop_reason() {
            ControlStopReason::DoneApplied => Ok(()),
            ControlStopReason::ObservationQueueOverflow => Err(RobotException::CommandException(
                "realtime control observation queue overflowed".to_string(),
            )),
            ControlStopReason::CommandQueueOverflow => Err(RobotException::CommandException(
                "realtime control command queue overflowed".to_string(),
            )),
            ControlStopReason::StaleCommand => Err(RobotException::CommandException(
                "realtime control command exceeded the maximum age".to_string(),
            )),
            ControlStopReason::InvalidCommandTick => Err(RobotException::CommandException(
                "realtime control received a command from a future tick".to_string(),
            )),
            ControlStopReason::CallbackStalled => Err(RobotException::CommandException(
                "realtime control callback stopped making progress".to_string(),
            )),
            ControlStopReason::None => Err(RobotException::CommandException(
                "realtime control callback ended without a terminal reason".to_string(),
            )),
        }
    }
}

struct ControlCallback<const N: usize> {
    mailbox: ControlMailbox<N>,
    callback_tick: u64,
    held_command: Option<TimedControl<N>>,
    application_log: Option<ControlApplicationLog<N>>,
    epoch: Instant,
}

impl<const N: usize> ControlCallback<N> {
    fn new(mailbox: ControlMailbox<N>, application_log: Option<ControlApplicationLog<N>>) -> Self {
        Self {
            mailbox,
            callback_tick: 0,
            held_command: None,
            application_log,
            epoch: Instant::now(),
        }
    }

    fn step(&mut self, state: ArmState<N>, period: Duration) -> (ControlType<N>, bool) {
        let state_hold = desired_torque_hold(&state);
        let stop_reason = self.mailbox.stop_reason();
        if stop_reason != ControlStopReason::None {
            self.mailbox
                .callback_finished
                .store(true, Ordering::Release);
            if stop_reason == ControlStopReason::DoneApplied {
                return (
                    self.held_command.map_or(state_hold, |value| value.command),
                    true,
                );
            }
            return self.transport_stop(&state, period, None);
        }

        let mut newly_applied = None;
        if let Some(command) = self.mailbox.commands.pop() {
            if command.source_tick >= self.callback_tick {
                self.mailbox
                    .finish_callback(ControlStopReason::InvalidCommandTick);
                return self.transport_stop(&state, period, Some(command.source_tick));
            }
            if self.callback_tick.saturating_sub(command.source_tick) > MAX_COMMAND_AGE_TICKS {
                self.mailbox
                    .finish_callback(ControlStopReason::StaleCommand);
                return self.transport_stop(&state, period, Some(command.source_tick));
            }
            self.held_command = Some(command);
            newly_applied = Some(command);
            if command.done {
                self.mailbox.finish_callback(ControlStopReason::DoneApplied);
                if self.mailbox.stop_reason() != ControlStopReason::DoneApplied {
                    return self.transport_stop(&state, period, Some(command.source_tick));
                }
                self.record_application_for_source(
                    command.source_tick,
                    &state,
                    period,
                    command.command,
                    true,
                    false,
                );
                return (command.command, true);
            }
        }

        if let Some(command) = self.held_command
            && self.callback_tick.saturating_sub(command.source_tick) > MAX_COMMAND_AGE_TICKS
        {
            self.mailbox
                .finish_callback(ControlStopReason::StaleCommand);
            return self.transport_stop(&state, period, Some(command.source_tick));
        }

        let output = self.held_command.map_or(state_hold, |value| value.command);
        let observation = ControlObservation {
            source_tick: self.callback_tick,
            state: state.clone(),
            period,
        };
        if self.mailbox.observations.push(observation).is_err() {
            self.mailbox
                .finish_callback(ControlStopReason::ObservationQueueOverflow);
            return self.transport_stop(&state, period, None);
        }
        let stop_reason = self.mailbox.stop_reason();
        if !matches!(
            stop_reason,
            ControlStopReason::None | ControlStopReason::DoneApplied
        ) {
            self.mailbox
                .callback_finished
                .store(true, Ordering::Release);
            return self.transport_stop(&state, period, None);
        }
        if let Some(command) = newly_applied {
            self.record_application_for_source(
                command.source_tick,
                &state,
                period,
                output,
                false,
                false,
            );
        }
        self.callback_tick = self.callback_tick.saturating_add(1);
        (output, false)
    }

    fn transport_stop(
        &self,
        state: &ArmState<N>,
        period: Duration,
        source_tick: Option<u64>,
    ) -> (ControlType<N>, bool) {
        let command = desired_torque_hold(state);
        let source_tick = source_tick
            .or_else(|| self.held_command.map(|held| held.source_tick))
            .unwrap_or_else(|| self.callback_tick.saturating_sub(1));
        self.record_application_for_source(source_tick, state, period, command, true, true);
        (command, true)
    }

    fn record_application_for_source(
        &self,
        source_tick: u64,
        state: &ArmState<N>,
        period: Duration,
        command: ControlType<N>,
        done: bool,
        transport_stop: bool,
    ) {
        if let Some(log) = &self.application_log {
            log.try_push(ControlApplicationSample {
                source_tick,
                application_tick: self.callback_tick,
                state: state.clone(),
                period,
                command,
                done,
                transport_stop,
                monotonic_ns: self.epoch.elapsed().as_nanos().min(u64::MAX as u128) as u64,
            });
        }
    }
}

fn desired_torque_hold<const N: usize>(state: &ArmState<N>) -> ControlType<N> {
    state
        .desired
        .torque
        .filter(|torque| torque.iter().all(|value| value.is_finite()))
        .map_or(ControlType::Zero, ControlType::Torque)
}

pub struct ArmControlRhythm<A, const N: usize> {
    pub arm: A,
    /// Optional rate-limit period. If Some, the drive loop will sleep to maintain
    /// the target frequency. Useful for simulation backends that run faster than
    /// real-time.  For real robots, set to None (timing is driven by hardware).
    pub period: Option<Duration>,
    application_log: Option<ControlApplicationLog<N>>,
}

impl<A, const N: usize> ArmControlRhythm<A, N> {
    pub fn new(arm: A) -> Self {
        Self { arm, period: None, application_log: None }
    }

    pub fn with_period(mut self, period: Duration) -> Self {
        self.period = Some(period);
        self
    }

    pub fn with_application_log(mut self, log: ControlApplicationLog<N>) -> Self {
        self.application_log = Some(log);
        self
    }
}

impl<A, const N: usize> Rhythm for ArmControlRhythm<A, N>
where
    A: ArmRealtimeControl<N> + Send,
{
    type Input = ();
    type Yield = (ArmState<N>, Duration);
    type Feed = (ControlType<N>, bool);
    type Output = Result<(), RobotException>;
    type Error = RoplatError;

    fn drive<Nodes, F, Fut>(
        &mut self,
        mut nodes: Nodes,
        mut op_domain: F,
        _input: Self::Input,
    ) -> impl Future<Output = (Self::Output, Nodes)> + Send
    where
        Nodes: Send,
        F: FnMut(Nodes, Self::Yield) -> Fut + Send,
        Fut: Future<Output = (Self::Feed, Nodes)> + Send,
    {
        async move {
            let mailbox = ControlMailbox::new();
            let mut callback = ControlCallback::new(mailbox.clone(), self.application_log.clone());
            if let Err(error) = self
                .arm
                .control_with_closure(move |state, period| callback.step(state, period))
            {
                return (Err(error), nodes);
            }

            let period = self.period;
            let mut done_published = false;
            let mut last_callback_progress = Instant::now();
            loop {
                if mailbox.callback_finished() {
                    break;
                }
                if done_published {
                    if last_callback_progress.elapsed() >= CONTROL_CALLBACK_STALL_TIMEOUT {
                        mailbox.request_stop(ControlStopReason::CallbackStalled);
                        break;
                    }
                    tokio::time::sleep(CONTROL_MAILBOX_POLL_INTERVAL).await;
                    continue;
                }
                if let Some(observation) = mailbox.observations.pop() {
                    last_callback_progress = Instant::now();
                    let tick_start = Instant::now();
                    let source_tick = observation.source_tick;
                    let ((command, done), returned_nodes) =
                        op_domain(nodes, (observation.state, observation.period)).await;
                    nodes = returned_nodes;
                    done_published = done;
                    if !mailbox.publish_command(TimedControl { source_tick, command, done }) {
                        continue;
                    }

                    // Rate limiting is for simulation backends only. Real hardware
                    // supplies the callback cadence and leaves `period` as `None`.
                    if !done && let Some(target_period) = period {
                        let elapsed = tick_start.elapsed();
                        if elapsed < target_period {
                            tokio::time::sleep(target_period - elapsed).await;
                        }
                    }
                } else {
                    if last_callback_progress.elapsed() >= CONTROL_CALLBACK_STALL_TIMEOUT {
                        mailbox.request_stop(ControlStopReason::CallbackStalled);
                        break;
                    }
                    tokio::time::sleep(CONTROL_MAILBOX_POLL_INTERVAL).await;
                }
            }

            (mailbox.result(), nodes)
        }
    }
}

// ── ArmMotionRhythm ──────────────────────────────────────────────────────
// Wraps ArmRealtimeControl::move_with_closure as a Rhythm source.
// Each rhythm tick corresponds to one realtime motion cycle.
// Yield = (ArmState<N>, Duration),  Feed = (MotionType<N>, bool)

pub struct ArmMotionRhythm<A, const N: usize> {
    pub arm: A,
    pub period: Option<Duration>,
}

impl<A, const N: usize> ArmMotionRhythm<A, N> {
    pub fn new(arm: A) -> Self {
        Self { arm, period: None }
    }

    pub fn with_period(mut self, period: Duration) -> Self {
        self.period = Some(period);
        self
    }
}

impl<A, const N: usize> Rhythm for ArmMotionRhythm<A, N>
where
    A: ArmRealtimeControl<N> + Send,
{
    type Input = ();
    type Yield = (ArmState<N>, Duration);
    type Feed = (MotionType<N>, bool);
    type Output = Result<(), RobotException>;
    type Error = RoplatError;

    fn drive<Nodes, F, Fut>(
        &mut self,
        mut nodes: Nodes,
        mut op_domain: F,
        _input: Self::Input,
    ) -> impl Future<Output = (Self::Output, Nodes)> + Send
    where
        Nodes: Send,
        F: FnMut(Nodes, Self::Yield) -> Fut + Send,
        Fut: Future<Output = (Self::Feed, Nodes)> + Send,
    {
        async move {
            let (state_tx, mut state_rx) =
                tokio::sync::mpsc::unbounded_channel::<(ArmState<N>, Duration)>();
            let (cmd_tx, cmd_rx) = std::sync::mpsc::sync_channel::<(MotionType<N>, bool)>(0);

            if let Err(e) = self.arm.move_with_closure(move |state, dt| {
                if state_tx.send((state, dt)).is_err() {
                    return (MotionType::Stop, true);
                }
                match cmd_rx.recv() {
                    Ok(cmd) => cmd,
                    Err(_) => (MotionType::Stop, true),
                }
            }) {
                return (Err(e), nodes);
            }

            let period = self.period;
            while let Some((state, dt)) = state_rx.recv().await {
                let tick_start = Instant::now();

                let ((cmd, done), returned_nodes) = op_domain(nodes, (state, dt)).await;
                nodes = returned_nodes;

                if cmd_tx.send((cmd, done)).is_err() || done {
                    break;
                }

                if let Some(p) = period {
                    let elapsed = tick_start.elapsed();
                    if elapsed < p {
                        tokio::time::sleep(p - elapsed).await;
                    }
                }
            }

            (Ok(()), nodes)
        }
    }
}

// ── CartesianImpedanceRhythm ─────────────────────────────────────────────
// Wraps ArmImpedance::cartesian_impedance_async as a Rhythm source.
// The robot runs its own impedance control loop internally.
// This rhythm periodically polls the arm state and lets nodes update the target.
// Yield = ArmState<N>,  Feed = Option<Pose>  (None = keep current target)

pub struct CartesianImpedanceRhythm<A, const N: usize> {
    pub arm: A,
    pub stiffness: (f64, f64),
    pub damping: (f64, f64),
    pub period: Duration,
}

impl<A, const N: usize> CartesianImpedanceRhythm<A, N> {
    pub fn new(arm: A, stiffness: (f64, f64), damping: (f64, f64), period: Duration) -> Self {
        Self { arm, stiffness, damping, period }
    }
}

impl<A, const N: usize> Rhythm for CartesianImpedanceRhythm<A, N>
where
    A: ArmImpedance<N> + crate::behavior::Arm<N> + Send,
{
    type Input = ();
    type Yield = ArmState<N>;
    type Feed = Option<Pose>;
    type Output = Result<(), RobotException>;
    type Error = RoplatError;

    fn drive<Nodes, F, Fut>(
        &mut self,
        mut nodes: Nodes,
        mut op_domain: F,
        _input: Self::Input,
    ) -> impl Future<Output = (Self::Output, Nodes)> + Send
    where
        Nodes: Send,
        F: FnMut(Nodes, Self::Yield) -> Fut + Send,
        Fut: Future<Output = (Self::Feed, Nodes)> + Send,
    {
        async move {
            let handle = match self
                .arm
                .cartesian_impedance_async(self.stiffness, self.damping)
            {
                Ok(h) => h,
                Err(e) => return (Err(e), nodes),
            };

            let mut sequence = 0u32;
            let start_time = Instant::now();

            loop {
                let next_target = start_time + self.period * sequence;
                tokio::time::sleep_until(next_target.into()).await;
                sequence += 1;

                let state = match self.arm.state() {
                    Ok(s) => s,
                    Err(e) => return (Err(e), nodes),
                };
                let (feed, returned_nodes) = op_domain(nodes, state).await;
                nodes = returned_nodes;

                if let Some(target) = feed {
                    handle.set_target(Some(target));
                }

                if handle.is_finished.load(Ordering::SeqCst) {
                    break;
                }
            }

            (Ok(()), nodes)
        }
    }
}

// ── JointImpedanceRhythm ────────────────────────────────────────────────
// Wraps ArmImpedance::joint_impedance_async as a Rhythm source.
// Yield = ArmState<N>,  Feed = Option<[f64; N]>  (None = keep current target)

pub struct JointImpedanceRhythm<A, const N: usize> {
    pub arm: A,
    pub stiffness: [f64; N],
    pub damping: [f64; N],
    pub period: Duration,
}

impl<A, const N: usize> JointImpedanceRhythm<A, N> {
    pub fn new(arm: A, stiffness: [f64; N], damping: [f64; N], period: Duration) -> Self {
        Self { arm, stiffness, damping, period }
    }
}

impl<A, const N: usize> Rhythm for JointImpedanceRhythm<A, N>
where
    A: ArmImpedance<N> + crate::behavior::Arm<N> + Send,
{
    type Input = ();
    type Yield = ArmState<N>;
    type Feed = Option<[f64; N]>;
    type Output = Result<(), RobotException>;
    type Error = RoplatError;

    fn drive<Nodes, F, Fut>(
        &mut self,
        mut nodes: Nodes,
        mut op_domain: F,
        _input: Self::Input,
    ) -> impl Future<Output = (Self::Output, Nodes)> + Send
    where
        Nodes: Send,
        F: FnMut(Nodes, Self::Yield) -> Fut + Send,
        Fut: Future<Output = (Self::Feed, Nodes)> + Send,
    {
        async move {
            let handle = match self
                .arm
                .joint_impedance_async(&self.stiffness, &self.damping)
            {
                Ok(h) => h,
                Err(e) => return (Err(e), nodes),
            };

            let mut sequence = 0u32;
            let start_time = Instant::now();

            loop {
                let next_target = start_time + self.period * sequence;
                tokio::time::sleep_until(next_target.into()).await;
                sequence += 1;

                let state = match self.arm.state() {
                    Ok(s) => s,
                    Err(e) => return (Err(e), nodes),
                };
                let (feed, returned_nodes) = op_domain(nodes, state).await;
                nodes = returned_nodes;

                if let Some(target) = feed {
                    handle.set_target(Some(target));
                }

                if handle.is_finished.load(Ordering::SeqCst) {
                    break;
                }
            }

            (Ok(()), nodes)
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use std::sync::{Arc, mpsc};
    use std::thread;

    use super::*;
    use crate::{Arm, ArmStateSample, Coord, LoadState, RobotResult};

    const TEST_DOF: usize = 2;

    fn state_with_desired_torque(torque: [f64; TEST_DOF]) -> ArmState<TEST_DOF> {
        ArmState {
            desired: ArmStateSample { torque: Some(torque), ..Default::default() },
            ..Default::default()
        }
    }

    struct MockArm {
        callback_count: Arc<AtomicUsize>,
        done_seen: Arc<AtomicBool>,
        produce_callbacks: bool,
    }

    impl Arm<TEST_DOF> for MockArm {
        fn state(&mut self) -> RobotResult<ArmState<TEST_DOF>> {
            Ok(ArmState::default())
        }

        fn set_load(&mut self, _load: LoadState) -> RobotResult<()> {
            Ok(())
        }

        fn set_coord(&mut self, _coord: Coord) -> RobotResult<()> {
            Ok(())
        }

        fn with_coord(&mut self, _coord: Coord) -> &mut Self {
            self
        }

        fn set_scale(&mut self, _scale: f64) -> RobotResult<()> {
            Ok(())
        }

        fn with_scale(&mut self, _scale: f64) -> &mut Self {
            self
        }

        fn with_velocity(&mut self, _joint_vel: &[f64; TEST_DOF]) -> &mut Self {
            self
        }

        fn with_acceleration(&mut self, _joint_acc: &[f64; TEST_DOF]) -> &mut Self {
            self
        }

        fn with_jerk(&mut self, _joint_jerk: &[f64; TEST_DOF]) -> &mut Self {
            self
        }

        fn with_cartesian_velocity(&mut self, _cartesian_vel: f64) -> &mut Self {
            self
        }

        fn with_cartesian_acceleration(&mut self, _cartesian_acc: f64) -> &mut Self {
            self
        }

        fn with_cartesian_jerk(&mut self, _cartesian_jerk: f64) -> &mut Self {
            self
        }

        fn with_rotation_velocity(&mut self, _rotation_vel: f64) -> &mut Self {
            self
        }

        fn with_rotation_acceleration(&mut self, _rotation_acc: f64) -> &mut Self {
            self
        }

        fn with_rotation_jerk(&mut self, _rotation_jerk: f64) -> &mut Self {
            self
        }
    }

    impl ArmRealtimeControl<TEST_DOF> for MockArm {
        fn move_with_closure<F>(&mut self, _closure: F) -> RobotResult<()>
        where
            F: FnMut(ArmState<TEST_DOF>, Duration) -> (MotionType<TEST_DOF>, bool) + Send + 'static,
        {
            unreachable!("control rhythm test must not enter motion mode")
        }

        fn control_with_closure<F>(&mut self, mut closure: F) -> RobotResult<()>
        where
            F: FnMut(ArmState<TEST_DOF>, Duration) -> (ControlType<TEST_DOF>, bool)
                + Send
                + 'static,
        {
            if !self.produce_callbacks {
                return Ok(());
            }
            let callback_count = Arc::clone(&self.callback_count);
            let done_seen = Arc::clone(&self.done_seen);
            thread::spawn(move || {
                for _ in 0..64 {
                    let state = state_with_desired_torque([0.25; TEST_DOF]);
                    let (_, done) = closure(state, Duration::from_millis(1));
                    callback_count.fetch_add(1, Ordering::Release);
                    if done {
                        done_seen.store(true, Ordering::Release);
                        return;
                    }
                    thread::sleep(Duration::from_millis(1));
                }
            });
            Ok(())
        }
    }

    #[tokio::test(flavor = "current_thread")]
    async fn done_feed_is_applied_by_the_callback_before_drive_returns() {
        let callback_count = Arc::new(AtomicUsize::new(0));
        let done_seen = Arc::new(AtomicBool::new(false));
        let arm = MockArm {
            callback_count: Arc::clone(&callback_count),
            done_seen: Arc::clone(&done_seen),
            produce_callbacks: true,
        };
        let mut rhythm = ArmControlRhythm::new(arm);
        let drive = rhythm.drive(
            0_usize,
            |count, _sample| async move {
                let next = count + 1;
                (
                    (ControlType::Torque([next as f64; TEST_DOF]), next == 4),
                    next,
                )
            },
            (),
        );
        let (result, processed) = tokio::time::timeout(Duration::from_secs(1), drive)
            .await
            .expect("mock control drive timed out");

        assert!(result.is_ok());
        assert_eq!(processed, 4);
        for _ in 0..100 {
            if done_seen.load(Ordering::Acquire) {
                break;
            }
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        assert!(done_seen.load(Ordering::Acquire));
        assert!(callback_count.load(Ordering::Acquire) >= 5);
    }

    #[tokio::test(flavor = "current_thread")]
    async fn drive_returns_an_error_when_the_callback_never_starts() {
        let arm = MockArm {
            callback_count: Arc::new(AtomicUsize::new(0)),
            done_seen: Arc::new(AtomicBool::new(false)),
            produce_callbacks: false,
        };
        let mut rhythm = ArmControlRhythm::new(arm);
        let drive = rhythm.drive(
            0_usize,
            |count, _sample| async move { ((ControlType::Zero, false), count + 1) },
            (),
        );
        let (result, processed) = tokio::time::timeout(Duration::from_secs(1), drive)
            .await
            .expect("callback stall watchdog did not return");

        assert!(result.is_err());
        assert_eq!(processed, 0);
    }

    #[test]
    fn callback_does_not_wait_when_the_observation_queue_is_full() {
        let mailbox = ControlMailbox::new();
        let mut callback = ControlCallback::new(mailbox.clone(), None);
        let (sender, receiver) = mpsc::channel();
        thread::spawn(move || {
            let mut output = (ControlType::Zero, false);
            for _ in 0..=CONTROL_QUEUE_CAPACITY {
                output = callback.step(
                    state_with_desired_torque([0.25; TEST_DOF]),
                    Duration::from_millis(1),
                );
            }
            let _ = sender.send((output, mailbox.stop_reason()));
        });

        let ((command, done), reason) = receiver
            .recv_timeout(Duration::from_millis(100))
            .expect("a full realtime queue must stop instead of blocking");
        assert!(done);
        assert!(matches!(command, ControlType::Torque([0.25, 0.25])));
        assert_eq!(reason, ControlStopReason::ObservationQueueOverflow);
    }

    #[test]
    fn initial_cycles_hold_the_robot_desired_torque() {
        let mailbox = ControlMailbox::new();
        let mut callback = ControlCallback::new(mailbox, None);
        let (command, done) = callback.step(
            state_with_desired_torque([0.4, -0.2]),
            Duration::from_millis(1),
        );

        assert!(!done);
        assert!(matches!(command, ControlType::Torque([0.4, -0.2])));
    }

    #[test]
    fn non_finite_desired_torque_falls_back_to_zero() {
        let state = state_with_desired_torque([f64::NAN, 0.0]);

        assert!(matches!(desired_torque_hold(&state), ControlType::Zero));
    }

    #[test]
    fn done_is_acknowledged_only_when_the_callback_consumes_it() {
        let mailbox = ControlMailbox::new();
        let mut callback = ControlCallback::new(mailbox.clone(), None);
        let (_, initial_done) = callback.step(
            state_with_desired_torque([0.0; TEST_DOF]),
            Duration::from_millis(1),
        );
        assert!(!initial_done);
        let observation = mailbox
            .observations
            .pop()
            .expect("the initial callback must publish an observation");
        assert!(mailbox.publish_command(TimedControl {
            source_tick: observation.source_tick,
            command: ControlType::Torque([0.5; TEST_DOF]),
            done: true,
        }));
        assert!(!mailbox.callback_finished());

        let (command, done) = callback.step(
            state_with_desired_torque([0.0; TEST_DOF]),
            Duration::from_millis(1),
        );
        assert!(done);
        assert!(matches!(command, ControlType::Torque([0.5, 0.5])));
        assert!(mailbox.callback_finished());
        assert_eq!(mailbox.stop_reason(), ControlStopReason::DoneApplied);
    }

    #[test]
    fn transport_stop_preempts_a_queued_terminal_command() {
        let mailbox = ControlMailbox::new();
        let mut callback = ControlCallback::new(mailbox.clone(), None);
        let _ = callback.step(
            state_with_desired_torque([0.1; TEST_DOF]),
            Duration::from_millis(1),
        );
        let observation = mailbox
            .observations
            .pop()
            .expect("the initial callback must publish an observation");
        assert!(mailbox.publish_command(TimedControl {
            source_tick: observation.source_tick,
            command: ControlType::Torque([0.8; TEST_DOF]),
            done: true,
        }));
        mailbox.request_stop(ControlStopReason::CallbackStalled);

        let (command, done) = callback.step(
            state_with_desired_torque([0.3, -0.2]),
            Duration::from_millis(1),
        );

        assert!(done);
        assert!(matches!(command, ControlType::Torque([0.3, -0.2])));
        assert_eq!(mailbox.stop_reason(), ControlStopReason::CallbackStalled);
    }

    #[test]
    fn application_log_records_the_callback_tick_that_consumed_a_command() {
        let mailbox = ControlMailbox::new();
        let log = ControlApplicationLog::with_capacity(4).expect("valid audit capacity");
        let mut callback = ControlCallback::new(mailbox.clone(), Some(log.clone()));
        let _ = callback.step(
            state_with_desired_torque([0.0; TEST_DOF]),
            Duration::from_millis(1),
        );
        let observation = mailbox
            .observations
            .pop()
            .expect("the initial callback must publish an observation");
        assert!(mailbox.publish_command(TimedControl {
            source_tick: observation.source_tick,
            command: ControlType::Torque([0.5; TEST_DOF]),
            done: true,
        }));

        let _ = callback.step(
            state_with_desired_torque([0.0; TEST_DOF]),
            Duration::from_millis(1),
        );
        let application = log.try_pop().expect("terminal application must be audited");
        assert_eq!(application.source_tick, 0);
        assert_eq!(application.application_tick, 1);
        assert!(application.done);
        assert!(!application.transport_stop);
        assert_eq!(log.dropped_samples(), 0);
    }

    #[test]
    fn application_log_does_not_repeat_an_ordinary_held_command() {
        let mailbox = ControlMailbox::new();
        let log = ControlApplicationLog::with_capacity(4).expect("valid audit capacity");
        let mut callback = ControlCallback::new(mailbox.clone(), Some(log.clone()));
        let _ = callback.step(
            state_with_desired_torque([0.0; TEST_DOF]),
            Duration::from_millis(1),
        );
        let observation = mailbox
            .observations
            .pop()
            .expect("the initial callback must publish an observation");
        assert!(mailbox.publish_command(TimedControl {
            source_tick: observation.source_tick,
            command: ControlType::Torque([0.5; TEST_DOF]),
            done: false,
        }));

        let _ = callback.step(
            state_with_desired_torque([0.0; TEST_DOF]),
            Duration::from_millis(1),
        );
        let _ = mailbox.observations.pop();
        let _ = callback.step(
            state_with_desired_torque([0.5; TEST_DOF]),
            Duration::from_millis(1),
        );

        let application = log.try_pop().expect("new command must be audited once");
        assert_eq!(application.source_tick, 0);
        assert_eq!(application.application_tick, 1);
        assert!(log.try_pop().is_none());
    }

    #[test]
    fn stale_command_stops_without_waiting_for_another_feed() {
        let mailbox = ControlMailbox::new();
        let mut callback = ControlCallback::new(mailbox.clone(), None);
        let _ = callback.step(
            state_with_desired_torque([0.0; TEST_DOF]),
            Duration::from_millis(1),
        );
        let observation = mailbox
            .observations
            .pop()
            .expect("the initial callback must publish an observation");
        assert!(mailbox.publish_command(TimedControl {
            source_tick: observation.source_tick,
            command: ControlType::Torque([0.75; TEST_DOF]),
            done: false,
        }));

        for _ in 1..=MAX_COMMAND_AGE_TICKS {
            let (_, done) = callback.step(
                state_with_desired_torque([0.0; TEST_DOF]),
                Duration::from_millis(1),
            );
            assert!(!done);
        }
        let (command, done) = callback.step(
            state_with_desired_torque([0.0; TEST_DOF]),
            Duration::from_millis(1),
        );
        assert!(done);
        assert!(matches!(command, ControlType::Torque([0.0, 0.0])));
        assert_eq!(mailbox.stop_reason(), ControlStopReason::StaleCommand);
    }

    #[test]
    fn stale_terminal_command_is_rejected_before_it_can_replace_the_hold() {
        let mailbox = ControlMailbox::new();
        let mut callback = ControlCallback::new(mailbox.clone(), None);
        for _ in 0..=MAX_COMMAND_AGE_TICKS {
            let (_, done) = callback.step(
                state_with_desired_torque([0.2; TEST_DOF]),
                Duration::from_millis(1),
            );
            assert!(!done);
            let _ = mailbox.observations.pop();
        }
        assert!(mailbox.publish_command(TimedControl {
            source_tick: 0,
            command: ControlType::Torque([0.9; TEST_DOF]),
            done: true,
        }));

        let (command, done) = callback.step(
            state_with_desired_torque([0.2; TEST_DOF]),
            Duration::from_millis(1),
        );
        assert!(done);
        assert!(matches!(command, ControlType::Torque([0.2, 0.2])));
        assert_eq!(mailbox.stop_reason(), ControlStopReason::StaleCommand);
    }

    #[test]
    fn future_command_stops_with_the_current_state_hold() {
        let mailbox = ControlMailbox::new();
        let mut callback = ControlCallback::new(mailbox.clone(), None);
        let _ = callback.step(
            state_with_desired_torque([0.1; TEST_DOF]),
            Duration::from_millis(1),
        );
        let observation = mailbox
            .observations
            .pop()
            .expect("the initial callback must publish an observation");
        assert!(mailbox.publish_command(TimedControl {
            source_tick: observation.source_tick,
            command: ControlType::Torque([0.8; TEST_DOF]),
            done: false,
        }));
        let _ = callback.step(
            state_with_desired_torque([0.1; TEST_DOF]),
            Duration::from_millis(1),
        );
        let _ = mailbox.observations.pop();
        assert!(mailbox.publish_command(TimedControl {
            source_tick: callback.callback_tick,
            command: ControlType::Torque([0.9; TEST_DOF]),
            done: false,
        }));

        let (command, done) = callback.step(
            state_with_desired_torque([0.35, -0.15]),
            Duration::from_millis(1),
        );

        assert!(done);
        assert!(matches!(command, ControlType::Torque([0.35, -0.15])));
        assert_eq!(mailbox.stop_reason(), ControlStopReason::InvalidCommandTick);
    }

    #[test]
    fn existing_transport_stop_uses_state_hold_and_audits_the_terminal_callback() {
        let mailbox = ControlMailbox::new();
        let log = ControlApplicationLog::with_capacity(4).expect("valid audit capacity");
        let mut callback = ControlCallback::new(mailbox.clone(), Some(log.clone()));
        let _ = callback.step(
            state_with_desired_torque([0.1; TEST_DOF]),
            Duration::from_millis(1),
        );
        let observation = mailbox
            .observations
            .pop()
            .expect("the initial callback must publish an observation");
        assert!(mailbox.publish_command(TimedControl {
            source_tick: observation.source_tick,
            command: ControlType::Torque([0.8; TEST_DOF]),
            done: false,
        }));
        let _ = callback.step(
            state_with_desired_torque([0.1; TEST_DOF]),
            Duration::from_millis(1),
        );
        let _ = mailbox.observations.pop();
        mailbox.request_stop(ControlStopReason::CallbackStalled);

        let (command, done) = callback.step(
            state_with_desired_torque([0.3, -0.2]),
            Duration::from_millis(1),
        );

        assert!(done);
        assert!(matches!(command, ControlType::Torque([0.3, -0.2])));
        let ordinary = log.try_pop().expect("ordinary command application");
        assert!(!ordinary.transport_stop);
        let terminal = log.try_pop().expect("terminal hold application");
        assert!(terminal.transport_stop);
        assert!(terminal.done);
        assert!(matches!(terminal.command, ControlType::Torque([0.3, -0.2])));
    }

    #[test]
    fn observation_overflow_replaces_a_held_command_with_state_hold() {
        let mailbox = ControlMailbox::new();
        let mut callback = ControlCallback::new(mailbox.clone(), None);
        let _ = callback.step(
            state_with_desired_torque([0.1; TEST_DOF]),
            Duration::from_millis(1),
        );
        let observation = mailbox
            .observations
            .pop()
            .expect("the initial callback must publish an observation");
        assert!(mailbox.publish_command(TimedControl {
            source_tick: observation.source_tick,
            command: ControlType::Torque([0.8; TEST_DOF]),
            done: false,
        }));
        let _ = callback.step(
            state_with_desired_torque([0.1; TEST_DOF]),
            Duration::from_millis(1),
        );
        for _ in 1..CONTROL_QUEUE_CAPACITY {
            assert!(mailbox.publish_command(TimedControl {
                source_tick: callback.callback_tick.saturating_sub(1),
                command: ControlType::Torque([0.8; TEST_DOF]),
                done: false,
            }));
            let (_, done) = callback.step(
                state_with_desired_torque([0.2; TEST_DOF]),
                Duration::from_millis(1),
            );
            assert!(!done);
        }

        let (command, done) = callback.step(
            state_with_desired_torque([0.35, -0.15]),
            Duration::from_millis(1),
        );

        assert!(done);
        assert!(matches!(command, ControlType::Torque([0.35, -0.15])));
        assert_eq!(
            mailbox.stop_reason(),
            ControlStopReason::ObservationQueueOverflow
        );
    }

    #[test]
    fn command_queue_overflow_requests_a_callback_stop() {
        let mailbox = ControlMailbox::new();
        let mut callback = ControlCallback::new(mailbox.clone(), None);
        let _ = callback.step(
            state_with_desired_torque([0.1; TEST_DOF]),
            Duration::from_millis(1),
        );
        let observation = mailbox
            .observations
            .pop()
            .expect("the initial callback must publish an observation");
        assert!(mailbox.publish_command(TimedControl {
            source_tick: observation.source_tick,
            command: ControlType::Torque([0.8; TEST_DOF]),
            done: false,
        }));
        let _ = callback.step(
            state_with_desired_torque([0.1; TEST_DOF]),
            Duration::from_millis(1),
        );
        let _ = mailbox.observations.pop();
        for source_tick in 1..=CONTROL_QUEUE_CAPACITY as u64 {
            assert!(mailbox.publish_command(TimedControl {
                source_tick,
                command: ControlType::Torque([0.1; TEST_DOF]),
                done: false,
            }));
        }
        assert!(!mailbox.publish_command(TimedControl {
            source_tick: CONTROL_QUEUE_CAPACITY as u64 + 1,
            command: ControlType::Torque([0.2; TEST_DOF]),
            done: false,
        }));

        let (command, done) = callback.step(
            state_with_desired_torque([0.3, -0.2]),
            Duration::from_millis(1),
        );
        assert!(done);
        assert!(matches!(command, ControlType::Torque([0.3, -0.2])));
        assert!(mailbox.callback_finished());
        assert_eq!(
            mailbox.stop_reason(),
            ControlStopReason::CommandQueueOverflow
        );
    }
}
