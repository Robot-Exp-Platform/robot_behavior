use std::future::Future;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use roplat::RoplatError;
use roplat::rhythm::Rhythm;

use crate::{
    ArmImpedance, ArmRealtimeControl, ArmState, ControlType, MotionType, Pose, RobotException,
};

// ── ArmControlRhythm ──────────────────────────────────────────────────────
// Wraps ArmRealtimeControl::control_with_closure as a Rhythm source.
// Each rhythm tick corresponds to one realtime control cycle.
// Yield = (ArmState<N>, Duration),  Feed = (ControlType<N>, bool)

pub struct ArmControlRhythm<A, const N: usize> {
    pub arm: A,
    /// Optional rate-limit period. If Some, the drive loop will sleep to maintain
    /// the target frequency. Useful for simulation backends that run faster than
    /// real-time.  For real robots, set to None (timing is driven by hardware).
    pub period: Option<Duration>,
}

impl<A, const N: usize> ArmControlRhythm<A, N> {
    pub fn new(arm: A) -> Self {
        Self { arm, period: None }
    }

    pub fn with_period(mut self, period: Duration) -> Self {
        self.period = Some(period);
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
            // Channel: closure �?drive loop (arm state)
            let (state_tx, mut state_rx) =
                tokio::sync::mpsc::unbounded_channel::<(ArmState<N>, Duration)>();
            // Channel: drive loop �?closure (control command)
            let (cmd_tx, cmd_rx) = std::sync::mpsc::sync_channel::<(ControlType<N>, bool)>(0);

            if let Err(e) = self.arm.control_with_closure(move |state, dt| {
                if state_tx.send((state, dt)).is_err() {
                    return (ControlType::Zero, true);
                }
                match cmd_rx.recv() {
                    Ok(cmd) => cmd,
                    Err(_) => (ControlType::Zero, true),
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

                // Rate limiting for simulation backends
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
