//! Joint position, velocity and torque channels.

use std::time::Duration;

use roplat_behavior::{JointState, Joints};
use roplat_msgs::JointLimits;

use crate::{
    Axis, CommandCheck, Quantity, SafetyViolation,
    bound::{self, within},
};

/// The limits a driver declares through [`Joints`]. Bounds left at their
/// `f64::MAX` default stay effectively unbounded.
pub fn joint_limits<R: Joints<N>, const N: usize>() -> JointLimits<N> {
    JointLimits {
        q_min: R::JOINT_MIN,
        q_max: R::JOINT_MAX,
        dq_max: R::JOINT_VEL_BOUND,
        ddq_max: R::JOINT_ACC_BOUND,
        tau_max: R::TORQUE_BOUND,
    }
}

/// Joint position commands: inside the position range, and moving no faster
/// than `dq_max`, nor changing that speed faster than `ddq_max`, from one
/// command to the next.
///
/// The first command of a session is measured against the observed position
/// and velocity. Optionally every command also stays within a tracking error
/// of the observed position, so commands cannot run away from a blocked arm.
#[derive(Clone, Debug)]
pub struct JointPositionCheck<const N: usize> {
    limits: JointLimits<N>,
    max_tracking_error: [f64; N],
    /// The previous command and the velocity it implied.
    previous: Option<([f64; N], [f64; N])>,
}

impl<const N: usize> JointPositionCheck<N> {
    pub fn new(limits: JointLimits<N>) -> Self {
        Self {
            limits,
            max_tracking_error: [f64::INFINITY; N],
            previous: None,
        }
    }

    /// Bounds `|command - observed position|` in every cycle.
    pub fn with_tracking_error(mut self, max: [f64; N]) -> Self {
        self.max_tracking_error = max;
        self
    }

    fn evaluate(
        &self,
        state: &JointState<N>,
        dt: f64,
        q: &[f64; N],
    ) -> Result<([f64; N], [f64; N]), SafetyViolation> {
        let limits = &self.limits;
        bound::finite(Quantity::Position, q)?;
        bound::range(Quantity::Position, q, &limits.q_min, &limits.q_max)?;
        if bound::bounded(&self.max_tracking_error) {
            let measured = bound::measured(Quantity::Position, state.meas.q(), true)?;
            let error = std::array::from_fn(|i| q[i] - measured[i]);
            bound::magnitude(Quantity::TrackingError, &error, &self.max_tracking_error)?;
        }
        let accelerating = bound::bounded(&limits.ddq_max);
        let (q0, v0) = match self.previous {
            Some(previous) => previous,
            None => (
                bound::measured(
                    Quantity::Position,
                    state.meas.q(),
                    accelerating || bound::bounded(&limits.dq_max),
                )?,
                bound::measured(Quantity::Velocity, state.meas.dq(), accelerating)?,
            ),
        };
        bound::rate(Quantity::Velocity, &q0, q, &limits.dq_max, dt)?;
        let v = if dt > 0. {
            std::array::from_fn(|i| (q[i] - q0[i]) / dt)
        } else {
            v0
        };
        bound::rate(Quantity::Acceleration, &v0, &v, &limits.ddq_max, dt)?;
        Ok((*q, v))
    }
}

impl<const N: usize> CommandCheck for JointPositionCheck<N> {
    type Obs = JointState<N>;
    type Command = [f64; N];

    fn check(
        &mut self,
        state: &JointState<N>,
        dt: Duration,
        q: &[f64; N],
    ) -> Result<(), SafetyViolation> {
        let result = self.evaluate(state, dt.as_secs_f64(), q);
        self.previous = result.as_ref().ok().copied();
        result.map(drop)
    }

    fn reset(&mut self) {
        self.previous = None;
    }
}

/// Joint velocity commands: within `dq_max`, changing no faster than
/// `ddq_max`, and never carrying a joint out of its position range.
///
/// From the observed position, one cycle at the commanded velocity followed
/// by braking at `ddq_max` must end inside the range; a command back toward
/// the range always passes this part. The first command of a session is
/// measured against the observed velocity.
#[derive(Clone, Debug)]
pub struct JointVelocityCheck<const N: usize> {
    limits: JointLimits<N>,
    previous: Option<[f64; N]>,
}

impl<const N: usize> JointVelocityCheck<N> {
    pub fn new(limits: JointLimits<N>) -> Self {
        Self { limits, previous: None }
    }

    fn evaluate(
        &self,
        state: &JointState<N>,
        dt: f64,
        dq: &[f64; N],
    ) -> Result<[f64; N], SafetyViolation> {
        let limits = &self.limits;
        bound::finite(Quantity::Velocity, dq)?;
        bound::magnitude(Quantity::Velocity, dq, &limits.dq_max)?;
        let ranged = bound::bounded(&limits.q_max)
            || limits.q_min.iter().any(|&min| min != f64::NEG_INFINITY);
        let q = bound::measured(Quantity::Position, state.meas.q(), ranged)?;
        for joint in 0..N {
            let (v, min, max) = (dq[joint], limits.q_min[joint], limits.q_max[joint]);
            let stop = q[joint] + v * dt + v * v.abs() / (2. * limits.ddq_max[joint]);
            let outward = (v > 0. && !within(stop, max)) || (v < 0. && !within(min, stop));
            if outward {
                return Err(SafetyViolation::OutOfRange {
                    quantity: Quantity::Position,
                    axis: Axis::Joint(joint),
                    value: stop,
                    min,
                    max,
                });
            }
        }
        let dq0 = match self.previous {
            Some(previous) => previous,
            None => bound::measured(
                Quantity::Velocity,
                state.meas.dq(),
                bound::bounded(&limits.ddq_max),
            )?,
        };
        bound::rate(Quantity::Acceleration, &dq0, dq, &limits.ddq_max, dt)?;
        Ok(*dq)
    }
}

impl<const N: usize> CommandCheck for JointVelocityCheck<N> {
    type Obs = JointState<N>;
    type Command = [f64; N];

    fn check(
        &mut self,
        state: &JointState<N>,
        dt: Duration,
        dq: &[f64; N],
    ) -> Result<(), SafetyViolation> {
        let result = self.evaluate(state, dt.as_secs_f64(), dq);
        self.previous = result.as_ref().ok().copied();
        result.map(drop)
    }

    fn reset(&mut self) {
        self.previous = None;
    }
}

/// Joint torque commands: within `tau_max`, and changing no faster than the
/// effort-rate limit, which is unbounded unless set.
///
/// What a torque command contains is driver-specific (Franka's excludes
/// gravity), so the first command of a session is measured against an
/// explicit start, zero unless set, and never against a measured torque.
#[derive(Clone, Debug)]
pub struct JointTorqueCheck<const N: usize> {
    tau_max: [f64; N],
    dtau_max: [f64; N],
    start: [f64; N],
    previous: Option<[f64; N]>,
}

impl<const N: usize> JointTorqueCheck<N> {
    pub fn new(limits: JointLimits<N>) -> Self {
        Self {
            tau_max: limits.tau_max,
            dtau_max: [f64::INFINITY; N],
            start: [0.; N],
            previous: None,
        }
    }

    /// Bounds the change between commands, e.g. `Joints::TORQUE_DOT_BOUND`.
    pub fn with_effort_rate(mut self, dtau_max: [f64; N]) -> Self {
        self.dtau_max = dtau_max;
        self
    }

    /// The torque the first command of a session is measured against.
    pub fn starting_at(mut self, tau: [f64; N]) -> Self {
        self.start = tau;
        self
    }

    fn evaluate(&self, dt: f64, tau: &[f64; N]) -> Result<[f64; N], SafetyViolation> {
        bound::finite(Quantity::Effort, tau)?;
        bound::magnitude(Quantity::Effort, tau, &self.tau_max)?;
        let tau0 = self.previous.unwrap_or(self.start);
        bound::rate(Quantity::EffortRate, &tau0, tau, &self.dtau_max, dt)?;
        Ok(*tau)
    }
}

impl<const N: usize> CommandCheck for JointTorqueCheck<N> {
    type Obs = JointState<N>;
    type Command = [f64; N];

    fn check(
        &mut self,
        _state: &JointState<N>,
        dt: Duration,
        tau: &[f64; N],
    ) -> Result<(), SafetyViolation> {
        let result = self.evaluate(dt.as_secs_f64(), tau);
        self.previous = result.as_ref().ok().copied();
        result.map(drop)
    }

    fn reset(&mut self) {
        self.previous = None;
    }
}
