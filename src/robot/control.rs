use std::time::Duration;

use futures::executor;

use crate::{
    Robot, RobotResult,
    robot::{
        arm::ArmState,
        state::{BaseState, JointState},
        types::Pose,
    },
};

/// Type-level mapping from a realtime *control channel* to its observation and
/// command types.
///
/// This is the control-side mirror of
/// [`MotionSpace`](crate::robot::motion::MotionSpace): it has **no methods** and
/// only fixes the [`Obs`](ControlSpace::Obs) / [`Command`](ControlSpace::Command)
/// pair. Because the channel is a *type-level marker*, two channels whose command
/// payloads share the same Rust type, e.g. joint torque and joint position are
/// both `[f64; N]`, they stay distinguishable, so a single device can implement one
/// [`ControlWith`] impl per channel without coherence conflicts.
pub trait ControlSpace<R: ?Sized> {
    /// The observed-state type handed to the closure each control cycle.
    type Obs;
    /// The command type the closure returns each control cycle.
    type Command;
}

/// Realtime joint-**torque** channel. Observes [`JointState<N>`] and commands a
/// per-joint torque vector `[f64; N]`.
pub struct TorqueControl<const N: usize>;
/// Realtime arm torque channel for controllers that need full arm context.
pub struct ArmTorqueControl<const N: usize>;
/// Realtime joint-**position** channel. Observes [`JointState<N>`] and commands a
/// per-joint position vector `[f64; N]`.
pub struct JointPositionControl<const N: usize>;
/// Realtime joint-**velocity** channel. Observes [`JointState<N>`] and commands a
/// per-joint velocity vector `[f64; N]`.
pub struct JointVelocityControl<const N: usize>;
/// Realtime **cartesian-velocity** channel. Observes [`ArmState<N>`] and commands
/// a 6-vector spatial velocity `[vx, vy, vz, wx, wy, wz]`.
pub struct CartesianVelocityControl<const N: usize>;
/// Realtime **cartesian-pose** channel. Observes [`ArmState<N>`] and commands a
/// flange / end-effector [`Pose`].
pub struct CartesianPoseControl<const N: usize>;
/// Realtime mobile / floating base velocity channel. Observes [`BaseState`] and
/// commands `[vx, vy, vz, wx, wy, wz]`.
pub struct BaseVelocityControl;
/// Realtime balance channel. The command shape is deliberately small until a
/// concrete humanoid or legged consumer needs richer balance commands.
pub struct BalanceControl;

impl<const N: usize, R> ControlSpace<R> for TorqueControl<N> {
    type Obs = JointState<N>;
    type Command = [f64; N];
}

impl<const N: usize, R> ControlSpace<R> for ArmTorqueControl<N> {
    type Obs = ArmState<N>;
    type Command = [f64; N];
}

impl<const N: usize, R> ControlSpace<R> for JointPositionControl<N> {
    type Obs = JointState<N>;
    type Command = [f64; N];
}

impl<const N: usize, R> ControlSpace<R> for JointVelocityControl<N> {
    type Obs = JointState<N>;
    type Command = [f64; N];
}

impl<const N: usize, R> ControlSpace<R> for CartesianVelocityControl<N> {
    type Obs = ArmState<N>;
    type Command = [f64; 6];
}

impl<const N: usize, R> ControlSpace<R> for CartesianPoseControl<N> {
    type Obs = ArmState<N>;
    type Command = Pose;
}

impl<R> ControlSpace<R> for BaseVelocityControl {
    type Obs = BaseState;
    type Command = [f64; 6];
}

impl<R> ControlSpace<R> for BalanceControl {
    type Obs = BaseState;
    type Command = [f64; 6];
}

/// Realtime, closure-driven control - the bridge into blocking control loops.
/// **Implemented by drivers**, one impl per control channel `S`.
///
/// Generic over the [`ControlSpace`] channel `S`, which fixes both the observed
/// state and the command type. The observation is decoupled from
/// [`Robot::State`](crate::Robot::State) on purpose: a driver's lifecycle
/// `State` may be a rich native struct, while the realtime loop observes a
/// task-shaped view. The closure is invoked once per control cycle with the
/// freshly observed state and the elapsed cycle time, and returns the next
/// command together with a `done` flag.
///
/// `control_with` runs a control relation until the closure reports `done = true`
/// or the driver returns an error. The closure is scoped to the call, so drivers
/// should not store it after `control_with` returns.
pub trait ControlWith<S: ControlSpace<Self>>: Robot {
    /// Build a continuity-preserving fallback command from the latest observed
    /// state.
    ///
    /// This is deliberately not named "safe": physical safety belongs to the
    /// driver/controller stack, often using the previous command in `obs` to
    /// rate-limit the next command. `hold_command` only answers what this
    /// channel should emit when upstream has no fresh output for this cycle.
    fn hold_command(obs: &S::Obs) -> S::Command
    where
        Self: Sized;

    /// Run a realtime control loop until the closure signals completion.
    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(S::Obs, Duration) -> (S::Command, bool);

    /// Blocking control loop that accepts an async per-cycle closure.
    ///
    /// The default implementation adapts an async controller back into the
    /// blocking [`control_with`](ControlWith::control_with) loop by running one
    /// controller future to completion per cycle. The method itself is still
    /// blocking: it returns only when the control loop finishes or errors.
    fn control_with_async<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: async FnMut(S::Obs, Duration) -> (S::Command, bool),
    {
        self.control_with(move |obs, duration| executor::block_on(closure(obs, duration)))
    }
}

/// Ergonomic, channel-parameterised entry point. **Blanket-implemented for every
/// robot** - users never implement this trait, they only call it:
///
/// ```ignore
/// arm.control_with::<TorqueControl<7>, _>(|s, dt| (tau, false))?;
/// arm.control_with::<JointPositionControl<7>, _>(|s, dt| (q, false))?;
/// ```
///
/// The channel `S` is a method-level generic disambiguated by turbofish, so
/// channels that share a command type (torque vs. position, both `[f64; N]`)
/// never collide. Each call forwards to the [`ControlWith<S>`] impl the
/// driver provides for that channel.
pub trait Control: Robot + Sized {
    fn control_with<S, F>(&mut self, closure: F) -> RobotResult<()>
    where
        S: ControlSpace<Self>,
        Self: ControlWith<S>,
        F: FnMut(S::Obs, Duration) -> (S::Command, bool),
    {
        <Self as ControlWith<S>>::control_with(self, closure)
    }

    /// The continuity fallback command for channel `S`. See
    /// [`ControlWith::hold_command`].
    fn hold_command<S>(&self, obs: &S::Obs) -> S::Command
    where
        S: ControlSpace<Self>,
        Self: ControlWith<S> + Sized,
    {
        <Self as ControlWith<S>>::hold_command(obs)
    }

    /// Blocking async-closure facade for channel `S`.
    /// See [`ControlWith::control_with_async`].
    fn control_with_async<S, F>(&mut self, closure: F) -> RobotResult<()>
    where
        S: ControlSpace<Self>,
        Self: ControlWith<S>,
        F: async FnMut(S::Obs, Duration) -> (S::Command, bool),
    {
        <Self as ControlWith<S>>::control_with_async(self, closure)
    }
}

impl<R: Robot> Control for R {}
