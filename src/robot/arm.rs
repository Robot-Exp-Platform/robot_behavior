use crate::{
    EndPoint, JointSample, JointState, Robot, RobotResult, SpatialSample, StateView,
    robot::{
        joint::Joints,
        load::LoadState,
        motion::MoveTo,
        spaces::{FlangeSpace, JointSpace},
        types::Pose,
    },
};

#[derive(Debug, Clone)]
pub struct ArmState<const N: usize> {
    /// Joint-space measured / commanded / desired samples.
    pub joint: JointState<N>,
    /// Flange frame pose / twist / wrench samples.
    pub flange: StateView<SpatialSample>,
    /// Optional tool-center-point state when the driver exposes TCP separately
    /// from the flange.
    pub tcp: Option<StateView<SpatialSample>>,
    /// Optional stiffness / compliance frame state.
    ///
    /// Some robots, such as Franka, expose a frame used by Cartesian impedance
    /// or external wrench estimation. It is modeled separately from `flange`
    /// and `tcp` because it is a physical/control frame, not a gain matrix.
    pub stiffness: Option<StateView<SpatialSample>>,
    /// Payload / load attached to the arm.
    pub load: Option<LoadState>,
}

impl<const N: usize> Default for ArmState<N> {
    fn default() -> Self {
        Self {
            joint: JointState::default(),
            flange: StateView::default(),
            tcp: None,
            stiffness: None,
            load: Some(LoadState { m: 0., x: [0.; 3], i: [0.; 9] }),
        }
    }
}

impl<const N: usize> ArmState<N> {
    pub fn from_meas(
        joint: JointSample<N>,
        flange: SpatialSample,
        load: Option<LoadState>,
    ) -> Self {
        Self {
            joint: StateView::from_meas(joint),
            flange: StateView::from_meas(flange),
            load,
            ..Default::default()
        }
    }
}
/// An `N`-DoF robotic arm: the central capability trait drivers implement.
///
/// `Arm<N>` ties together everything needed to command a serial manipulator:
/// the lifecycle of a [`Robot`], the joint limits of [`Joints<N>`], the
/// Cartesian limits of [`EndPoint`], and the ability to [`MoveTo`] both a
/// [`JointSpace<N>`] and a [`FlangeSpace`] target. On top of those it adds
/// state read-back, payload configuration, and ergonomic accessors/builders
/// for the motion limits.
///
/// The `get_*_bound` methods simply surface the associated constants from
/// [`Joints`] / [`EndPoint`], while the `with_*` builders return `Self` so a
/// caller can temporarily tighten a limit for a single motion before calling
/// `Motion::move_to::<JointSpace<N>>`.
pub trait Arm<const N: usize>:
    Robot + Joints<N> + EndPoint + MoveTo<JointSpace<N>> + MoveTo<FlangeSpace> + Sized
{
    /// Read the full [`ArmState`] (measured / commanded / desired + load).
    fn state(&mut self) -> RobotResult<ArmState<N>>;
    /// Configure the payload attached to the flange.
    fn set_load(&mut self, load: LoadState) -> RobotResult<()>;

    /// Current joint positions (rad).
    fn get_joint(&self) -> [f64; N];
    /// Lower joint limits; defaults to [`Joints::JOINT_MIN`].
    fn get_joint_min(&self) -> [f64; N] {
        Self::JOINT_MIN
    }
    /// Upper joint limits; defaults to [`Joints::JOINT_MAX`].
    fn get_joint_max(&self) -> [f64; N] {
        Self::JOINT_MAX
    }
    /// Joint velocity limits; defaults to [`Joints::JOINT_VEL_BOUND`].
    fn get_joint_vel_bound(&self) -> [f64; N] {
        Self::JOINT_VEL_BOUND
    }
    /// Joint acceleration limits; defaults to [`Joints::JOINT_ACC_BOUND`].
    fn get_joint_acc_bound(&self) -> [f64; N] {
        Self::JOINT_ACC_BOUND
    }
    /// Joint jerk limits; defaults to [`Joints::JOINT_JERK_BOUND`].
    fn get_joint_jerk_bound(&self) -> [f64; N] {
        Self::JOINT_JERK_BOUND
    }
    /// Joint torque limits; defaults to [`Joints::TORQUE_BOUND`].
    fn get_torque_bound(&self) -> [f64; N] {
        Self::TORQUE_BOUND
    }
    /// Joint torque-rate limits; defaults to [`Joints::TORQUE_DOT_BOUND`].
    fn get_torque_dot_bound(&self) -> [f64; N] {
        Self::TORQUE_DOT_BOUND
    }

    /// Override the joint velocity limit for subsequent motions.
    fn with_joint_vel(self, vel_bound: [f64; N]) -> Self;
    /// Override the joint acceleration limit for subsequent motions.
    fn with_joint_acc(self, acc_bound: [f64; N]) -> Self;
    /// Override the joint jerk limit for subsequent motions.
    fn with_joint_jerk(self, jerk_bound: [f64; N]) -> Self;
    /// Override the joint torque limit for subsequent motions.
    fn with_torque(self, torque_bound: [f64; N]) -> Self;
    /// Override the joint torque-rate limit for subsequent motions.
    fn with_torque_dot(self, torque_dot_bound: [f64; N]) -> Self;

    /// Current end-effector [`Pose`] in the base frame.
    fn get_endpoint(&self) -> Pose;
    /// Cartesian velocity limit; defaults to [`EndPoint::CARTESIAN_VEL_BOUND`].
    fn get_cartesian_vel_bound(&self) -> f64 {
        Self::CARTESIAN_VEL_BOUND
    }
    /// Cartesian acceleration limit; defaults to [`EndPoint::CARTESIAN_ACC_BOUND`].
    fn get_cartesian_acc_bound(&self) -> f64 {
        Self::CARTESIAN_ACC_BOUND
    }
    /// Cartesian jerk limit; defaults to [`EndPoint::CARTESIAN_JERK_BOUND`].
    fn get_cartesian_jerk_bound(&self) -> f64 {
        Self::CARTESIAN_JERK_BOUND
    }
    /// Rotational velocity limit; defaults to [`EndPoint::ROTATION_VEL_BOUND`].
    fn get_rotation_vel_bound(&self) -> f64 {
        Self::ROTATION_VEL_BOUND
    }
    /// Rotational acceleration limit; defaults to [`EndPoint::ROTATION_ACC_BOUND`].
    fn get_rotation_acc_bound(&self) -> f64 {
        Self::ROTATION_ACC_BOUND
    }
    /// Rotational jerk limit; defaults to [`EndPoint::ROTATION_JERK_BOUND`].
    fn get_rotation_jerk_bound(&self) -> f64 {
        Self::ROTATION_JERK_BOUND
    }

    /// Override the Cartesian velocity limit for subsequent motions.
    fn with_cartesian_vel(self, vel_bound: f64) -> Self;
    /// Override the Cartesian acceleration limit for subsequent motions.
    fn with_cartesian_acc(self, acc_bound: f64) -> Self;
    /// Override the Cartesian jerk limit for subsequent motions.
    fn with_cartesian_jerk(self, jerk_bound: f64) -> Self;
    /// Override the rotational velocity limit for subsequent motions.
    fn with_rotation_vel(self, vel_bound: f64) -> Self;
    /// Override the rotational acceleration limit for subsequent motions.
    fn with_rotation_acc(self, acc_bound: f64) -> Self;
    /// Override the rotational jerk limit for subsequent motions.
    fn with_rotation_jerk(self, jerk_bound: f64) -> Self;
}
