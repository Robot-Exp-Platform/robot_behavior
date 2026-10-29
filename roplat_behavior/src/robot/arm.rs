use crate::{
    EndPoint, EndPointPose, JointSample, JointState, LoadState, MotionLimits, Robot, RobotResult,
    SpatialSample, StateView,
    robot::{
        joint::Joints,
        motion::MoveTo,
        spaces::{FlangeSpace, JointSpace},
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
            load: Some(LoadState::default()),
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
/// the lifecycle of a [`Robot`], the joint envelope of [`Joints<N>`], the
/// Cartesian envelope of [`EndPoint`], the flange pose of [`EndPointPose`],
/// and the ability to [`MoveTo`] both a [`JointSpace<N>`] and a
/// [`FlangeSpace`] target. On top of those it adds state read-back, payload
/// configuration and the [`MotionLimits`] its own planned motions respect.
///
/// A driver keeps one [`MotionLimits`] value, starting from
/// [`MotionLimits::envelope`], and reads it when it plans. Applying limits to
/// a single motion is [`Motion::with_limits`](crate::Motion::with_limits),
/// written once for every arm.
pub trait Arm<const N: usize>:
    Robot + Joints<N> + EndPoint + EndPointPose + MoveTo<JointSpace<N>> + MoveTo<FlangeSpace> + Sized
{
    /// Read the full [`ArmState`] (measured / commanded / desired + load).
    fn state(&mut self) -> RobotResult<ArmState<N>>;
    /// Configure the payload attached to the flange.
    fn set_load(&mut self, load: LoadState) -> RobotResult<()>;

    /// Current joint positions (rad). The flange pose is
    /// [`EndPointPose::get_endpoint`].
    fn get_joint(&self) -> [f64; N];

    /// The limits subsequent motions respect.
    fn motion_limits(&self) -> MotionLimits<N>;
    /// Replace the limits for subsequent motions. Implementations refuse
    /// limits looser than the declared envelope, which
    /// [`MotionLimits::within`] checks.
    fn set_motion_limits(&mut self, limits: MotionLimits<N>) -> RobotResult<()>;
}
