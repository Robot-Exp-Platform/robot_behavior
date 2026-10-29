use std::marker::PhantomData;

use serde::{Deserialize, Serialize};

use crate::robot::{
    arm::Arm,
    endpoint::{EndPoint, EndPointPose},
    joint::Joints,
    math::{JMat, Jaco},
    motion::{MotionSpace, MoveTo},
    types::Pose,
};
use crate::{RobotException, RobotResult};

/// Joint-level command space: a command carries the robot's full joint vector
/// `[f64; N]` in radians.
///
/// Valid for any robot that implements [`Joints<N>`]. Cartesian spaces such as
/// [`FlangeSpace`] and [`TcpSpace`] can reduce to this space through inverse
/// kinematics.
pub struct JointSpace<const N: usize>;

impl<const N: usize, R: Joints<N>> MotionSpace<R> for JointSpace<N> {
    type Target = [f64; N];
}

/// End-effector pose space with declared Cartesian limits.
///
/// This space is valid only for robots that implement [`EndPoint`]. Use
/// [`FlangeSpace`] or [`TcpSpace`] when the caller wants an unconstrained
/// semantic endpoint space.
pub struct EndSpace;

impl<R: EndPoint> MotionSpace<R> for EndSpace {
    type Target = Pose;
}

/// Motion space for commands to the robot's flange.
pub struct FlangeSpace;

impl<R> MotionSpace<R> for FlangeSpace {
    type Target = Pose;
}

/// Motion space for commands to the robot's tool center point (TCP).
pub struct TcpSpace;

impl<R> MotionSpace<R> for TcpSpace {
    type Target = Pose;
}

/// Command space for a mobile or floating base pose.
pub struct BasePoseSpace;

impl<R> MotionSpace<R> for BasePoseSpace {
    type Target = Pose;
}

/// Command space for mobile or floating base spatial velocity:
/// `[vx, vy, vz, wx, wy, wz]`.
pub struct BaseVelocitySpace;

impl<R> MotionSpace<R> for BaseVelocitySpace {
    type Target = [f64; 6];
}

/// Whole-body joint position command space for an articulated robot.
pub struct WholeBodyJointSpace<const N: usize>;

impl<const N: usize, R: Joints<N>> MotionSpace<R> for WholeBodyJointSpace<N> {
    type Target = [f64; N];
}

/// Whole-body joint velocity command space for an articulated robot.
pub struct WholeBodyVelocitySpace<const N: usize>;

impl<const N: usize, R: Joints<N>> MotionSpace<R> for WholeBodyVelocitySpace<N> {
    type Target = [f64; N];
}

/// Whole-body joint torque / force command space for an articulated robot.
pub struct WholeBodyTorqueSpace<const N: usize>;

impl<const N: usize, R: Joints<N>> MotionSpace<R> for WholeBodyTorqueSpace<N> {
    type Target = [f64; N];
}

/// Center-of-mass position command space.
pub struct CenterOfMassSpace;

impl<R> MotionSpace<R> for CenterOfMassSpace {
    type Target = [f64; 3];
}

/// Backend-neutral gait command for legged robots.
///
/// This is intentionally small; vendor-specific gait parameters should live in
/// driver-specific types until a shared need appears.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub enum GaitCommand {
    #[default]
    Stop,
    Stand,
    Walk {
        /// Desired linear velocity `[vx, vy, vz]`.
        linear: [f64; 3],
        /// Desired angular velocity `[wx, wy, wz]`.
        angular: [f64; 3],
    },
}

/// Gait command space for legged robots.
pub struct GaitSpace;

impl<R> MotionSpace<R> for GaitSpace {
    type Target = GaitCommand;
}

/// Foot pose command space for a leg indexed at the type level.
pub struct FootSpace<const LEG: usize>;

impl<const LEG: usize, R> MotionSpace<R> for FootSpace<LEG> {
    type Target = Pose;
}

/// Hand pose command space for a hand indexed at the type level.
pub struct HandSpace<const HAND: usize>;

impl<const HAND: usize, R> MotionSpace<R> for HandSpace<HAND> {
    type Target = Pose;
}

/// Space containing a geometric Jacobian.
pub struct JacobianSpace<const N: usize>;

impl<const N: usize, R> MotionSpace<R> for JacobianSpace<N> {
    type Target = Jaco<N>;
}

/// Space containing a joint-space mass matrix.
pub struct MassMatrixSpace<const N: usize>;

impl<const N: usize, R> MotionSpace<R> for MassMatrixSpace<N> {
    type Target = JMat<N>;
}

/// Space containing a joint-space torque / force vector.
pub struct JointTorqueSpace<const N: usize>;

impl<const N: usize, R> MotionSpace<R> for JointTorqueSpace<N> {
    type Target = [f64; N];
}

/// Input marker for Coriolis / centrifugal terms.
pub struct CoriolisInputSpace<const N: usize>;

/// Input marker for gravity terms.
pub struct GravityInputSpace<const N: usize>;

/// Wraps a [`MotionSpace`] so its target is an offset from where the robot is
/// rather than an absolute target: `move_to::<Relative<JointSpace<N>>>(dq)`
/// adds `dq` to the current joint positions, and
/// `move_to::<Relative<FlangeSpace>>(delta)` moves the flange by `delta`
/// expressed in the flange frame.
///
/// The contract implements [`MoveTo`] for `Relative<S>` once, for every robot
/// that can move to `S` and every `S` that is a [`RelativeSpace`]: it reads
/// the current target, offsets it and moves there. Drivers implement nothing.
pub struct Relative<S>(PhantomData<S>);

impl<R, S: MotionSpace<R>> MotionSpace<R> for Relative<S> {
    type Target = S::Target;
}

/// A [`MotionSpace`] whose targets compose: where the robot is now, and a
/// target moved by an offset.
pub trait RelativeSpace<R: ?Sized>: MotionSpace<R> {
    /// The robot's current target in this space, read from the robot.
    fn current(robot: &mut R) -> RobotResult<Self::Target>;
    /// `current` moved by `delta`.
    fn offset(current: &Self::Target, delta: &Self::Target) -> Self::Target;
}

/// The measured joint positions ([`Arm::state`]) plus the offset.
impl<R: Arm<N>, const N: usize> RelativeSpace<R> for JointSpace<N> {
    fn current(robot: &mut R) -> RobotResult<[f64; N]> {
        robot.state()?.joint.meas.q().ok_or_else(|| {
            RobotException::UnprocessableInstructionError(
                "a relative joint motion needs the measured joint positions".into(),
            )
        })
    }
    fn offset(current: &[f64; N], delta: &[f64; N]) -> [f64; N] {
        std::array::from_fn(|i| current[i] + delta[i])
    }
}

/// The flange pose followed by the offset, which is expressed in the flange
/// frame: a translation along the flange's own axes, then a rotation about
/// them.
impl<R: EndPointPose> RelativeSpace<R> for FlangeSpace {
    fn current(robot: &mut R) -> RobotResult<Pose> {
        robot.read_endpoint()
    }
    fn offset(current: &Pose, delta: &Pose) -> Pose {
        *current * *delta
    }
}

impl<R, S> MoveTo<Relative<S>> for R
where
    R: MoveTo<S>,
    S: RelativeSpace<R>,
{
    fn move_to(&mut self, delta: S::Target) -> RobotResult<()> {
        let target = S::offset(&S::current(self)?, &delta);
        <R as MoveTo<S>>::move_to(self, target)
    }

    async fn move_to_async(&mut self, delta: S::Target) -> RobotResult<()> {
        let target = S::offset(&S::current(self)?, &delta);
        <R as MoveTo<S>>::move_to_async(self, target).await
    }
}
