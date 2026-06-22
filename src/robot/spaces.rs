use std::marker::PhantomData;

use serde::{Deserialize, Serialize};

use crate::robot::{
    endpoint::EndPoint,
    joint::Joints,
    kinematics_dynamics::{JMat, Jaco},
    motion::MotionSpace,
    types::Pose,
};

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
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum GaitCommand {
    Stop,
    Stand,
    Walk {
        /// Desired linear velocity `[vx, vy, vz]`.
        linear: [f64; 3],
        /// Desired angular velocity `[wx, wy, wz]`.
        angular: [f64; 3],
    },
}

impl Default for GaitCommand {
    fn default() -> Self {
        Self::Stop
    }
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

/// Wraps another [`MotionSpace`] so its target is interpreted *relative* to the
/// robot's current pose (incremental motion) rather than in absolute
/// coordinates. The wrapped command target type is unchanged.
///
/// See also [`Coord::Relative`](crate::Coord).
pub struct Relative<S>(PhantomData<S>);

impl<R, S: MotionSpace<R>> MotionSpace<R> for Relative<S> {
    type Target = S::Target;
}

/// Wraps another [`MotionSpace`] so its target is interpreted in a fixed
/// *inertial* frame. The wrapped command target type is unchanged.
///
/// See also [`Coord::Inertial`](crate::Coord).
pub struct Inertial<S>(PhantomData<S>);

impl<R, S: MotionSpace<R>> MotionSpace<R> for Inertial<S> {
    type Target = S::Target;
}
