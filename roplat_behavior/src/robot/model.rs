use roplat_msgs::Twist;

use crate::{
    RobotResult,
    robot::{
        math::{JMat, JVec, Jaco},
        motion::MotionSpace,
        spaces::{
            CoriolisInputSpace, FlangeSpace, GravityInputSpace, JacobianSpace, JointSpace,
            JointTorqueSpace, MassMatrixSpace,
        },
        types::Pose,
    },
};

/// Generic mapping from one named space to another.
///
/// `From` and `To` are type-level space markers. `Input` and `Output` are kept
/// explicit so a mapping can carry extra context when needed, such as velocity
/// for Coriolis terms or gravity direction for gravity compensation.
pub trait SpaceMap<From, To> {
    type Input;
    type Output;

    fn map(&self, input: Self::Input) -> RobotResult<Self::Output>;
}

/// Convenience adapter for maps whose input/output exactly match
/// [`MotionSpace`] target types for the implementor.
pub trait TypedSpaceMap<From, To>: Sized
where
    From: MotionSpace<Self>,
    To: MotionSpace<Self>,
{
    fn map_space(&self, input: From::Target) -> RobotResult<To::Target>;
}

impl<T, From, To> TypedSpaceMap<From, To> for T
where
    T: SpaceMap<From, To> + Sized,
    From: MotionSpace<T, Target = <T as SpaceMap<From, To>>::Input>,
    To: MotionSpace<T, Target = <T as SpaceMap<From, To>>::Output>,
{
    fn map_space(&self, input: From::Target) -> RobotResult<To::Target> {
        <T as SpaceMap<From, To>>::map(self, input)
    }
}

/// Joint position and velocity input for Coriolis / centrifugal terms.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CoriolisInput<const N: usize> {
    pub q: [f64; N],
    pub dq: [f64; N],
}

/// Joint position plus gravity vector input for gravity terms.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GravityInput<const N: usize> {
    pub q: [f64; N],
    pub gravity: [f64; 3],
}

/// Forward kinematics as a typed map from joint space to flange / end-effector
/// pose space.
pub trait ForwardKinematics<const N: usize>:
    SpaceMap<JointSpace<N>, FlangeSpace, Input = [f64; N], Output = Pose>
{
    fn forward_kinematics(&self, q: [f64; N]) -> RobotResult<Pose> {
        self.map(q)
    }
}

impl<T, const N: usize> ForwardKinematics<N> for T where
    T: SpaceMap<JointSpace<N>, FlangeSpace, Input = [f64; N], Output = Pose>
{
}

/// Inverse kinematics as a typed map from flange / end-effector pose space back
/// to joint space.
///
/// `roplat_kinematics::dh_kinematics!(Type, N, inverse)` implements this map
/// for a Denavit–Hartenberg arm.
pub trait InverseKinematics<const N: usize>:
    SpaceMap<FlangeSpace, JointSpace<N>, Input = Pose, Output = [f64; N]>
{
    fn inverse_kinematics(&self, target: Pose) -> RobotResult<[f64; N]> {
        self.map(target)
    }
}

impl<T, const N: usize> InverseKinematics<N> for T where
    T: SpaceMap<FlangeSpace, JointSpace<N>, Input = Pose, Output = [f64; N]>
{
}

/// Geometric Jacobian model as a typed map from joint space to Jacobian space.
pub trait JacobianModel<const N: usize>:
    SpaceMap<JointSpace<N>, JacobianSpace<N>, Input = [f64; N], Output = Jaco<N>>
{
    fn jacobian(&self, q: [f64; N]) -> RobotResult<Jaco<N>> {
        self.map(q)
    }

    fn twist(&self, q: [f64; N], dq: [f64; N]) -> RobotResult<Twist> {
        Ok((self.jacobian(q)? * JVec::<N>::from_column_slice(&dq)).into())
    }
}

impl<T, const N: usize> JacobianModel<N> for T where
    T: SpaceMap<JointSpace<N>, JacobianSpace<N>, Input = [f64; N], Output = Jaco<N>>
{
}

/// Dynamics model as typed maps for mass, Coriolis / centrifugal and gravity
/// terms.
pub trait DynamicsModel<const N: usize>: SpaceMap<JointSpace<N>, MassMatrixSpace<N>, Input = [f64; N], Output = JMat<N>>
    + SpaceMap<
        CoriolisInputSpace<N>,
        JointTorqueSpace<N>,
        Input = CoriolisInput<N>,
        Output = [f64; N],
    > + SpaceMap<GravityInputSpace<N>, JointTorqueSpace<N>, Input = GravityInput<N>, Output = [f64; N]>
{
    fn mass(&self, q: [f64; N]) -> RobotResult<JMat<N>> {
        <Self as SpaceMap<JointSpace<N>, MassMatrixSpace<N>>>::map(self, q)
    }

    fn coriolis(&self, q: [f64; N], dq: [f64; N]) -> RobotResult<[f64; N]> {
        <Self as SpaceMap<CoriolisInputSpace<N>, JointTorqueSpace<N>>>::map(
            self,
            CoriolisInput { q, dq },
        )
    }

    fn gravity(&self, q: [f64; N], gravity: [f64; 3]) -> RobotResult<[f64; N]> {
        <Self as SpaceMap<GravityInputSpace<N>, JointTorqueSpace<N>>>::map(
            self,
            GravityInput { q, gravity },
        )
    }
}

impl<T, const N: usize> DynamicsModel<N> for T where
    T: SpaceMap<JointSpace<N>, MassMatrixSpace<N>, Input = [f64; N], Output = JMat<N>>
        + SpaceMap<
            CoriolisInputSpace<N>,
            JointTorqueSpace<N>,
            Input = CoriolisInput<N>,
            Output = [f64; N],
        > + SpaceMap<
            GravityInputSpace<N>,
            JointTorqueSpace<N>,
            Input = GravityInput<N>,
            Output = [f64; N],
        >
{
}
