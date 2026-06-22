use nalgebra as na;

use crate::{
    RobotResult,
    robot::{
        kinematics_dynamics::{
            ArmForwardKinematics, ArmInverseKinematics, JMat, JVec, Jaco, Twist,
        },
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

impl<T, const N: usize> SpaceMap<JointSpace<N>, FlangeSpace> for T
where
    T: ArmForwardKinematics<N>,
    [(); N + 1]:,
{
    type Input = [f64; N];
    type Output = Pose;

    fn map(&self, input: Self::Input) -> RobotResult<Self::Output> {
        Ok(T::fk_end_pose(&input))
    }
}

/// Inverse kinematics as a typed map from flange / end-effector pose space back
/// to joint space.
///
/// The default map uses the model's default joint posture as the seed and the
/// default [`IKMethod`](crate::IKMethod). More specialised callers can still use
/// [`ArmInverseKinematics`] directly when they need explicit solver control.
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

impl<T, const N: usize> SpaceMap<FlangeSpace, JointSpace<N>> for T
where
    T: ArmInverseKinematics<N>,
    [(); N + 1]:,
    na::Const<N>: na::DimMin<na::Const<N>, Output = na::Const<N>> + na::ToTypenum,
{
    type Input = Pose;
    type Output = [f64; N];

    fn map(&self, input: Self::Input) -> RobotResult<Self::Output> {
        let q0 = JVec::<N>::from_row_slice(&T::JOINT_DEFAULT);
        let q = T::ik_solve(&q0, &input, Default::default());
        Ok(q.as_slice().try_into().unwrap())
    }
}

/// Geometric Jacobian model as a typed map from joint space to Jacobian space.
pub trait JacobianModel<const N: usize>:
    SpaceMap<JointSpace<N>, JacobianSpace<N>, Input = [f64; N], Output = Jaco<N>>
{
    fn jacobian(&self, q: [f64; N]) -> RobotResult<Jaco<N>> {
        self.map(q)
    }

    fn twist(&self, q: [f64; N], dq: [f64; N]) -> RobotResult<Twist> {
        Ok(self.jacobian(q)? * JVec::<N>::from_column_slice(&dq))
    }
}

impl<T, const N: usize> JacobianModel<N> for T where
    T: SpaceMap<JointSpace<N>, JacobianSpace<N>, Input = [f64; N], Output = Jaco<N>>
{
}

impl<T, const N: usize> SpaceMap<JointSpace<N>, JacobianSpace<N>> for T
where
    T: ArmForwardKinematics<N>,
    [(); N + 1]:,
{
    type Input = [f64; N];
    type Output = Jaco<N>;

    fn map(&self, input: Self::Input) -> RobotResult<Self::Output> {
        Ok(T::kine_cache(&input, &[0.; N]).jacobian())
    }
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
