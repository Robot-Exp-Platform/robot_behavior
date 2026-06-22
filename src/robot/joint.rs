/// A robot with a fixed number `N` of actuated joints, together with the
/// compile-time kinematic and dynamic limits of those joints.
///
/// The constants describe the physical envelope of one specific robot model.
/// Every value is a per-joint array of length `N`; angles are in radians,
/// velocities in rad/s, accelerations in rad/s^2, jerk in rad/s^3 and torques
/// in N*m. Only [`JOINT_MIN`](Joints::JOINT_MIN) and
/// [`JOINT_MAX`](Joints::JOINT_MAX) are required; the remaining limits default
/// to an effectively unbounded `f64::MAX`, so a driver only declares what its
/// firmware actually enforces.
///
/// Implementing `Joints<N>` makes [`JointSpace<N>`](crate::JointSpace) a valid
/// [`MotionSpace`](crate::MotionSpace) for the robot, and it is a super-trait of
/// [`Arm<N>`](crate::Arm).
///
/// # Example
/// ```ignore
/// use robot_behavior::{Joints, to_radians_array};
///
/// impl Joints<6> for MyArm {
///     const JOINT_MIN: [f64; 6] = to_radians_array([-170., -120., -170., -120., -170., -360.]);
///     const JOINT_MAX: [f64; 6] = to_radians_array([ 170.,  120.,  170.,  120.,  170.,  360.]);
///     const JOINT_VEL_BOUND: [f64; 6] = [3.14; 6];
/// }
/// ```
pub trait Joints<const N: usize> {
    /// The arm's home / reference posture (rad). Defaults to all-zeros.
    const JOINT_DEFAULT: [f64; N] = [0.; N];
    /// A compact "packed" posture for transport or storage (rad). Defaults to
    /// all-zeros.
    const JOINT_PACKED: [f64; N] = [0.; N];
    /// Lower joint position limits (rad). Required.
    const JOINT_MIN: [f64; N];
    /// Upper joint position limits (rad). Required.
    const JOINT_MAX: [f64; N];
    /// Maximum joint velocities (rad/s). Defaults to unbounded.
    const JOINT_VEL_BOUND: [f64; N] = [f64::MAX; N];
    /// Maximum joint accelerations (rad/s^2). Defaults to unbounded.
    const JOINT_ACC_BOUND: [f64; N] = [f64::MAX; N];
    /// Maximum joint jerk (rad/s^3). Defaults to unbounded.
    const JOINT_JERK_BOUND: [f64; N] = [f64::MAX; N];
    /// Maximum joint torques (N*m). Defaults to unbounded.
    const TORQUE_BOUND: [f64; N] = [f64::MAX; N];
    /// Maximum joint torque rate (N*m/s). Defaults to unbounded.
    const TORQUE_DOT_BOUND: [f64; N] = [f64::MAX; N];
}
