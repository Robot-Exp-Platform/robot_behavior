use super::limits::unbounded_above;
use crate::{CartesianLimits, Pose};

/// An arm's end effector: where its flange is, and its Cartesian limits.
///
/// Where [`Joints`](crate::Joints) describes per-joint limits, `EndPoint`
/// describes the task-space envelope of the flange / TCP: how fast it may
/// translate and rotate. Linear quantities use m/s, m/s^2 and m/s^3; angular
/// quantities use rad/s, rad/s^2 and rad/s^3. Every constant defaults to
/// unbounded, `f64::INFINITY`, so a driver overrides only the limits its
/// firmware enforces. A declared `f64::MAX` also reads as unbounded.
///
/// Its current pose makes [`FlangeSpace`](crate::FlangeSpace) a
/// [`RelativeSpace`](crate::RelativeSpace).
///
/// `EndPoint` makes [`EndSpace`](crate::EndSpace) a valid
/// [`MotionSpace`](crate::MotionSpace) and is a super-trait of
/// [`Arm<N>`](crate::Arm).
///
/// # Example
/// ```ignore
/// use roplat_behavior::EndPoint;
///
/// impl EndPoint for MyArm {
///     const CARTESIAN_VEL_BOUND: f64 = 2.0; // 2 m/s
///     const ROTATION_VEL_BOUND: f64 = std::f64::consts::PI; // pi rad/s
/// }
/// ```
pub trait EndPoint {
    /// Maximum linear speed of the end effector (m/s). Defaults to unbounded.
    const CARTESIAN_VEL_BOUND: f64 = f64::INFINITY;
    /// Maximum linear acceleration (m/s^2). Defaults to unbounded.
    const CARTESIAN_ACC_BOUND: f64 = f64::INFINITY;
    /// Maximum linear jerk (m/s^3). Defaults to unbounded.
    const CARTESIAN_JERK_BOUND: f64 = f64::INFINITY;
    /// Maximum angular speed (rad/s). Defaults to unbounded.
    const ROTATION_VEL_BOUND: f64 = f64::INFINITY;
    /// Maximum angular acceleration (rad/s^2). Defaults to unbounded.
    const ROTATION_ACC_BOUND: f64 = f64::INFINITY;
    /// Maximum angular jerk (rad/s^3). Defaults to unbounded.
    const ROTATION_JERK_BOUND: f64 = f64::INFINITY;

    /// Current flange pose in the base frame.
    fn get_endpoint(&self) -> Pose;

    /// The declared bounds as data. A bound at or beyond `f64::MAX` reads as
    /// unbounded (`f64::INFINITY`).
    fn cartesian_limits() -> CartesianLimits {
        CartesianLimits {
            linear_velocity: unbounded_above(Self::CARTESIAN_VEL_BOUND),
            linear_acceleration: unbounded_above(Self::CARTESIAN_ACC_BOUND),
            linear_jerk: unbounded_above(Self::CARTESIAN_JERK_BOUND),
            angular_velocity: unbounded_above(Self::ROTATION_VEL_BOUND),
            angular_acceleration: unbounded_above(Self::ROTATION_ACC_BOUND),
            angular_jerk: unbounded_above(Self::ROTATION_JERK_BOUND),
        }
    }
}
