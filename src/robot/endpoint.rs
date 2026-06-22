/// Cartesian-space limits of an arm's end effector.
///
/// Where [`Joints`](crate::Joints) describes per-joint limits, `EndPoint`
/// describes the task-space envelope of the flange / TCP: how fast it may
/// translate and rotate. Linear quantities use m/s, m/s^2 and m/s^3; angular
/// quantities use rad/s, rad/s^2 and rad/s^3. Every constant defaults to
/// unbounded `f64::MAX`, so a driver overrides only the limits its firmware
/// enforces.
///
/// `EndPoint` makes [`EndSpace`](crate::EndSpace) a valid
/// [`MotionSpace`](crate::MotionSpace) and is a super-trait of
/// [`Arm<N>`](crate::Arm).
///
/// # Example
/// ```ignore
/// use robot_behavior::EndPoint;
///
/// impl EndPoint for MyArm {
///     const CARTESIAN_VEL_BOUND: f64 = 2.0; // 2 m/s
///     const ROTATION_VEL_BOUND: f64 = std::f64::consts::PI; // pi rad/s
/// }
/// ```
pub trait EndPoint {
    /// Maximum linear speed of the end effector (m/s). Defaults to unbounded.
    const CARTESIAN_VEL_BOUND: f64 = f64::MAX;
    /// Maximum linear acceleration (m/s^2). Defaults to unbounded.
    const CARTESIAN_ACC_BOUND: f64 = f64::MAX;
    /// Maximum linear jerk (m/s^3). Defaults to unbounded.
    const CARTESIAN_JERK_BOUND: f64 = f64::MAX;
    /// Maximum angular speed (rad/s). Defaults to unbounded.
    const ROTATION_VEL_BOUND: f64 = f64::MAX;
    /// Maximum angular acceleration (rad/s^2). Defaults to unbounded.
    const ROTATION_ACC_BOUND: f64 = f64::MAX;
    /// Maximum angular jerk (rad/s^3). Defaults to unbounded.
    const ROTATION_JERK_BOUND: f64 = f64::MAX;
}
