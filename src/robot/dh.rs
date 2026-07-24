use nalgebra as na;

/// Denavit-Hartenberg parameters for a robotic arm link.
/// - DH: Standard Denavit-Hartenberg parameters.
///   - theta: joint angle
///   - d: link offset
///   - r: link length
///   - alpha: link twist
/// - MDH: Modified Denavit-Hartenberg parameters.
///  - theta: joint angle
///  - d: link offset
///  - a: link length
///  - alpha: link twist
///    you can use the macros `dh_param!` and `mdh_param!` to create instances of this enum.
///    omitting the field names in the macro invocation is supported.
/// ```rust
/// use robot_behavior::{DhParam, dh_param, mdh_param};
/// let dh = dh_param! {theta: 0.0, d: 0.5, r: 0.3, alpha: std::f64::consts::FRAC_PI_2};
/// let dh = dh_param! {0.0, 0.5, 0.3, std::f64::consts::FRAC_PI_2};
/// let mdh = mdh_param! {theta: 0.0, d: 0.5, a: 0.3, alpha: std::f64::consts::FRAC_PI_2};
/// let mdh = mdh_param! {0.0, 0.5, 0.3, std::f64::consts::FRAC_PI_2};
/// ```
#[derive(Debug,  Clone, Copy)]
#[rustfmt::skip] 
pub enum DhParam {
    DH { theta: f64, d: f64, r: f64, alpha: f64 },
    MDH { alpha: f64 ,a: f64,theta: f64, d: f64,  },
    Iso3Quat { pos: [f64; 3], quat: [f64; 4] },
    Iso3RPY { pos: [f64; 3], rpy: [f64; 3] },
}

#[macro_export]
#[rustfmt::skip]
macro_rules! dh_param {
    ($theta:expr, $d:expr, $r:expr, $alpha:expr $(,)?) => {
        $crate::DhParam::DH {theta: $theta, d: $d, r: $r, alpha: $alpha}
    };
    (theta : $theta:expr, d : $d:expr, r : $r:expr, alpha : $alpha:expr $(,)?) => {
        $crate::DhParam::DH {theta: $theta, d: $d, r: $r, alpha: $alpha}
    };
}
#[macro_export]
macro_rules! mdh_param {
    ($theta:expr, $d:expr, $a:expr, $alpha:expr $(,)?) => {
        $crate::DhParam::MDH { theta: $theta, d: $d, a: $a, alpha: $alpha }
    };
    (theta : $theta:expr, d : $d:expr, a : $a:expr, alpha : $alpha:expr $(,)?) => {
        $crate::DhParam::MDH { theta: $theta, d: $d, a: $a, alpha: $alpha }
    };
}

impl DhParam {
    #[inline(always)]
    pub fn to_se3(&self, q: f64) -> na::Isometry3<f64> {
        match self {
            DhParam::DH { theta, d, r, alpha } => {
                // Standard DH: T = Rz(θ+q) · Tz(d) · Tx(r) · Rx(α)
                //   R = Rz(θ+q) · Rx(α)
                //   t = Rz(θ+q) · (r, 0, d) = (r·cos(θ+q), r·sin(θ+q), d)
                let tq = theta + q;
                let (s, c) = tq.sin_cos();
                let translation = na::Translation3::new(r * c, r * s, *d);
                let rotation = na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), tq)
                    * na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), *alpha);
                na::Isometry::from_parts(translation, rotation)
            }
            DhParam::MDH { theta, d, a, alpha } => {
                // Khalil-Kleinfinger MDH: T = Rx(α) · Tx(a) · Rz(θ+q) · Tz(d)
                //   R = Rx(α) · Rz(θ+q)
                //   t = (a, -d·sin α, d·cos α)
                let (s, c) = alpha.sin_cos();
                let translation = na::Translation3::new(*a, -d * s, d * c);
                let rotation = na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), *alpha)
                    * na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), theta + q);
                na::Isometry::from_parts(translation, rotation)
            }
            DhParam::Iso3Quat { pos, quat } => {
                let translation = na::Translation3::new(pos[0], pos[1], pos[2]);
                let rotation = na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                    quat[0], quat[1], quat[2], quat[3],
                ));
                let joint_rotation = na::UnitQuaternion::from_euler_angles(0.0, 0.0, q);
                na::Isometry::from_parts(translation, rotation)
                    * na::Isometry::from_parts(na::Translation3::identity(), joint_rotation)
            }
            DhParam::Iso3RPY { pos, rpy } => {
                let translation = na::Translation3::new(pos[0], pos[1], pos[2]);
                let rotation = na::UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]);
                let joint_rotation = na::UnitQuaternion::from_euler_angles(0.0, 0.0, q);
                na::Isometry::from_parts(translation, rotation)
                    * na::Isometry::from_parts(na::Translation3::identity(), joint_rotation)
            }
        }
    }

    pub const fn default() -> Self {
        dh_param! {0.0, 0.0, 0.0, 0.0}
    }

    /// Constant transform applied BEFORE the joint actuation `Rz(θ+q)`.
    /// The joint's revolute axis (world z of the resulting frame) is
    /// `t_prefix[i] * pre_actuation()` applied to (0,0,1). Used by the
    /// geometric Jacobian to handle DH/MDH conventions uniformly.
    #[inline(always)]
    pub fn pre_actuation(&self) -> na::Isometry3<f64> {
        match self {
            // DH: Rz(θ+q) is applied FIRST, so pre = identity.
            DhParam::DH { .. } => na::Isometry3::identity(),
            // MDH (Khalil): T = Rx(α)·Tx(a)·Rz(θ+q)·Tz(d), so pre = Rx(α)·Tx(a).
            DhParam::MDH { alpha, a, .. } => na::Isometry::from_parts(
                na::Translation3::new(*a, 0.0, 0.0),
                na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), *alpha),
            ),
            // Iso3*: q is applied as Rz(q) at the end; pre = the constant SE(3) part.
            DhParam::Iso3Quat { pos, quat } => {
                let translation = na::Translation3::new(pos[0], pos[1], pos[2]);
                let rotation = na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                    quat[0], quat[1], quat[2], quat[3],
                ));
                na::Isometry::from_parts(translation, rotation)
            }
            DhParam::Iso3RPY { pos, rpy } => {
                let translation = na::Translation3::new(pos[0], pos[1], pos[2]);
                let rotation = na::UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]);
                na::Isometry::from_parts(translation, rotation)
            }
        }
    }
}
