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
/// use robot_behavior::robot::DhParam;
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
    Isometry3(na::Isometry3<f64>),
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

#[roplat::interface]
impl DhParam {
    #[inline(always)]
    pub fn to_se3(&self, q: f64) -> na::Isometry3<f64> {
        match self {
            DhParam::DH { theta, d, r, alpha } => {
                let (s, c) = alpha.sin_cos();
                let translation = na::Translation3::new(r * c, r * s, *d);
                let rotation = na::UnitQuaternion::from_euler_angles(0.0, *alpha, theta + q);
                na::Isometry::from_parts(translation, rotation)
            }
            DhParam::MDH { theta, d, a, alpha } => {
                let (s, c) = alpha.sin_cos();
                let translation = na::Translation3::new(*a, -d * s, d * c);
                let rotation = na::UnitQuaternion::from_euler_angles(theta + q, 0.0, *alpha);
                na::Isometry::from_parts(translation, rotation)
            }
            DhParam::Isometry3(iso) => *iso,
        }
    }

    pub const fn default() -> Self {
        dh_param! {0.0, 0.0, 0.0, 0.0}
    }
}
