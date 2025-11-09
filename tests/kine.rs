#![feature(portable_simd)]
use nalgebra as na;
use robot_behavior::{DhParam, Pose};

#[inline(always)]
pub fn arm_forward_kinematics<const N: usize>(q: [f64; N], dh: &[DhParam; N]) -> Pose {
    let mut isometry = na::Isometry3::identity();

    for i in 0..N {
        let iso_inc = match dh[i] {
            DhParam::MDH { theta, d, a, alpha } => {
                let (s, c) = alpha.sin_cos();
                let translation = na::Translation3::new(a, -d * s, d * c);
                let rotation = na::UnitQuaternion::from_euler_angles(theta + q[i], 0.0, alpha);
                na::Isometry3::from_parts(translation, rotation)
            }
            DhParam::DH { theta, d, r, alpha } => {
                let (s, c) = alpha.sin_cos();
                let translation = na::Translation3::new(r * c, r * s, d);
                let rotation = na::UnitQuaternion::from_euler_angles(0.0, alpha, theta + q[i]);
                na::Isometry3::from_parts(translation, rotation)
            }
        };

        isometry *= iso_inc;
    }

    Pose::Quat(isometry)
}

pub fn arm_forward_kinematics_without_inline(n: usize, q: &Vec<f64>, dh: &[DhParam]) -> Pose {
    let mut isometry = na::Isometry3::identity();

    for i in 0..n {
        let iso_inc = match dh[i] {
            DhParam::MDH { theta, d, a, alpha } => {
                let (s, c) = alpha.sin_cos();
                let translation = na::Translation3::new(a, -d * s, d * c);
                let rotation = na::UnitQuaternion::from_euler_angles(theta + q[i], 0.0, alpha);
                na::Isometry3::from_parts(translation, rotation)
            }
            DhParam::DH { theta, d, r, alpha } => {
                let (s, c) = alpha.sin_cos();
                let translation = na::Translation3::new(r * c, r * s, d);
                let rotation = na::UnitQuaternion::from_euler_angles(0.0, alpha, theta + q[i]);
                na::Isometry3::from_parts(translation, rotation)
            }
        };

        isometry *= iso_inc;
    }
    Pose::Quat(isometry)
}

pub fn arm_forward_kinematics_use_matrix<const N: usize>(q: [f64; N], dh: &[DhParam; N]) -> Pose {
    let mut matrix = na::Matrix4::identity();

    for i in 0..N {
        let iso_inc = match dh[i] {
            DhParam::MDH { theta, d, a, alpha } => {
                let (s, c) = alpha.sin_cos();
                let translation = na::Translation3::new(a, -d * s, d * c);
                let rotation = na::UnitQuaternion::from_euler_angles(theta + q[i], 0.0, alpha);
                na::Isometry3::from_parts(translation, rotation)
            }
            DhParam::DH { theta, d, r, alpha } => {
                let (s, c) = alpha.sin_cos();
                let translation = na::Translation3::new(r * c, r * s, d);
                let rotation = na::UnitQuaternion::from_euler_angles(0.0, alpha, theta + q[i]);
                na::Isometry3::from_parts(translation, rotation)
            }
        };
        matrix *= iso_inc.to_homogeneous();
    }
    Pose::Homo(matrix.as_slice().try_into().unwrap())
}

#[cfg(test)]
mod tests {
    use robot_behavior::{DhParam, mdh_param};

    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const DH: [DhParam; 8] = [
        mdh_param!(0., 0.333, 0., 0.),
        mdh_param!(0., 0., 0., -FRAC_PI_2),
        mdh_param!(0., 0.316, 0., FRAC_PI_2),
        mdh_param!(0., 0., 0.0825, FRAC_PI_2),
        mdh_param!(0., 0.384, -0.0825, -FRAC_PI_2),
        mdh_param!(0., 0., 0., FRAC_PI_2),
        mdh_param!(0., 0., 0.088, FRAC_PI_2),
        mdh_param!(0., 0.107, 0., 0.),
    ];

    // Removed trait-based robot wrapper to avoid const-eval overflow; use free function instead.

    fn test_time_for_n<const N: usize, const T: usize>() {
        let dh = &DH[..N];
        let q = [1.5; N];

        println!("Testing with N = {}", N);

        let start_time = std::time::Instant::now();
        for _ in 0..T {
            let _ = arm_forward_kinematics::<N>(q, dh.try_into().unwrap());
        }
        println!("arm_forward_kinematics: {:?}", start_time.elapsed());

        let start_time = std::time::Instant::now();
        for _ in 0..T {
            let q_vec = q.to_vec();
            let _ = arm_forward_kinematics_without_inline(N, &q_vec, dh);
        }
        println!("arm_forward_kinematics: {:?}", start_time.elapsed());

        let start_time = std::time::Instant::now();
        for _ in 0..T {
            let _ = arm_forward_kinematics_use_matrix::<N>(q, dh.try_into().unwrap());
        }
        println!(
            "arm_forward_kinematics_use_matrix: {:?}",
            start_time.elapsed()
        );
    }

    #[test]
    fn test_arm_forward_kinematics() {
        test_time_for_n::<1, 100_000>();
        test_time_for_n::<2, 100_000>();
        test_time_for_n::<3, 100_000>();
        test_time_for_n::<4, 100_000>();
        test_time_for_n::<5, 100_000>();
        test_time_for_n::<6, 100_000>();
        test_time_for_n::<7, 100_000>();
        test_time_for_n::<8, 100_000>();

        let mut q = [1.5; 8];
        let mut pose = Pose::default();
        let start_time = std::time::Instant::now();
        for _ in 0..100_000 {
            q[0] += 0.01;
            pose = arm_forward_kinematics::<8>(q, &DH);
        }
        println!("arm_forward_kinematics: {:?}", start_time.elapsed());
        println!("pose: {:?}", pose);

        let mut q = [1.5; 8];
        let mut pose = Pose::default();
        let start_time = std::time::Instant::now();
        for _ in 0..100_000 {
            q[0] += 0.01;
            pose = arm_forward_kinematics::<8>(q, &DH);
        }
        println!("arm_forward_kinematics: {:?}", start_time.elapsed());
        println!("pose: {:?}", pose);
    }
}
