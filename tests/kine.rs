#![feature(portable_simd)]
use nalgebra as na;
use robot_behavior::Pose;

#[inline(always)]
pub fn arm_forward_kinematics<const N: usize>(q: [f64; N], dh: [[f64; 4]; N]) -> Pose {
    let mut isometry = na::Isometry3::identity();

    for i in 0..N {
        let translation = na::Translation3::new(
            dh[i][2],
            -dh[i][1] * dh[i][3].sin(),
            dh[i][1] * dh[i][3].cos(),
        );
        let rotation = na::UnitQuaternion::from_euler_angles(q[i], 0.0, dh[i][3]);
        let isometry_increment = na::Isometry::from_parts(translation, rotation);

        isometry *= isometry_increment;
    }

    Pose::Quat(isometry)
}

pub fn arm_forward_kinematics_without_inline(n: usize, q: &Vec<f64>, dh: Vec<f64>) -> Pose {
    let mut isometry = na::Isometry3::identity();

    for i in 0..n {
        let translation = na::Translation3::new(
            dh[i * 4 + 2],
            -dh[i * 4 + 1] * dh[i * 4 + 3].sin(),
            dh[i * 4 + 1] * dh[i * 4 + 3].cos(),
        );
        let rotation = na::UnitQuaternion::from_euler_angles(q[i], 0.0, dh[i * 4 + 3]);
        let isometry_increment = na::Isometry::from_parts(translation, rotation);

        isometry *= isometry_increment;
    }
    Pose::Quat(isometry)
}

pub fn arm_forward_kinematics_use_matrix<const N: usize>(q: [f64; N], dh: [[f64; 4]; N]) -> Pose {
    let mut matrix = na::Matrix4::identity();

    for i in 0..N {
        let translation = na::Matrix4::new_translation(&na::Vector3::new(
            dh[i][2],
            -dh[i][1] * dh[i][3].sin(),
            dh[i][1] * dh[i][3].cos(),
        ));
        let rotation = na::Matrix4::from_euler_angles(q[i], 0.0, dh[i][3]);
        matrix *= translation * rotation;
    }
    Pose::Homo(matrix.as_slice().try_into().unwrap())
}

#[cfg(test)]
mod tests {
    use robot_behavior::ArmParam;

    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const DH: [[f64; 4]; 8] = [
        [0., 0.333, 0., 0.],
        [0., 0., 0., -FRAC_PI_2],
        [0., 0.316, 0., FRAC_PI_2],
        [0., 0., 0.0825, FRAC_PI_2],
        [0., 0.384, -0.0825, -FRAC_PI_2],
        [0., 0., 0., FRAC_PI_2],
        [0., 0., 0.088, FRAC_PI_2],
        [0., 0.107, 0., 0.],
    ];

    struct ExRobot {}

    impl ArmParam<8> for ExRobot {
        const DH: [[f64; 4]; 8] = DH;
        const JOINT_MAX: [f64; 8] = [0.; 8];
        const JOINT_MIN: [f64; 8] = [0.; 8];
    }

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
            let dh_vec: Vec<f64> = dh.iter().flat_map(|&row| row.to_vec()).collect();
            let _ = arm_forward_kinematics_without_inline(N, &q_vec, dh_vec);
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
            pose = arm_forward_kinematics::<8>(q, DH);
        }
        println!("arm_forward_kinematics: {:?}", start_time.elapsed());
        println!("pose: {:?}", pose);

        let mut q = [1.5; 8];
        let mut pose = Pose::default();
        let start_time = std::time::Instant::now();
        for _ in 0..100_000 {
            q[0] += 0.01;
            pose = ExRobot::forward_kinematics(&q);
        }
        println!("arm_forward_kinematics: {:?}", start_time.elapsed());
        println!("pose: {:?}", pose);
    }
}
