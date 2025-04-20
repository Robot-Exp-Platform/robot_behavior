use nalgebra as na;
use serde::{Deserialize, Serialize};
use serde_with::serde_as;

#[derive(Debug, Serialize, Deserialize, PartialEq, Clone, Copy)]
pub enum Pose {
    Euler([f64; 6]),
    Quat(na::Isometry3<f64>),
    Homo([f64; 16]),
    Position([f64; 3]),
}

pub enum ControlType<const N: usize> {
    Zero,
    Torque([f64; N]),
}

#[serde_as]
#[derive(Serialize, Deserialize, Clone, Copy)]
pub enum MotionType<const N: usize> {
    Joint(#[serde_as(as = "[_; N]")] [f64; N]),
    JointVel(#[serde_as(as = "[_; N]")] [f64; N]),
    Cartesian(Pose),
    CartesianVel([f64; 6]),
    Position([f64; 3]),
    PositionVel([f64; 3]),
}

impl Pose {
    pub fn euler(&self) -> [f64; 6] {
        match self {
            Pose::Euler(pose) => *pose,
            _ => unimplemented!(),
        }
    }

    pub fn quat(&self) -> na::Isometry3<f64> {
        match self {
            Pose::Euler(pose) => na::Isometry3::<f64>::from_parts(
                na::Translation3::new(pose[0], pose[1], pose[2]),
                na::UnitQuaternion::from_euler_angles(pose[3], pose[4], pose[5]),
            ),
            Pose::Homo(pose) => array_to_isometry(pose),
            Pose::Quat(pose) => *pose,
            _ => unimplemented!(),
        }
    }

    pub fn homo(&self) -> [f64; 16] {
        match self {
            Pose::Homo(pose) => *pose,
            _ => unimplemented!(),
        }
    }
}

impl Default for Pose {
    fn default() -> Self {
        Pose::Quat(na::Isometry3::default())
    }
}

pub fn array_to_isometry(array: &[f64; 16]) -> na::Isometry3<f64> {
    let rot = na::Rotation3::from_matrix(
        &na::Matrix4::from_column_slice(array)
            .remove_column(3)
            .remove_row(3),
    );
    na::Isometry3::from_parts(
        na::Vector3::new(array[12], array[13], array[14]).into(),
        rot.into(),
    )
}
