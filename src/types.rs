use nalgebra as na;
use serde::{Deserialize, Serialize};
use serde_with::serde_as;

#[derive(Debug, Clone)]
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
    CartesianQuat(na::Isometry3<f64>),
    CartesianEuler([f64; 6]),
    CartesianHomo([f64; 16]),
    CartesianVel([f64; 6]),
    Position([f64; 3]),
    PositionVel([f64; 3]),
}

impl Default for Pose {
    fn default() -> Self {
        Pose::Quat(na::Isometry3::default())
    }
}
