use nalgebra as na;
use serde_json::from_reader;
use std::fs::File;
use std::io::BufReader;

use crate::{ControlType, MotionType, RobotBehavior, RobotResult, realtime::RealtimeBehavior};

pub trait ArmBehavior<const N: usize>: RobotBehavior {
    fn move_to(&mut self, target: MotionType<N>) -> RobotResult<()>;
    fn move_to_async(&mut self, target: MotionType<N>) -> RobotResult<()>;
    fn move_rel(&mut self, rel: MotionType<N>) -> RobotResult<()>;
    fn move_rel_async(&mut self, rel: MotionType<N>) -> RobotResult<()>;
    fn move_path(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()>;
    fn control_with(&mut self, control: ControlType<N>) -> RobotResult<()>;
    fn read_state(&mut self, state_type: ArmStateType) -> RobotResult<ArmState<N>>;
}

pub trait ArmBehaviorExt<const N: usize>: ArmBehavior<N> {
    fn move_joint(&mut self, joint: [f64; N]) -> RobotResult<()> {
        self.move_to(MotionType::Joint(joint))
    }
    fn move_joint_async(&mut self, joint: [f64; N]) -> RobotResult<()> {
        self.move_to_async(MotionType::Joint(joint))
    }
    fn move_joint_rel(&mut self, joint: [f64; N]) -> RobotResult<()> {
        self.move_rel(MotionType::Joint(joint))
    }
    fn move_joint_rel_async(&mut self, joint: [f64; N]) -> RobotResult<()> {
        self.move_rel_async(MotionType::Joint(joint))
    }
    fn move_linear_with_quat(&mut self, pose: na::Isometry3<f64>) -> RobotResult<()> {
        self.move_to(MotionType::CartesianQuat(pose))
    }
    fn move_linear_with_quat_async(&mut self, pose: na::Isometry3<f64>) -> RobotResult<()> {
        self.move_to_async(MotionType::CartesianQuat(pose))
    }
    fn move_linear_with_euler(&mut self, pose: [f64; 6]) -> RobotResult<()> {
        self.move_to(MotionType::CartesianEuler(pose))
    }
    fn move_linear_with_euler_async(&mut self, pose: [f64; 6]) -> RobotResult<()> {
        self.move_to_async(MotionType::CartesianEuler(pose))
    }

    fn move_path_prepare(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()>;
    fn move_path_start(&mut self) -> RobotResult<()>;
    fn move_path_prepare_from_file(&mut self, path: &str) -> RobotResult<()>;
    fn move_path_from_file(&mut self, path: &str) -> RobotResult<()> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);
        let path: Vec<MotionType<N>> = from_reader(reader).unwrap();
        self.move_path(path)
    }
}

pub struct ArmRealtimeConfig {
    pub period: f64,
    pub timeout: f64,
    pub realtime_mode: bool,
}

// pub trait ArmState<const N: usize> {
//     fn joint(&self) -> [f64; N];
//     fn joint_vel(&self) -> [f64; N];
//     fn cartisian(&self) -> na::Isometry3<f64>;
//     fn cartisian_vel(&self) -> [f64; 6];
// }

pub enum ArmStateType {
    Both(Box<ArmStateType>, Box<ArmStateType>),
    Vec(Vec<ArmStateType>),

    Joint,
    JointVel,
    CartesianQuat,
    CartesianEuler,
    CartesianVel,
}

pub enum ArmState<const N: usize> {
    Both(Box<ArmState<N>>, Box<ArmState<N>>),
    Vec(Vec<ArmState<N>>),

    Joint([f64; N]),
    JointVel([f64; N]),
    CartesianQuat(na::Isometry3<f64>),
    CartesianEuler([f64; 6]),
    CartesianVel([f64; 6]),
}

pub trait ArmRealtimeHandle<const N: usize> {
    fn move_to(&mut self, target: MotionType<N>) -> RobotResult<()>;
    fn control_with(&mut self, control: ControlType<N>) -> RobotResult<()>;
}

pub trait ArmRealtimeBehavior<const N: usize, H>:
    ArmBehavior<N> + RealtimeBehavior<ArmRealtimeConfig, H>
where
    H: ArmRealtimeHandle<N>,
{
}
