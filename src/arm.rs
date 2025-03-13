use nalgebra as na;
use serde_json::from_reader;
use std::io::BufReader;
use std::sync::{Arc, Mutex};
use std::{fs::File, time::Duration};

use crate::{ControlType, MotionType, RobotBehavior, RobotResult};

pub trait ArmBehavior<const N: usize>: RobotBehavior {
    fn move_to(&mut self, target: MotionType<N>, speed: f64) -> RobotResult<()>;
    fn move_to_async(&mut self, target: MotionType<N>, speed: f64) -> RobotResult<()>;
    fn move_rel(&mut self, rel: MotionType<N>) -> RobotResult<()>;
    fn move_rel_async(&mut self, rel: MotionType<N>) -> RobotResult<()>;
    fn move_path(&mut self, path: Vec<MotionType<N>>, speed: f64) -> RobotResult<()>;
    fn control_with(&mut self, control: ControlType<N>) -> RobotResult<()>;
    fn read_state(&mut self) -> RobotResult<ArmState<N>>;
}

pub trait ArmBehaviorExt<const N: usize>: ArmBehavior<N> {
    fn move_joint(&mut self, target: &[f64; N], speed: f64) -> RobotResult<()> {
        self.move_to(MotionType::Joint(*target), speed)
    }
    fn move_joint_async(&mut self, target: &[f64; N], speed: f64) -> RobotResult<()> {
        self.move_to_async(MotionType::Joint(*target), speed)
    }
    fn move_joint_rel(&mut self, target: &[f64; N]) -> RobotResult<()> {
        self.move_rel(MotionType::Joint(*target))
    }
    fn move_joint_rel_async(&mut self, target: &[f64; N]) -> RobotResult<()> {
        self.move_rel_async(MotionType::Joint(*target))
    }
    fn move_linear_with_quat(
        &mut self,
        target: &na::Isometry3<f64>,
        speed: f64,
    ) -> RobotResult<()> {
        self.move_to(MotionType::CartesianQuat(*target), speed)
    }
    fn move_linear_with_quat_async(
        &mut self,
        target: &na::Isometry3<f64>,
        speed: f64,
    ) -> RobotResult<()> {
        self.move_to_async(MotionType::CartesianQuat(*target), speed)
    }
    fn move_linear_with_euler(&mut self, target: &[f64; 6], speed: f64) -> RobotResult<()> {
        self.move_to(MotionType::CartesianEuler(*target), speed)
    }
    fn move_linear_with_euler_async(&mut self, target: &[f64; 6], speed: f64) -> RobotResult<()> {
        self.move_to_async(MotionType::CartesianEuler(*target), speed)
    }

    fn move_path_prepare(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()>;
    fn move_path_start(&mut self) -> RobotResult<()>;
    fn move_path_prepare_from_file(&mut self, path: &str) -> RobotResult<()>;
    fn move_path_from_file(&mut self, path: &str, speed: f64) -> RobotResult<()> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);
        let path: Vec<MotionType<N>> = from_reader(reader).unwrap();
        self.move_path(path, speed)
    }
}

pub struct ArmRealtimeConfig {
    pub period: f64,
    pub timeout: f64,
    pub realtime_mode: bool,
}

#[derive(Debug, Clone)]
pub struct ArmState<const N: usize> {
    pub joint: Option<[f64; N]>,
    pub joint_vel: Option<[f64; N]>,
    pub joint_acc: Option<[f64; N]>,
    pub tau: Option<[f64; N]>,
    pub cartesian_euler: Option<[f64; 6]>,
    pub cartesian_quat: Option<na::Isometry3<f64>>,
    pub cartesian_homo: Option<[f64; 16]>,
    pub cartesian_vel: Option<[f64; 6]>,
}

pub trait ArmRealtimeHandle<const N: usize> {
    fn move_to(&mut self, target: MotionType<N>) -> RobotResult<()>;
    fn control_with(&mut self, control: ControlType<N>) -> RobotResult<()>;
}

pub trait ArmRealtimeBehavior<const N: usize>: ArmBehavior<N> {
    fn move_with_closure<FM: Fn(ArmState<N>, Duration) -> MotionType<N> + Send + 'static>(
        &mut self,
        closure: FM,
    ) -> RobotResult<()>;
    fn control_with_closure<FC: Fn(ArmState<N>, Duration) -> ControlType<N> + Send + 'static>(
        &mut self,
        closure: FC,
    ) -> RobotResult<()>;
    fn move_to_target(&mut self) -> Arc<Mutex<Option<MotionType<N>>>>;
    fn control_to_target(&mut self) -> Arc<Mutex<Option<ControlType<N>>>>;
}

pub trait ArmRealtimeBehaviorExt<const N: usize> {
    fn move_joint_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
    fn move_joint_vel_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
    fn move_joint_acc_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
    fn move_cartesian_euler_target(&mut self) -> Arc<Mutex<Option<[f64; 6]>>>;
    fn move_cartesian_quat_target(&mut self) -> Arc<Mutex<Option<na::Isometry3<f64>>>>;
    fn move_cartesian_homo_target(&mut self) -> Arc<Mutex<Option<[f64; 16]>>>;
    fn move_cartesian_vel_target(&mut self) -> Arc<Mutex<Option<[f64; 6]>>>;
    fn control_tau_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
}

impl<const N: usize> Default for ArmState<N> {
    fn default() -> Self {
        ArmState {
            joint: Some([0.; N]),
            joint_vel: Some([0.; N]),
            joint_acc: Some([0.; N]),
            tau: Some([0.; N]),
            cartesian_euler: Some([0.; 6]),
            cartesian_quat: Some(na::Isometry3::default()),
            cartesian_homo: Some([0.; 16]),
            cartesian_vel: Some([0.; 6]),
        }
    }
}

impl<const N: usize> PartialEq<MotionType<N>> for ArmState<N> {
    fn eq(&self, other: &MotionType<N>) -> bool {
        if let (MotionType::Joint(joint_target), Some(joint_state)) = (other, self.joint) {
            return joint_state == *joint_target;
        }
        if let (MotionType::JointVel(vel_target), Some(vel_state)) = (other, self.joint_vel) {
            return vel_state == *vel_target;
        }
        if let (MotionType::CartesianEuler(pose_target), Some(pose_state)) =
            (other, self.cartesian_euler)
        {
            return pose_state == *pose_target;
        }
        if let (MotionType::CartesianQuat(pose_target), Some(pose_state)) =
            (other, self.cartesian_quat)
        {
            return pose_state == *pose_target;
        }
        if let (MotionType::CartesianHomo(pose_target), Some(pose_state)) =
            (other, self.cartesian_homo)
        {
            return pose_state == *pose_target;
        }
        if let (MotionType::CartesianVel(vel_target), Some(vel_state)) = (other, self.cartesian_vel)
        {
            return vel_state == *vel_target;
        }

        false
    }
}

impl<const N: usize> PartialEq<ControlType<N>> for ArmState<N> {
    fn eq(&self, other: &ControlType<N>) -> bool {
        if let (ControlType::Force(force_target), Some(force_state)) = (other, self.tau) {
            return force_state == *force_target;
        }

        false
    }
}
