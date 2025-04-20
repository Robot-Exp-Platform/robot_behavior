use nalgebra as na;
use serde_json::from_reader;
use std::fmt::Display;
use std::io::BufReader;
use std::sync::{Arc, Mutex};
use std::{fs::File, time::Duration};

use crate::{ControlType, LoadState, MotionType, Pose, RobotBehavior, RobotResult};

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
    pub pose_o_to_ee: Option<Pose>,
    pub pose_f_to_ee: Option<Pose>,
    pub pose_ee_to_k: Option<Pose>,
    pub cartesian_vel: Option<[f64; 6]>,
    pub load: Option<LoadState>,
}

pub trait ArmBehavior<const N: usize>: RobotBehavior {
    type State;
    fn read_state(&mut self) -> RobotResult<Self::State>;
    fn state(&mut self) -> RobotResult<ArmState<N>>;
    fn set_load(&mut self, load: LoadState) -> RobotResult<()>;
}

pub trait ArmPreplannedMotion<const N: usize>: ArmBehavior<N> {
    fn move_to(&mut self, target: MotionType<N>, speed: f64) -> RobotResult<()>;
    fn move_to_async(&mut self, target: MotionType<N>, speed: f64) -> RobotResult<()>;
    fn move_rel(&mut self, rel: MotionType<N>, speed: f64) -> RobotResult<()>;
    fn move_rel_async(&mut self, rel: MotionType<N>, speed: f64) -> RobotResult<()>;
    fn move_path(&mut self, path: Vec<MotionType<N>>, speed: f64) -> RobotResult<()>;
    fn control_with(&mut self, control: ControlType<N>) -> RobotResult<()>;
}

pub trait ArmPreplannedMotionExt<const N: usize>: ArmPreplannedMotion<N> {
    fn move_joint(&mut self, target: &[f64; N], speed: f64) -> RobotResult<()> {
        self.move_to(MotionType::Joint(*target), speed)
    }
    fn move_joint_async(&mut self, target: &[f64; N], speed: f64) -> RobotResult<()> {
        self.move_to_async(MotionType::Joint(*target), speed)
    }
    fn move_joint_rel(&mut self, target: &[f64; N], speed: f64) -> RobotResult<()> {
        self.move_rel(MotionType::Joint(*target), speed)
    }
    fn move_joint_rel_async(&mut self, target: &[f64; N], speed: f64) -> RobotResult<()> {
        self.move_rel_async(MotionType::Joint(*target), speed)
    }

    fn move_cartesian(&mut self, target: &Pose, speed: f64) -> RobotResult<()> {
        self.move_to(MotionType::Cartesian(*target), speed)
    }
    fn move_cartesian_async(&mut self, target: &Pose, speed: f64) -> RobotResult<()> {
        self.move_to_async(MotionType::Cartesian(*target), speed)
    }
    fn move_cartesian_rel(&mut self, target: &Pose, speed: f64) -> RobotResult<()> {
        self.move_rel(MotionType::Cartesian(*target), speed)
    }
    fn move_cartesian_rel_async(&mut self, target: &Pose, speed: f64) -> RobotResult<()> {
        self.move_rel_async(MotionType::Cartesian(*target), speed)
    }

    fn move_linear_with_quat(
        &mut self,
        target: &na::Isometry3<f64>,
        speed: f64,
    ) -> RobotResult<()> {
        self.move_cartesian(&Pose::Quat(*target), speed)
    }
    fn move_linear_with_quat_async(
        &mut self,
        target: &na::Isometry3<f64>,
        speed: f64,
    ) -> RobotResult<()> {
        self.move_cartesian_async(&Pose::Quat(*target), speed)
    }
    fn move_linear_with_euler(&mut self, target: &[f64; 6], speed: f64) -> RobotResult<()> {
        self.move_cartesian(&Pose::Euler(*target), speed)
    }
    fn move_linear_with_euler_async(&mut self, target: &[f64; 6], speed: f64) -> RobotResult<()> {
        self.move_cartesian_async(&Pose::Euler(*target), speed)
    }
    fn move_linear_with_homo(&mut self, target: &[f64; 16], speed: f64) -> RobotResult<()> {
        self.move_cartesian(&Pose::Homo(*target), speed)
    }
    fn move_linear_with_homo_async(&mut self, target: &[f64; 16], speed: f64) -> RobotResult<()> {
        self.move_cartesian_async(&Pose::Homo(*target), speed)
    }
    fn move_path_prepare(&mut self, _path: Vec<MotionType<N>>) -> RobotResult<()>;
    fn move_path_start(&mut self) -> RobotResult<()>;
    fn move_path_prepare_from_file(&mut self, path: &str) -> RobotResult<()> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);
        let path: Vec<MotionType<N>> = from_reader(reader).unwrap();
        self.move_path_prepare(path)
    }
    fn move_path_from_file(&mut self, path: &str, speed: f64) -> RobotResult<()> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);
        let path: Vec<MotionType<N>> = from_reader(reader).unwrap();
        self.move_path(path, speed)
    }
}

pub trait ArmStreamingHandle<const N: usize> {
    fn last_motion(&self) -> RobotResult<MotionType<N>>;
    fn move_to(&mut self, target: MotionType<N>) -> RobotResult<()>;

    fn last_control(&self) -> RobotResult<ControlType<N>>;
    fn control_with(&mut self, control: ControlType<N>) -> RobotResult<()>;
}

pub trait ArmStreamingMotion<const N: usize>: ArmBehavior<N> {
    type Handle: ArmStreamingHandle<N>;
    fn start_streaming(&mut self) -> RobotResult<Self::Handle>;
    fn end_streaming(&mut self) -> RobotResult<()>;

    fn move_to_target(&mut self) -> Arc<Mutex<Option<MotionType<N>>>>;
    fn control_to_target(&mut self) -> Arc<Mutex<Option<ControlType<N>>>>;
}

pub trait ArmStreamingMotionExt<const N: usize>: ArmStreamingMotion<N> {
    fn move_joint_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
    fn move_joint_vel_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
    fn move_joint_acc_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
    fn move_cartesian_target(&mut self) -> Arc<Mutex<Option<Pose>>>;
    fn move_cartesian_vel_target(&mut self) -> Arc<Mutex<Option<[f64; 6]>>>;
    fn move_cartesian_euler_target(&mut self) -> Arc<Mutex<Option<[f64; 6]>>>;
    fn move_cartesian_quat_target(&mut self) -> Arc<Mutex<Option<na::Isometry3<f64>>>>;
    fn move_cartesian_homo_target(&mut self) -> Arc<Mutex<Option<[f64; 16]>>>;
    fn control_tau_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
}

pub trait ArmRealtimeControl<const N: usize>: ArmBehavior<N> {
    fn move_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> MotionType<N> + Send + 'static;
    fn control_with_closure<FC>(&mut self, closure: FC) -> RobotResult<()>
    where
        FC: Fn(ArmState<N>, Duration) -> ControlType<N> + Send + 'static;
}

pub trait ArmRealtimeControlExt<const N: usize>: ArmRealtimeControl<N> {
    fn move_joint_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> [f64; N] + Send + Sync + 'static,
    {
        self.move_with_closure(move |state, duration| MotionType::Joint(closure(state, duration)))
    }

    fn move_joint_vel_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> [f64; N] + Send + Sync + 'static,
    {
        self.move_with_closure(move |state, duration| {
            MotionType::JointVel(closure(state, duration))
        })
    }

    fn move_cartesian_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> Pose + Send + Sync + 'static,
    {
        self.move_with_closure(move |state, duration| {
            MotionType::Cartesian(closure(state, duration))
        })
    }

    fn move_cartesian_vel_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> [f64; 6] + Send + Sync + 'static,
    {
        self.move_with_closure(move |state, duration| {
            MotionType::CartesianVel(closure(state, duration))
        })
    }

    fn move_cartesian_euler_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> [f64; 6] + Send + Sync + 'static,
    {
        self.move_cartesian_with_closure(move |state, duration| {
            Pose::Euler(closure(state, duration))
        })
    }

    fn move_cartesian_quat_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> na::Isometry3<f64> + Send + Sync + 'static,
    {
        self.move_cartesian_with_closure(move |state, duration| {
            Pose::Quat(closure(state, duration))
        })
    }

    fn move_cartesian_homo_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: Fn(ArmState<N>, Duration) -> [f64; 16] + Send + Sync + 'static,
    {
        self.move_cartesian_with_closure(move |state, duration| {
            Pose::Homo(closure(state, duration))
        })
    }
}

impl<const N: usize> Default for ArmState<N> {
    fn default() -> Self {
        ArmState {
            joint: Some([0.; N]),
            joint_vel: Some([0.; N]),
            joint_acc: Some([0.; N]),
            tau: Some([0.; N]),
            pose_o_to_ee: Some(Pose::default()),
            pose_f_to_ee: Some(Pose::default()),
            pose_ee_to_k: Some(Pose::default()),
            cartesian_vel: Some([0.; 6]),
            load: Some(LoadState {
                m: 0.,
                x: [0.; 3],
                i: [0.; 9],
            }),
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
        if let (MotionType::Cartesian(pose_target), Some(pose_state)) = (other, self.pose_o_to_ee) {
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
        if let (ControlType::Torque(torque_target), Some(force_state)) = (other, self.tau) {
            return force_state == *torque_target;
        }
        false
    }
}

impl<const N: usize> Display for ArmState<N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            r#"arm_state:
    | q: {:?},
    | dq: {:?},
    | ddq: {:?},
    | tau: {:?},
    | pose_o_to_ee: {:?},"#,
            self.joint, self.joint_vel, self.joint_acc, self.tau, self.pose_o_to_ee
        )
    }
}
