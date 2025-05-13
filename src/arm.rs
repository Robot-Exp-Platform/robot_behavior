use nalgebra as na;
use pipe_trait::*;
use serde_json::from_reader;
use std::fmt::Display;
use std::io::BufReader;
use std::sync::{Arc, Mutex};
use std::{fs::File, time::Duration};

use crate::utils::limit::*;
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
    fn state(&mut self) -> RobotResult<ArmState<N>>;
    fn set_load(&mut self, load: LoadState) -> RobotResult<()>;
}

pub trait ArmParam<const N: usize> {
    const DH: [[f64; 4]; N];
    const JOINT_DEFAULT: [f64; N] = [f64::MAX; N];
    const JOINT_MIN: [f64; N];
    const JOINT_MAX: [f64; N];
    const JOINT_VEL_BOUND: [f64; N] = [f64::MAX; N];
    const JOINT_ACC_BOUND: [f64; N] = [f64::MAX; N];
    const JOINT_JECK_BOUND: [f64; N] = [f64::MAX; N];
    const CARTESIAN_VEL_BOUND: f64 = f64::MAX;
    const CARTESIAN_ACC_BOUND: f64 = f64::MAX;
    const TORQUE_BOUND: [f64; N] = [f64::MAX; N];
    const TORQUE_DOT_BOUND: [f64; N] = [f64::MAX; N];

    #[inline(always)]
    fn forward_kinematics(q: &[f64; N]) -> Pose {
        let mut isometry = na::Isometry3::identity();
        for (dh, q) in Self::DH.iter().zip(q) {
            let (s, c) = dh[3].sin_cos();
            let translation = na::Translation3::new(dh[2], -dh[1] * s, dh[1] * c);
            let rotation = na::UnitQuaternion::from_euler_angles(*q, 0.0, dh[3]);
            let isometry_increment = na::Isometry::from_parts(translation, rotation);
            isometry *= isometry_increment;
        }
        Pose::Quat(isometry)
    }

    #[deprecated(
        note = "inverse kinematics is not implemented because the representation method of the robotic arm configuration has not been determined"
    )]
    #[inline(always)]
    fn inverse_kinematics(_pose: Pose) -> [f64; N] {
        unimplemented!(
            "inverse kinematics is not implemented because the representation method of the robotic arm configuration has not been determined"
        )
    }

    #[inline(always)]
    fn limit_joint_jeck(q_dddot: &mut [f64; N]) -> &mut [f64; N] {
        limit(q_dddot, &Self::JOINT_JECK_BOUND)
    }

    #[inline]
    fn limit_joint_acc<'a>(
        q_ddot: &'a mut [f64; N],
        q_ddot_last: &[f64; N],
        time: f64,
    ) -> &'a mut [f64; N] {
        q_ddot
            .pipe_mut(|q| limit_dot(q, q_ddot_last, time, &Self::JOINT_JECK_BOUND))
            .pipe_mut(|q| limit(q, &Self::JOINT_ACC_BOUND))
    }

    #[inline]
    fn limit_joint_vel<'a>(
        q_dot: &'a mut [f64; N],
        q_dot_last: &[f64; N],
        q_ddot_last: &[f64; N],
        time: f64,
    ) -> &'a mut [f64; N] {
        let mut q_ddot = difference(q_dot, q_dot_last, time);
        q_ddot.pipe_mut(|q| Self::limit_joint_acc(q, q_ddot_last, time));

        q_dot
            .pipe_mut(|q| update(q, &q_ddot, time))
            .pipe_mut(|q| limit(q, &Self::JOINT_VEL_BOUND))
    }

    #[inline]
    fn limit_joint<'a>(
        q: &'a mut [f64; N],
        q_last: &[f64; N],
        q_dot_last: &[f64; N],
        q_ddot_last: &[f64; N],
        time: f64,
    ) -> &'a mut [f64; N] {
        let mut q_dot = difference(q, q_last, time);
        q_dot.pipe_mut(|q| Self::limit_joint_vel(q, q_dot_last, q_ddot_last, time));

        q.pipe_mut(|q| update(q, &q_dot, time))
            .pipe_mut(|q| clamp(q, &Self::JOINT_MIN, &Self::JOINT_MAX))
    }
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
    fn move_joint_path(&mut self, path: Vec<[f64; N]>, speed: f64) -> RobotResult<()> {
        self.move_path(path.iter().map(|j| MotionType::Joint(*j)).collect(), speed)
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
    fn move_cartesian_path(&mut self, path: Vec<Pose>, speed: f64) -> RobotResult<()> {
        self.move_path(
            path.iter().map(|p| MotionType::Cartesian(*p)).collect(),
            speed,
        )
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
    fn move_linear_with_euler(
        &mut self,
        tran: [f64; 3],
        rot: [f64; 3],
        speed: f64,
    ) -> RobotResult<()> {
        self.move_cartesian(&Pose::Euler(tran, rot), speed)
    }
    fn move_linear_with_euler_async(
        &mut self,
        tran: [f64; 3],
        rot: [f64; 3],
        speed: f64,
    ) -> RobotResult<()> {
        self.move_cartesian_async(&Pose::Euler(tran, rot), speed)
    }
    fn move_linear_with_homo(&mut self, target: &[f64; 16], speed: f64) -> RobotResult<()> {
        self.move_cartesian(&Pose::Homo(*target), speed)
    }
    fn move_linear_with_homo_async(&mut self, target: &[f64; 16], speed: f64) -> RobotResult<()> {
        self.move_cartesian_async(&Pose::Homo(*target), speed)
    }

    fn move_path_prepare(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()>;
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
    fn control_with_target(&mut self) -> Arc<Mutex<Option<ControlType<N>>>>;
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
        FM: FnMut(ArmState<N>, Duration) -> (MotionType<N>, bool) + Send + 'static;
    fn control_with_closure<FC>(&mut self, closure: FC) -> RobotResult<()>
    where
        FC: FnMut(ArmState<N>, Duration) -> (ControlType<N>, bool) + Send + 'static;
}

pub trait ArmRealtimeControlExt<const N: usize>: ArmRealtimeControl<N> {
    fn move_joint_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, Duration) -> ([f64; N], bool) + Send + Sync + 'static,
    {
        self.move_with_closure(move |state, duration| {
            let (joint, finished) = closure(state, duration);
            (MotionType::Joint(joint), finished)
        })
    }

    fn move_joint_vel_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, Duration) -> ([f64; N], bool) + Send + Sync + 'static,
    {
        self.move_with_closure(move |state, duration| {
            let (joint_vel, finished) = closure(state, duration);
            (MotionType::JointVel(joint_vel), finished)
        })
    }

    fn move_cartesian_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, Duration) -> (Pose, bool) + Send + Sync + 'static,
    {
        self.move_with_closure(move |state, duration| {
            let (pose, finished) = closure(state, duration);
            (MotionType::Cartesian(pose), finished)
        })
    }

    fn move_cartesian_vel_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, Duration) -> ([f64; 6], bool) + Send + Sync + 'static,
    {
        self.move_with_closure(move |state, duration| {
            let (vel, finished) = closure(state, duration);
            (MotionType::CartesianVel(vel), finished)
        })
    }

    fn move_cartesian_euler_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, Duration) -> ([f64; 3], [f64; 3], bool) + Send + Sync + 'static,
    {
        self.move_cartesian_with_closure(move |state, duration| {
            let (tran, rot, finished) = closure(state, duration);
            (Pose::Euler(tran, rot), finished)
        })
    }

    fn move_cartesian_quat_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, Duration) -> (na::Isometry3<f64>, bool) + Send + Sync + 'static,
    {
        self.move_cartesian_with_closure(move |state, duration| {
            let (quat, finished) = closure(state, duration);
            (Pose::Quat(quat), finished)
        })
    }

    fn move_cartesian_homo_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, Duration) -> ([f64; 16], bool) + Send + Sync + 'static,
    {
        self.move_cartesian_with_closure(move |state, duration| {
            let (homo, finished) = closure(state, duration);
            (Pose::Homo(homo), finished)
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

#[cfg(feature = "to_py")]
mod to_py {
    use super::*;
    use pyo3::{pyclass, pymethods};
    #[pyclass(name = "ArmState")]
    pub struct PyArmState(ArmStateEnum);

    pub enum ArmStateEnum {
        ArmState0(ArmState<0>),
        ArmState1(ArmState<1>),
        ArmState2(ArmState<2>),
        ArmState3(ArmState<3>),
        ArmState4(ArmState<4>),
        ArmState5(ArmState<5>),
        ArmState6(ArmState<6>),
        ArmState7(ArmState<7>),
    }

    macro_rules! impl_from_arm_state {
        ($name: ident, $dof: expr) => {
            impl From<ArmState<$dof>> for PyArmState {
                fn from(state: ArmState<$dof>) -> Self {
                    PyArmState(ArmStateEnum::$name(state))
                }
            }
        };
    }

    impl_from_arm_state!(ArmState0, 0);
    impl_from_arm_state!(ArmState1, 1);
    impl_from_arm_state!(ArmState2, 2);
    impl_from_arm_state!(ArmState3, 3);
    impl_from_arm_state!(ArmState4, 4);
    impl_from_arm_state!(ArmState5, 5);
    impl_from_arm_state!(ArmState6, 6);
    impl_from_arm_state!(ArmState7, 7);

    #[pymethods]
    impl PyArmState {
        fn joint(&self) -> Option<Vec<f64>> {
            match &self.0 {
                ArmStateEnum::ArmState0(state) => state.joint.map(|j| j.to_vec()),
                ArmStateEnum::ArmState1(state) => state.joint.map(|j| j.to_vec()),
                ArmStateEnum::ArmState2(state) => state.joint.map(|j| j.to_vec()),
                ArmStateEnum::ArmState3(state) => state.joint.map(|j| j.to_vec()),
                ArmStateEnum::ArmState4(state) => state.joint.map(|j| j.to_vec()),
                ArmStateEnum::ArmState5(state) => state.joint.map(|j| j.to_vec()),
                ArmStateEnum::ArmState6(state) => state.joint.map(|j| j.to_vec()),
                ArmStateEnum::ArmState7(state) => state.joint.map(|j| j.to_vec()),
            }
        }

        fn joint_vel(&self) -> Option<Vec<f64>> {
            match &self.0 {
                ArmStateEnum::ArmState0(state) => state.joint_vel.map(|j| j.to_vec()),
                ArmStateEnum::ArmState1(state) => state.joint_vel.map(|j| j.to_vec()),
                ArmStateEnum::ArmState2(state) => state.joint_vel.map(|j| j.to_vec()),
                ArmStateEnum::ArmState3(state) => state.joint_vel.map(|j| j.to_vec()),
                ArmStateEnum::ArmState4(state) => state.joint_vel.map(|j| j.to_vec()),
                ArmStateEnum::ArmState5(state) => state.joint_vel.map(|j| j.to_vec()),
                ArmStateEnum::ArmState6(state) => state.joint_vel.map(|j| j.to_vec()),
                ArmStateEnum::ArmState7(state) => state.joint_vel.map(|j| j.to_vec()),
            }
        }

        fn joint_acc(&self) -> Option<Vec<f64>> {
            match &self.0 {
                ArmStateEnum::ArmState0(state) => state.joint_acc.map(|j| j.to_vec()),
                ArmStateEnum::ArmState1(state) => state.joint_acc.map(|j| j.to_vec()),
                ArmStateEnum::ArmState2(state) => state.joint_acc.map(|j| j.to_vec()),
                ArmStateEnum::ArmState3(state) => state.joint_acc.map(|j| j.to_vec()),
                ArmStateEnum::ArmState4(state) => state.joint_acc.map(|j| j.to_vec()),
                ArmStateEnum::ArmState5(state) => state.joint_acc.map(|j| j.to_vec()),
                ArmStateEnum::ArmState6(state) => state.joint_acc.map(|j| j.to_vec()),
                ArmStateEnum::ArmState7(state) => state.joint_acc.map(|j| j.to_vec()),
            }
        }

        fn tau(&self) -> Option<Vec<f64>> {
            match &self.0 {
                ArmStateEnum::ArmState0(state) => state.tau.map(|j| j.to_vec()),
                ArmStateEnum::ArmState1(state) => state.tau.map(|j| j.to_vec()),
                ArmStateEnum::ArmState2(state) => state.tau.map(|j| j.to_vec()),
                ArmStateEnum::ArmState3(state) => state.tau.map(|j| j.to_vec()),
                ArmStateEnum::ArmState4(state) => state.tau.map(|j| j.to_vec()),
                ArmStateEnum::ArmState5(state) => state.tau.map(|j| j.to_vec()),
                ArmStateEnum::ArmState6(state) => state.tau.map(|j| j.to_vec()),
                ArmStateEnum::ArmState7(state) => state.tau.map(|j| j.to_vec()),
            }
        }

        fn pose_o_to_ee(&self) -> Option<Pose> {
            match &self.0 {
                ArmStateEnum::ArmState0(state) => state.pose_o_to_ee,
                ArmStateEnum::ArmState1(state) => state.pose_o_to_ee,
                ArmStateEnum::ArmState2(state) => state.pose_o_to_ee,
                ArmStateEnum::ArmState3(state) => state.pose_o_to_ee,
                ArmStateEnum::ArmState4(state) => state.pose_o_to_ee,
                ArmStateEnum::ArmState5(state) => state.pose_o_to_ee,
                ArmStateEnum::ArmState6(state) => state.pose_o_to_ee,
                ArmStateEnum::ArmState7(state) => state.pose_o_to_ee,
            }
        }

        fn pose_f_to_ee(&self) -> Option<Pose> {
            match &self.0 {
                ArmStateEnum::ArmState0(state) => state.pose_f_to_ee,
                ArmStateEnum::ArmState1(state) => state.pose_f_to_ee,
                ArmStateEnum::ArmState2(state) => state.pose_f_to_ee,
                ArmStateEnum::ArmState3(state) => state.pose_f_to_ee,
                ArmStateEnum::ArmState4(state) => state.pose_f_to_ee,
                ArmStateEnum::ArmState5(state) => state.pose_f_to_ee,
                ArmStateEnum::ArmState6(state) => state.pose_f_to_ee,
                ArmStateEnum::ArmState7(state) => state.pose_f_to_ee,
            }
        }

        fn pose_ee_to_k(&self) -> Option<Pose> {
            match &self.0 {
                ArmStateEnum::ArmState0(state) => state.pose_ee_to_k,
                ArmStateEnum::ArmState1(state) => state.pose_ee_to_k,
                ArmStateEnum::ArmState2(state) => state.pose_ee_to_k,
                ArmStateEnum::ArmState3(state) => state.pose_ee_to_k,
                ArmStateEnum::ArmState4(state) => state.pose_ee_to_k,
                ArmStateEnum::ArmState5(state) => state.pose_ee_to_k,
                ArmStateEnum::ArmState6(state) => state.pose_ee_to_k,
                ArmStateEnum::ArmState7(state) => state.pose_ee_to_k,
            }
        }

        fn cartesian_vel(&self) -> Option<Vec<f64>> {
            match &self.0 {
                ArmStateEnum::ArmState0(state) => state.cartesian_vel.map(|j| j.to_vec()),
                ArmStateEnum::ArmState1(state) => state.cartesian_vel.map(|j| j.to_vec()),
                ArmStateEnum::ArmState2(state) => state.cartesian_vel.map(|j| j.to_vec()),
                ArmStateEnum::ArmState3(state) => state.cartesian_vel.map(|j| j.to_vec()),
                ArmStateEnum::ArmState4(state) => state.cartesian_vel.map(|j| j.to_vec()),
                ArmStateEnum::ArmState5(state) => state.cartesian_vel.map(|j| j.to_vec()),
                ArmStateEnum::ArmState6(state) => state.cartesian_vel.map(|j| j.to_vec()),
                ArmStateEnum::ArmState7(state) => state.cartesian_vel.map(|j| j.to_vec()),
            }
        }

        fn load(&self) -> Option<LoadState> {
            match &self.0 {
                ArmStateEnum::ArmState0(state) => state.load.clone(),
                ArmStateEnum::ArmState1(state) => state.load.clone(),
                ArmStateEnum::ArmState2(state) => state.load.clone(),
                ArmStateEnum::ArmState3(state) => state.load.clone(),
                ArmStateEnum::ArmState4(state) => state.load.clone(),
                ArmStateEnum::ArmState5(state) => state.load.clone(),
                ArmStateEnum::ArmState6(state) => state.load.clone(),
                ArmStateEnum::ArmState7(state) => state.load.clone(),
            }
        }

        fn __repr__(&self) -> String {
            match &self.0 {
                ArmStateEnum::ArmState0(state) => format!("{:?}", state),
                ArmStateEnum::ArmState1(state) => format!("{:?}", state),
                ArmStateEnum::ArmState2(state) => format!("{:?}", state),
                ArmStateEnum::ArmState3(state) => format!("{:?}", state),
                ArmStateEnum::ArmState4(state) => format!("{:?}", state),
                ArmStateEnum::ArmState5(state) => format!("{:?}", state),
                ArmStateEnum::ArmState6(state) => format!("{:?}", state),
                ArmStateEnum::ArmState7(state) => format!("{:?}", state),
            }
        }
    }
}

#[cfg(feature = "to_py")]
pub use to_py::*;
