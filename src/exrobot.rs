use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use crate::{ArmState, ControlType, Coord, LoadState, MotionType, Pose, RobotResult, behavior::*};

pub struct ExRobot<const N: usize>;
#[derive(Default)]
pub struct ExStreamHandle<const N: usize>;

impl<const N: usize> ExRobot<N> {
    pub fn new() -> Self {
        ExRobot
    }
}

impl<const N: usize> RobotBehavior for ExRobot<N> {
    type State = String;

    fn version(&self) -> String {
        format!("ExRobot<{N}> Version: {}", env!("CARGO_PKG_VERSION"))
    }

    fn init(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> init");
        Ok(())
    }

    fn shutdown(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> shutdown");
        Ok(())
    }

    fn enable(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> enable");
        Ok(())
    }

    fn disable(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> disable");
        Ok(())
    }

    fn reset(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> reset");
        Ok(())
    }

    fn is_moving(&mut self) -> bool {
        println!("ExRobot<{N}> is_moving");
        false
    }

    fn stop(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> stop");
        Ok(())
    }

    fn pause(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> pause");
        Ok(())
    }

    fn resume(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> resume");
        Ok(())
    }

    fn emergency_stop(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> emergency_stop");
        Ok(())
    }

    fn clear_emergency_stop(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> clear_emergency_stop");
        Ok(())
    }

    fn read_state(&mut self) -> RobotResult<Self::State> {
        println!("ExRobot<{N}> read_state");
        Ok(format!("ExRobot<{N}> State"))
    }
}

impl<const N: usize> ArmBehavior<N> for ExRobot<N> {
    fn state(&mut self) -> RobotResult<ArmState<N>> {
        println!("ExRobot<{N}> state");
        Ok(ArmState::default())
    }

    fn set_load(&mut self, load: LoadState) -> RobotResult<()> {
        println!("ExRobot<{N}> set_load: {load:?}");
        Ok(())
    }
    fn set_coord(&mut self, coord: Coord) -> RobotResult<()> {
        println!("ExRobot<{N}> set_coord: {coord:?}");
        Ok(())
    }
    fn with_coord(&mut self, coord: Coord) -> &mut Self {
        println!("ExRobot<{N}> with_coord: {coord:?}");
        self
    }

    fn set_speed(&mut self, speed: f64) -> RobotResult<()> {
        println!("ExRobot<{N}> set_speed: {speed}");
        Ok(())
    }
    fn with_speed(&mut self, speed: f64) -> &mut Self {
        println!("ExRobot<{N}> with_speed: {speed}");
        self
    }

    fn with_velocity(&mut self, joint_vel: &[f64; N]) -> &mut Self {
        println!("ExRobot<{N}> with_velocity: {joint_vel:?}");
        self
    }
    fn with_acceleration(&mut self, joint_acc: &[f64; N]) -> &mut Self {
        println!("ExRobot<{N}> with_acceleration: {joint_acc:?}");
        self
    }
    fn with_jerk(&mut self, joint_jerk: &[f64; N]) -> &mut Self {
        println!("ExRobot<{N}> with_jerk: {joint_jerk:?}");
        self
    }
    fn with_cartesian_velocity(&mut self, cartesian_vel: f64) -> &mut Self {
        println!("ExRobot<{N}> with_cartesian_velocity: {cartesian_vel}");
        self
    }
    fn with_cartesian_acceleration(&mut self, cartesian_acc: f64) -> &mut Self {
        println!("ExRobot<{N}> with_cartesian_acceleration: {cartesian_acc}");
        self
    }
    fn with_cartesian_jerk(&mut self, cartesian_jerk: f64) -> &mut Self {
        println!("ExRobot<{N}> with_cartesian_jerk: {cartesian_jerk}");
        self
    }
    fn with_rotation_velocity(&mut self, rotation_vel: f64) -> &mut Self {
        println!("ExRobot<{N}> with_rotation_velocity: {rotation_vel}");
        self
    }
    fn with_rotation_acceleration(&mut self, rotation_acc: f64) -> &mut Self {
        println!("ExRobot<{N}> with_rotation_acceleration: {rotation_acc}");
        self
    }
    fn with_rotation_jerk(&mut self, rotation_jerk: f64) -> &mut Self {
        println!("ExRobot<{N}> with_rotation_jerk: {rotation_jerk}");
        self
    }
}

impl<const N: usize> ArmParam<N> for ExRobot<N> {
    const DH: [[f64; 4]; N] = [[0.0; 4]; N];
    const JOINT_MIN: [f64; N] = [0.0; N];
    const JOINT_MAX: [f64; N] = [1.0; N];
}

impl<const N: usize> ArmPreplannedMotionImpl<N> for ExRobot<N> {
    fn move_joint(&mut self, target: &[f64; N]) -> RobotResult<()> {
        println!("ExRobot<{N}> move_joint: {target:?}");
        self.move_joint_async(target)
    }

    fn move_joint_async(&mut self, target: &[f64; N]) -> RobotResult<()> {
        println!("ExRobot<{N}> move_joint_async: {target:?}");
        Ok(())
    }

    fn move_cartesian(&mut self, target: &Pose) -> RobotResult<()> {
        println!("ExRobot<{N}> move_cartesian: {target:?}");
        self.move_cartesian_async(target)
    }

    fn move_cartesian_async(&mut self, target: &Pose) -> RobotResult<()> {
        println!("ExRobot<{N}> move_cartesian_async: {target:?}");
        Ok(())
    }
}

impl<const N: usize> ArmPreplannedMotion<N> for ExRobot<N> {
    fn move_path(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()> {
        println!("ExRobot<{N}> move_path: {path:?}");
        self.move_path_async(path)
    }
    fn move_path_async(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()> {
        println!("ExRobot<{N}> move_path_async: {path:?}");
        Ok(())
    }
    fn move_path_start(&mut self, start: MotionType<N>) -> RobotResult<()> {
        println!("ExRobot<{N}> move_path_start: {start:?}");
        Ok(())
    }
    fn move_path_prepare(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()> {
        println!("ExRobot<{N}> move_path_prepare: {path:?}");
        Ok(())
    }
}

impl<const N: usize> ArmStreamingHandle<N> for ExStreamHandle<N> {
    fn last_motion(&self) -> RobotResult<MotionType<N>> {
        println!("ExStreamHandle<{N}> last_motion");
        Ok(MotionType::Joint([0.0; N]))
    }
    fn move_to(&mut self, target: MotionType<N>) -> RobotResult<()> {
        println!("ExStreamHandle<{N}> move_to: {target:?}");
        Ok(())
    }
    fn last_control(&self) -> RobotResult<ControlType<N>> {
        println!("ExStreamHandle<{N}> last_control");
        Ok(ControlType::Torque([0.0; N]))
    }
    fn control_with(&mut self, control: ControlType<N>) -> RobotResult<()> {
        println!("ExStreamHandle<{N}> control_with: {control:?}");
        Ok(())
    }
}

impl<const N: usize> ArmStreamingMotion<N> for ExRobot<N> {
    type Handle = ExStreamHandle<N>;
    fn start_streaming(&mut self) -> RobotResult<Self::Handle> {
        println!("ExRobot<{N}> start_streaming");
        Ok(ExStreamHandle::<N>)
    }
    fn end_streaming(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> end_streaming");
        Ok(())
    }

    fn move_to_target(&mut self) -> Arc<Mutex<Option<MotionType<N>>>> {
        println!("ExRobot<{N}> move_to_target");
        Arc::new(Mutex::new(Some(MotionType::Joint([0.0; N]))))
    }
    fn control_with_target(&mut self) -> Arc<Mutex<Option<ControlType<N>>>> {
        println!("ExRobot<{N}> control_with_target");
        Arc::new(Mutex::new(Some(ControlType::Torque([0.0; N]))))
    }
}

impl<const N: usize> ArmStreamingMotionExt<N> for ExRobot<N> {
    fn move_joint_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>> {
        println!("ExRobot<{N}> move_joint_target");
        Arc::new(Mutex::new(Some([0.0; N])))
    }
    fn move_joint_vel_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>> {
        println!("ExRobot<{N}> move_joint_vel_target");
        Arc::new(Mutex::new(Some([0.0; N])))
    }
    fn move_joint_acc_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>> {
        println!("ExRobot<{N}> move_joint_acc_target");
        Arc::new(Mutex::new(Some([0.0; N])))
    }
    fn move_cartesian_target(&mut self) -> Arc<Mutex<Option<Pose>>> {
        println!("ExRobot<{N}> move_cartesian_target");
        Arc::new(Mutex::new(Some(Pose::default())))
    }
    fn move_cartesian_vel_target(&mut self) -> Arc<Mutex<Option<[f64; 6]>>> {
        println!("ExRobot<{N}> move_cartesian_vel_target");
        Arc::new(Mutex::new(Some([0.0; 6])))
    }
    fn move_cartesian_quat_target(&mut self) -> Arc<Mutex<Option<nalgebra::Isometry3<f64>>>> {
        println!("ExRobot<{N}> move_cartesian_quat_target");
        Arc::new(Mutex::new(Some(nalgebra::Isometry3::identity())))
    }
    fn move_cartesian_homo_target(&mut self) -> Arc<Mutex<Option<[f64; 16]>>> {
        println!("ExRobot<{N}> move_cartesian_homo_target");
        Arc::new(Mutex::new(Some([0.0; 16])))
    }
    fn move_cartesian_euler_target(&mut self) -> Arc<Mutex<Option<[f64; 6]>>> {
        println!("ExRobot<{N}> move_cartesian_euler_target");
        Arc::new(Mutex::new(Some([0.0; 6])))
    }
    fn control_tau_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>> {
        println!("ExRobot<{N}> control_tau_target");
        Arc::new(Mutex::new(Some([0.0; N])))
    }
}

impl<const N: usize> ArmRealtimeControl<N> for ExRobot<N> {
    fn move_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, std::time::Duration) -> (MotionType<N>, bool) + Send + 'static,
    {
        println!(
            "closure use for default state = {:?}",
            closure(ArmState::default(), Duration::from_secs(0))
        );
        println!("ExRobot<{N}> move_with_closure");
        Ok(())
    }
    fn control_with_closure<FC>(&mut self, mut closure: FC) -> RobotResult<()>
    where
        FC: FnMut(ArmState<N>, std::time::Duration) -> (ControlType<N>, bool) + Send + 'static,
    {
        println!(
            "closure use for default state = {:?}",
            closure(ArmState::default(), Duration::from_secs(0))
        );
        println!("ExRobot<{N}> control_with_closure");
        Ok(())
    }
}

impl<const N: usize> ArmRealtimeControlExt<N> for ExRobot<N> {}
