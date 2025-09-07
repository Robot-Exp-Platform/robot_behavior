#[macro_export]
macro_rules! cxx_bridge {
    ($cxxname: ident<{$dof: expr}>($name: ty) -> $cxx_handle_name: ident($handle_name: ty)) => {
        #[cxx::bridge]
        mod to_cxx {
            pub struct CxxMotionType {
                pub mode: CxxMotionTypeMode,
                pub values: Vec<f64>,
            }
            pub enum CxxMotionTypeMode {
                Joint,
                JointVel,
                Cartesian,
                CartesianVel,
                Position,
                PositionVel,
                Stop,
            }
            pub struct CxxControlType {
                pub mode: CxxControlTypeMode,
                pub values: Vec<f64>,
            }
            pub enum CxxControlTypeMode {
                Zero,
                Torque,
            }
            pub struct CxxLoadState {
                pub m: f64,
                pub x: [f64; 3],
                pub i: [f64; 9],
            }
            extern "Rust" {
                type $cxxname;

                #[Self = "$cxxname"]
                fn attach() -> Box<$cxxname>;

                #[Self = "$cxxname"]
                fn version() -> String;
                fn init(&mut self) -> Result<()>;
                fn shutdown(&mut self) -> Result<()>;
                fn enable(&mut self) -> Result<()>;
                fn disable(&mut self) -> Result<()>;
                fn reset(&mut self) -> Result<()>;
                fn is_moving(&mut self) -> bool;
                fn stop(&mut self) -> Result<()>;
                fn pause(&mut self) -> Result<()>;
                fn resume(&mut self) -> Result<()>;
                fn emergency_stop(&mut self) -> Result<()>;
                fn clear_emergency_stop(&mut self) -> Result<()>;

                // fn state(&mut self)
                fn set_load(&mut self, load: CxxLoadState) -> Result<()>;
                // fn set_coord
                // fn with_coord(&mut self)
                fn set_speed(&mut self, speed: f64) -> Result<()>;
                fn with_speed(&mut self, speed: f64) -> &mut $cxxname;
                unsafe fn with_velocity<'a>(
                    &'a mut self,
                    joint_vel: &[f64; $dof],
                ) -> &'a mut $cxxname;
                unsafe fn with_acceleration<'a>(
                    &'a mut self,
                    joint_acc: &[f64; $dof],
                ) -> &'a mut $cxxname;
                unsafe fn with_jerk<'a>(&'a mut self, joint_jerk: &[f64; $dof])
                -> &'a mut $cxxname;
                fn with_cartesian_velocity(&mut self, cartesian_vel: f64) -> &mut $cxxname;
                fn with_cartesian_acceleration(&mut self, cartesian_acc: f64) -> &mut $cxxname;
                fn with_cartesian_jerk(&mut self, cartesian_jerk: f64) -> &mut $cxxname;
                fn with_rotation_velocity(&mut self, rotation_vel: f64) -> &mut $cxxname;
                fn with_rotation_acceleration(&mut self, rotation_acc: f64) -> &mut $cxxname;
                fn with_rotation_jerk(&mut self, rotation_jerk: f64) -> &mut $cxxname;

                fn move_joint(&mut self, target: &[f64; $dof]) -> Result<()>;
                fn move_joint_async(&mut self, target: &[f64; $dof]) -> Result<()>;
                fn move_cartesian(&mut self, target: &[f64]) -> Result<()>;
                fn move_cartesian_async(&mut self, target: &[f64]) -> Result<()>;

                fn move_to(&mut self, target: CxxMotionType) -> Result<()>;
                fn move_to_async(&mut self, target: CxxMotionType) -> Result<()>;
                fn move_rel(&mut self, target: CxxMotionType) -> Result<()>;
                fn move_rel_async(&mut self, target: CxxMotionType) -> Result<()>;
                fn move_int(&mut self, target: CxxMotionType) -> Result<()>;
                fn move_int_async(&mut self, target: CxxMotionType) -> Result<()>;
                // fn move_path(&mut self, path: Vec<CxxMotionType>) -> Result<()>;
                // fn move_path_async(&mut self, path: Vec<CxxMotionType>) -> Result<()>;
                // fn move_path_prepare(&mut self, path: Vec<CxxMotionType>) -> Result<()>;
                fn move_path_start(&mut self, start: CxxMotionType) -> Result<()>;

                fn move_joint_rel(&mut self, target: &[f64; $dof]) -> Result<()>;
                fn move_joint_rel_async(&mut self, target: &[f64; $dof]) -> Result<()>;
                // fn move_joint_path(&mut self, path: Vec<&[f64; $dof]>) -> Result<()>;
                // fn move_cartesian_rel(&mut self, target: &Pose) -> Result<()>;
                // fn move_cartesian_rel_async(&mut self, target: &Pose) -> Result<()>;
                // fn move_cartesian_int(&mut self, target: &Pose) -> Result<()>;
                // fn move_cartesian_int_async(&mut self, target: &Pose) -> Result<()>;
                // fn move_cartesian_path(&mut self, path: Vec<Pose>) -> Result<()>;/
                fn move_linear_with_euler(&mut self, pose: [f64; 6]) -> Result<()>;
                fn move_linear_with_euler_async(&mut self, pose: [f64; 6]) -> Result<()>;
                fn move_linear_with_euler_rel(&mut self, pose: [f64; 6]) -> Result<()>;
                fn move_linear_with_euler_rel_async(&mut self, pose: [f64; 6]) -> Result<()>;
                fn move_linear_with_euler_int(&mut self, pose: [f64; 6]) -> Result<()>;
                fn move_linear_with_euler_int_async(&mut self, pose: [f64; 6]) -> Result<()>;
                // fn move_linear_with_quat(&mut self, target: &na::Isometry3<f64>) -> Result<()>;
                // fn move_linear_with_quat_async(&mut self, target: &na::Isometry3<f64>) -> Result<()>;
                // fn move_linear_with_quat_rel(&mut self, target: &na::Isometry3<f64>) -> Result<()>;
                // fn move_linear_with_quat_rel_async(&mut self, target: &na::Isometry3<f64>) -> Result<()>;
                // fn move_linear_with_quat_int(&mut self, target: &na::Isometry3<f64>) -> Result<()>;
                // fn move_linear_with_quat_int_async(&mut self, target: &na::Isometry3<f64>) -> Result<()>;
                fn move_linear_with_homo(&mut self, target: [f64; 16]) -> Result<()>;
                fn move_linear_with_homo_async(&mut self, target: [f64; 16]) -> Result<()>;
                fn move_linear_with_homo_rel(&mut self, target: [f64; 16]) -> Result<()>;
                fn move_linear_with_homo_rel_async(&mut self, target: [f64; 16]) -> Result<()>;
                fn move_linear_with_homo_int(&mut self, target: [f64; 16]) -> Result<()>;
                fn move_linear_with_homo_int_async(&mut self, target: [f64; 16]) -> Result<()>;
                fn move_path_prepare_from_file(&mut self, path: &str) -> Result<()>;
                fn move_path_from_file(&mut self, path: &str) -> Result<()>;

                fn start_streaming(&mut self) -> Result<Box<$cxx_handle_name>>;
                fn end_streaming(&mut self) -> Result<()>;
                // fn move_to_target(&mut self) -> Arc<Mutex<Option<CxxMotionType>>>;
                // fn control_with_target(&mut self) -> Arc<Mutex<Option<ControlType<6>>>>;
            }
            extern "Rust" {
                type $cxx_handle_name;

                fn last_motion(&self) -> CxxMotionType;
                fn move_to(&mut self, target: CxxMotionType) -> Result<()>;
                fn last_control(&self) -> CxxControlType;
                fn control_with(&mut self, control: CxxControlType) -> Result<()>;
            }
        }

        pub use to_cxx::*;

        impl<const N: usize> From<CxxMotionType> for MotionType<N> {
            fn from(cxx: CxxMotionType) -> Self {
                match cxx.mode {
                    CxxMotionTypeMode::Joint => MotionType::Joint(cxx.values.try_into().unwrap()),
                    CxxMotionTypeMode::JointVel => {
                        MotionType::JointVel(cxx.values.try_into().unwrap())
                    }
                    CxxMotionTypeMode::Cartesian => {
                        MotionType::Cartesian(cxx.values.try_into().unwrap())
                    }
                    CxxMotionTypeMode::CartesianVel => {
                        MotionType::CartesianVel(cxx.values.try_into().unwrap())
                    }
                    CxxMotionTypeMode::Position => {
                        MotionType::Position(cxx.values.try_into().unwrap())
                    }
                    CxxMotionTypeMode::PositionVel => {
                        MotionType::PositionVel(cxx.values.try_into().unwrap())
                    }
                    CxxMotionTypeMode::Stop => MotionType::Stop,
                    _ => panic!("Invalid mode for MotionType"),
                }
            }
        }

        impl<const N: usize> From<MotionType<N>> for CxxMotionType {
            fn from(motion: MotionType<N>) -> Self {
                match motion {
                    MotionType::Joint(v) => CxxMotionType {
                        mode: CxxMotionTypeMode::Joint,
                        values: v.to_vec(),
                    },
                    MotionType::JointVel(v) => CxxMotionType {
                        mode: CxxMotionTypeMode::JointVel,
                        values: v.to_vec(),
                    },
                    MotionType::Cartesian(v) => CxxMotionType {
                        mode: CxxMotionTypeMode::Cartesian,
                        values: v.into(),
                    },
                    MotionType::CartesianVel(v) => CxxMotionType {
                        mode: CxxMotionTypeMode::CartesianVel,
                        values: v.to_vec(),
                    },
                    MotionType::Position(v) => CxxMotionType {
                        mode: CxxMotionTypeMode::Position,
                        values: v.to_vec(),
                    },
                    MotionType::PositionVel(v) => CxxMotionType {
                        mode: CxxMotionTypeMode::PositionVel,
                        values: v.to_vec(),
                    },
                    MotionType::Stop => CxxMotionType {
                        mode: CxxMotionTypeMode::Position, // Use Position as a placeholder for Stop
                        values: vec![],
                    },
                }
            }
        }

        impl<const N: usize> From<CxxControlType> for ControlType<N> {
            fn from(cxx: CxxControlType) -> Self {
                match cxx.mode {
                    CxxControlTypeMode::Zero => ControlType::Zero,
                    CxxControlTypeMode::Torque => {
                        ControlType::Torque(cxx.values.try_into().unwrap())
                    }
                    _ => panic!("Invalid mode for ControlType"),
                }
            }
        }

        impl<const N: usize> From<ControlType<N>> for CxxControlType {
            fn from(control: ControlType<N>) -> Self {
                match control {
                    ControlType::Zero => CxxControlType {
                        mode: CxxControlTypeMode::Zero,
                        values: vec![],
                    },
                    ControlType::Torque(v) => CxxControlType {
                        mode: CxxControlTypeMode::Torque,
                        values: v.to_vec(),
                    },
                }
            }
        }

        impl From<CxxLoadState> for LoadState {
            fn from(cxx: CxxLoadState) -> Self {
                LoadState {
                    m: cxx.m,
                    x: cxx.x,
                    i: cxx.i,
                }
            }
        }
    };
}
#[macro_export]
macro_rules! cxx_robot_behavior {
    ($cxxname: ident($name:ty)) => {
        fn version() -> String {
            <$name>::version()
        }
        fn init(&mut self) -> $crate::RobotResult<()> {
            self.0.init()
        }
        fn shutdown(&mut self) -> $crate::RobotResult<()> {
            self.0.shutdown()
        }
        fn enable(&mut self) -> $crate::RobotResult<()> {
            self.0.enable()
        }
        fn disable(&mut self) -> $crate::RobotResult<()> {
            self.0.disable()
        }
        fn reset(&mut self) -> $crate::RobotResult<()> {
            self.0.reset()
        }
        fn is_moving(&mut self) -> bool {
            self.0.is_moving()
        }
        fn stop(&mut self) -> $crate::RobotResult<()> {
            self.0.stop()
        }
        fn pause(&mut self) -> $crate::RobotResult<()> {
            self.0.pause()
        }
        fn resume(&mut self) -> $crate::RobotResult<()> {
            self.0.resume()
        }
        fn emergency_stop(&mut self) -> $crate::RobotResult<()> {
            self.0.emergency_stop()
        }
        fn clear_emergency_stop(&mut self) -> $crate::RobotResult<()> {
            self.0.clear_emergency_stop()
        }
    };
}
#[macro_export]
macro_rules! cxx_arm_behavior {
    ($cxxname: ident<{$dof: expr}>($name: ty)) => {
        fn state(&mut self) -> $crate::RobotResult<CxxArmState> {
            self.0.state().map(Into::into)
        }
        fn set_load(&mut self, load: CxxLoadState) -> $crate::RobotResult<()> {
            self.0.set_load(load.into())
        }
        fn set_coord(&mut self, coord: $crate::Coord) -> $crate::RobotResult<()> {
            self.0.set_coord(coord)
        }
        fn with_coord(&mut self, coord: $crate::Coord) -> &mut Self {
            self.0.with_coord(coord);
            self
        }
        fn set_speed(&mut self, speed: f64) -> $crate::RobotResult<()> {
            self.0.set_speed(speed)
        }
        fn with_speed(&mut self, speed: f64) -> &mut Self {
            self.0.with_speed(speed);
            self
        }
        fn with_velocity(&mut self, joint_vel: &[f64; $dof]) -> &mut Self {
            self.0.with_velocity(joint_vel);
            self
        }
        fn with_acceleration(&mut self, joint_acc: &[f64; $dof]) -> &mut Self {
            self.0.with_acceleration(joint_acc);
            self
        }
        fn with_jerk(&mut self, joint_jerk: &[f64; $dof]) -> &mut Self {
            self.0.with_jerk(joint_jerk);
            self
        }
        fn with_cartesian_velocity(&mut self, cartesian_vel: f64) -> &mut Self {
            self.0.with_cartesian_velocity(cartesian_vel);
            self
        }
        fn with_cartesian_acceleration(&mut self, cartesian_acc: f64) -> &mut Self {
            self.0.with_cartesian_acceleration(cartesian_acc);
            self
        }
        fn with_cartesian_jerk(&mut self, cartesian_jerk: f64) -> &mut Self {
            self.0.with_cartesian_jerk(cartesian_jerk);
            self
        }
        fn with_rotation_velocity(&mut self, rotation_vel: f64) -> &mut Self {
            self.0.with_rotation_velocity(rotation_vel);
            self
        }
        fn with_rotation_acceleration(&mut self, rotation_acc: f64) -> &mut Self {
            self.0.with_rotation_acceleration(rotation_acc);
            self
        }
        fn with_rotation_jerk(&mut self, rotation_jerk: f64) -> &mut Self {
            self.0.with_rotation_jerk(rotation_jerk);
            self
        }
    };
}
#[macro_export]
macro_rules! cxx_arm_param {
    ($cxxname: ident<{ $dof: expr }>($name: ty)) => {
        // 对于 ArmParam 的参数，这些是关联常量，不是方法
        // 这里只提供一些基础的参数访问接口
        // 具体的参数获取需要在实现中手动处理
    };
}
#[macro_export]
macro_rules! cxx_arm_preplanned_motion_impl {
    ($cxxname: ident<{ $dof: literal }>($name: ty)) => {
        fn move_joint(&mut self, target: &[f64; $dof]) -> $crate::RobotResult<()> {
            self.0.move_joint(target)
        }
        fn move_joint_async(&mut self, target: &[f64; $dof]) -> $crate::RobotResult<()> {
            self.0.move_joint_async(target)
        }
        fn move_cartesian(&mut self, target: &[f64]) -> $crate::RobotResult<()> {
            self.0.move_cartesian(&target.into())
        }
        fn move_cartesian_async(&mut self, target: &[f64]) -> $crate::RobotResult<()> {
            self.0.move_cartesian_async(&target.into())
        }
    };
}
#[macro_export]
macro_rules! cxx_arm_preplanned_motion {
    ($cxxname: ident<{ $dof: literal }>($name: ty)) => {
        fn move_to(&mut self, target: CxxMotionType) -> $crate::RobotResult<()> {
            self.0.move_to(target.into())
        }
        fn move_to_async(&mut self, target: CxxMotionType) -> $crate::RobotResult<()> {
            self.0.move_to_async(target.into())
        }
        fn move_rel(&mut self, target: CxxMotionType) -> $crate::RobotResult<()> {
            self.0.move_rel(target.into())
        }
        fn move_rel_async(&mut self, target: CxxMotionType) -> $crate::RobotResult<()> {
            self.0.move_rel_async(target.into())
        }
        fn move_int(&mut self, target: CxxMotionType) -> $crate::RobotResult<()> {
            self.0.move_int(target.into())
        }
        fn move_int_async(&mut self, target: CxxMotionType) -> $crate::RobotResult<()> {
            self.0.move_int_async(target.into())
        }
        fn move_path(&mut self, path: Vec<$crate::MotionType<$dof>>) -> $crate::RobotResult<()> {
            self.0.move_path(path)
        }
        fn move_path_async(
            &mut self,
            path: Vec<$crate::MotionType<$dof>>,
        ) -> $crate::RobotResult<()> {
            self.0.move_path_async(path)
        }
        fn move_path_prepare(
            &mut self,
            path: Vec<$crate::MotionType<$dof>>,
        ) -> $crate::RobotResult<()> {
            self.0.move_path_prepare(path)
        }
        fn move_path_start(&mut self, start: CxxMotionType) -> $crate::RobotResult<()> {
            self.0.move_path_start(start.into())
        }
    };
}
#[macro_export]
macro_rules! cxx_arm_preplanned_motion_ext {
    ($cxxname: ident<{ $dof: expr }>($name: ty)) => {
        fn move_joint_rel(&mut self, target: &[f64; $dof]) -> $crate::RobotResult<()> {
            self.0.move_joint_rel(target)
        }
        fn move_joint_rel_async(&mut self, target: &[f64; $dof]) -> $crate::RobotResult<()> {
            self.0.move_joint_rel_async(target)
        }
        fn move_joint_path(&mut self, path: Vec<[f64; $dof]>) -> $crate::RobotResult<()> {
            self.0.move_joint_path(path)
        }
        fn move_cartesian_rel(&mut self, target: &$crate::Pose) -> $crate::RobotResult<()> {
            self.0.move_cartesian_rel(target)
        }
        fn move_cartesian_rel_async(&mut self, target: &$crate::Pose) -> $crate::RobotResult<()> {
            self.0.move_cartesian_rel_async(target)
        }
        fn move_cartesian_int(&mut self, target: &$crate::Pose) -> $crate::RobotResult<()> {
            self.0.move_cartesian_int(target)
        }
        fn move_cartesian_int_async(&mut self, target: &$crate::Pose) -> $crate::RobotResult<()> {
            self.0.move_cartesian_int_async(target)
        }
        fn move_cartesian_path(&mut self, path: Vec<$crate::Pose>) -> $crate::RobotResult<()> {
            self.0.move_cartesian_path(path)
        }
        fn move_linear_with_euler(&mut self, pose: [f64; 6]) -> $crate::RobotResult<()> {
            self.0.move_linear_with_euler(pose)
        }
        fn move_linear_with_euler_async(&mut self, pose: [f64; 6]) -> $crate::RobotResult<()> {
            self.0.move_linear_with_euler_async(pose)
        }
        fn move_linear_with_euler_rel(&mut self, pose: [f64; 6]) -> $crate::RobotResult<()> {
            self.0.move_linear_with_euler_rel(pose)
        }
        fn move_linear_with_euler_rel_async(&mut self, pose: [f64; 6]) -> $crate::RobotResult<()> {
            self.0.move_linear_with_euler_rel_async(pose)
        }
        fn move_linear_with_euler_int(&mut self, pose: [f64; 6]) -> $crate::RobotResult<()> {
            self.0.move_linear_with_euler_int(pose)
        }
        fn move_linear_with_euler_int_async(&mut self, pose: [f64; 6]) -> $crate::RobotResult<()> {
            self.0.move_linear_with_euler_int_async(pose)
        }
        fn move_linear_with_quat(
            &mut self,
            target: &nalgebra::Isometry3<f64>,
        ) -> $crate::RobotResult<()> {
            self.0.move_linear_with_quat(target)
        }
        fn move_linear_with_quat_async(
            &mut self,
            target: &nalgebra::Isometry3<f64>,
        ) -> $crate::RobotResult<()> {
            self.0.move_linear_with_quat_async(target)
        }
        fn move_linear_with_quat_rel(
            &mut self,
            target: &nalgebra::Isometry3<f64>,
        ) -> $crate::RobotResult<()> {
            self.0.move_linear_with_quat_rel(target)
        }
        fn move_linear_with_quat_rel_async(
            &mut self,
            target: &nalgebra::Isometry3<f64>,
        ) -> $crate::RobotResult<()> {
            self.0.move_linear_with_quat_rel_async(target)
        }
        fn move_linear_with_quat_int(
            &mut self,
            target: &nalgebra::Isometry3<f64>,
        ) -> $crate::RobotResult<()> {
            self.0.move_linear_with_quat_int(target)
        }
        fn move_linear_with_quat_int_async(
            &mut self,
            target: &nalgebra::Isometry3<f64>,
        ) -> $crate::RobotResult<()> {
            self.0.move_linear_with_quat_int_async(target)
        }
        fn move_linear_with_homo(&mut self, target: [f64; 16]) -> $crate::RobotResult<()> {
            self.0.move_linear_with_homo(target)
        }
        fn move_linear_with_homo_async(&mut self, target: [f64; 16]) -> $crate::RobotResult<()> {
            self.0.move_linear_with_homo_async(target)
        }
        fn move_linear_with_homo_rel(&mut self, target: [f64; 16]) -> $crate::RobotResult<()> {
            self.0.move_linear_with_homo_rel(target)
        }
        fn move_linear_with_homo_rel_async(
            &mut self,
            target: [f64; 16],
        ) -> $crate::RobotResult<()> {
            self.0.move_linear_with_homo_rel_async(target)
        }
        fn move_linear_with_homo_int(&mut self, target: [f64; 16]) -> $crate::RobotResult<()> {
            self.0.move_linear_with_homo_int(target)
        }
        fn move_linear_with_homo_int_async(
            &mut self,
            target: [f64; 16],
        ) -> $crate::RobotResult<()> {
            self.0.move_linear_with_homo_int_async(target)
        }
        fn move_path_prepare_from_file(&mut self, path: &str) -> $crate::RobotResult<()> {
            self.0.move_path_prepare_from_file(path)
        }
        fn move_path_from_file(&mut self, path: &str) -> $crate::RobotResult<()> {
            self.0.move_path_from_file(path)
        }
    };
}
#[macro_export]
macro_rules! cxx_arm_streaming_handle {
    ($cxxname: ident<{ $dof: expr }>($name: ty)) => {
        fn last_motion(&self) -> CxxMotionType {
            self.0.last_motion().unwrap_or_default().into()
        }
        fn move_to(&mut self, target: CxxMotionType) -> $crate::RobotResult<()> {
            self.0.move_to(target.into())
        }
        fn last_control(&self) -> CxxControlType {
            self.0.last_control().unwrap_or_default().into()
        }
        fn control_with(&mut self, control: CxxControlType) -> $crate::RobotResult<()> {
            self.0.control_with(control.into())
        }
    };
}
#[macro_export]
macro_rules! cxx_arm_streaming_motion {
    ($cxxname: ident<{ $dof: expr }>($name: ty) -> $handle_name: ident) => {
        fn start_streaming(&mut self) -> $crate::RobotResult<Box<$handle_name>> {
            let handle = self.0.start_streaming()?;
            Ok(Box::new($handle_name(handle)))
        }
        fn end_streaming(&mut self) -> $crate::RobotResult<()> {
            self.0.end_streaming()
        }
        fn move_to_target(
            &mut self,
        ) -> std::sync::Arc<std::sync::Mutex<Option<$crate::MotionType<$dof>>>> {
            self.0.move_to_target()
        }
        fn control_with_target(
            &mut self,
        ) -> std::sync::Arc<std::sync::Mutex<Option<$crate::ControlType<$dof>>>> {
            self.0.control_with_target()
        }
    };
}
#[macro_export]
macro_rules! cxx_arm_streaming_motion_ext {
    ($cxxname: ident<{ $dof: expr }>($name: ty)) => {
        fn move_joint_target(&mut self) -> std::sync::Arc<std::sync::Mutex<Option<[f64; $dof]>>> {
            self.0.move_joint_target()
        }
        fn move_joint_vel_target(
            &mut self,
        ) -> std::sync::Arc<std::sync::Mutex<Option<[f64; $dof]>>> {
            self.0.move_joint_vel_target()
        }
        fn move_joint_acc_target(
            &mut self,
        ) -> std::sync::Arc<std::sync::Mutex<Option<[f64; $dof]>>> {
            self.0.move_joint_acc_target()
        }
        fn move_cartesian_target(
            &mut self,
        ) -> std::sync::Arc<std::sync::Mutex<Option<$crate::Pose>>> {
            self.0.move_cartesian_target()
        }
        fn move_cartesian_vel_target(
            &mut self,
        ) -> std::sync::Arc<std::sync::Mutex<Option<[f64; 6]>>> {
            self.0.move_cartesian_vel_target()
        }
        fn move_cartesian_euler_target(
            &mut self,
        ) -> std::sync::Arc<std::sync::Mutex<Option<[f64; 6]>>> {
            self.0.move_cartesian_euler_target()
        }
        fn move_cartesian_quat_target(
            &mut self,
        ) -> std::sync::Arc<std::sync::Mutex<Option<nalgebra::Isometry3<f64>>>> {
            self.0.move_cartesian_quat_target()
        }
        fn move_cartesian_homo_target(
            &mut self,
        ) -> std::sync::Arc<std::sync::Mutex<Option<[f64; 16]>>> {
            self.0.move_cartesian_homo_target()
        }
        fn control_tau_target(&mut self) -> std::sync::Arc<std::sync::Mutex<Option<[f64; $dof]>>> {
            self.0.control_tau_target()
        }
    };
}
#[macro_export]
macro_rules! cxx_arm_real_time_control {
    ($cxxname: ident<{ $dof: expr }>($name: ty)) => {};
}
#[macro_export]
macro_rules! cxx_arm_real_time_control_ext {
    ($cxxname: ident<{ $dof: expr }>($name: ty)) => {};
}
