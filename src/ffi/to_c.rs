use std::ffi::{CString, c_char};

use crate::{ControlType, MotionType, RobotResult, behavior::*};

#[repr(C)]
pub struct CError {
    code: i32,
    message: CString,
}

impl<T> From<RobotResult<T>> for CError {
    fn from(result: RobotResult<T>) -> Self {
        match result {
            Ok(_) => CError {
                code: 0,
                message: CString::new("".to_string()).unwrap(),
            },
            Err(e) => {
                let message = CString::new(e.to_string()).unwrap();
                let mut c_message = [0; 256];
                message
                    .as_bytes()
                    .iter()
                    .enumerate()
                    .for_each(|(i, &c)| c_message[i] = c as c_char);
                CError { code: 1, message }
            }
        }
    }
}

#[repr(C)]
pub enum CMotionType {
    Joint,
    JointVel,
    Cartesian,
    CartesianVel,
    Position,
    PositionVel,
}

impl<const N: usize> Into<MotionType<N>> for (CMotionType, [f64; N]) {
    fn into(self) -> MotionType<N> {
        match self.0 {
            CMotionType::Joint => MotionType::Joint(self.1),
            CMotionType::JointVel => MotionType::JointVel(self.1),
            CMotionType::Cartesian => MotionType::Cartesian(self.1[0..6].try_into().unwrap()),
            CMotionType::CartesianVel => MotionType::CartesianVel(self.1[0..6].try_into().unwrap()),
            CMotionType::Position => MotionType::Position(self.1[0..3].try_into().unwrap()),
            CMotionType::PositionVel => MotionType::PositionVel(self.1[0..3].try_into().unwrap()),
        }
    }
}
#[repr(C)]
pub enum CControlType {
    Zero,
    Torque,
}

impl<const N: usize> Into<ControlType<N>> for (CControlType, [f64; N]) {
    fn into(self) -> ControlType<N> {
        match self.0 {
            CControlType::Zero => ControlType::Zero,
            CControlType::Torque => ControlType::Torque(self.1),
        }
    }
}

#[cxx::bridge]
pub mod ffi {
    extern "Rust" {
        type Pose;
    }
}

#[macro_export]
macro_rules! impl_self {
    ($(fn $name: ident() -> $ret: ty;)*) => {
        $(
            pub fn $name() -> $ret {
                self.0.$name()
            }
        )*
    };
    ($(fn $name: ident(&mut self) -> $ret: ty;)*) => {
        $(
            pub fn $name(&mut self) -> $ret {
                self.0.$name()
            }
        )*
    };
    ($(fn $name: ident(&self) -> $ret: ty;)*) => {
        $(
            pub fn $name(&self) -> $ret {
                self.0.$name()
            }
        )*
    };
    ($(fn $name: ident(&mut self, $($arg: ident: $arg_ty: ty),*) -> $ret: ty;)*) => {
        $(
            pub fn $name(&mut self, $($arg: $arg_ty),*) -> $ret {
                self.0.$name($($arg),*)
            }
        )*
    };
    ($(fn $name: ident(&self, $($arg: ident: $arg_ty: ty),*) -> $ret: ty;)*) => {
        $(
            pub fn $name(&self, $($arg: $arg_ty),*) -> $ret {
                self.0.$name($($arg),*)
            }
        )*
    };
    ($(fn $name: ident<{ $dof: literal }>(&mut self, $($arg: ident: $arg_ty: ty),*) -> $ret: ty;)*) => {
        $(
            pub fn $name(&mut self, $($arg: $arg_ty),*) -> $ret {
                self.0.$name($($arg),*)
            }
        )*
    };
    ($(fn $name: ident<{ $dof: literal }>(&self, $($arg: ident: $arg_ty: ty),*) -> $ret: ty;)*) => {
        $(
            pub fn $name(&self, $($arg: $arg_ty),*) -> $ret {
                self.0.$name($($arg),*)
            }
        )*
    };
}

#[macro_export]
macro_rules! c_robot_behavior {
    ($cname: ident($name: ident)) => {
        impl_self! {
            fn version(&self) -> String;
        }
        impl_self! {
            fn init(&mut self) -> RobotResult<()>;
            fn shutdown(&mut self) -> RobotResult<()>;
            fn enable(&mut self) -> RobotResult<()>;
            fn disable(&mut self) -> RobotResult<()>;
            fn reset(&mut self) -> RobotResult<()>;
            fn is_moving(&mut self) -> bool;
            fn stop(&mut self) -> RobotResult<()>;
            fn pause(&mut self) -> RobotResult<()>;
            fn resume(&mut self) -> RobotResult<()>;
            fn emergency_stop(&mut self) -> RobotResult<()>;
            fn clear_emergency_stop(&mut self) -> RobotResult<()>;
        }
    };
}

#[macro_export]
macro_rules! c_arm_behavior {
    ($cname: ident<{ $dof: expr }>($name: ident)) => {
        impl_self! {
            fn state(&mut self) -> RobotResult<ArmState<$dof>>;
        }
        impl_self! {
            fn set_load(&mut self, load: LoadState) -> RobotResult<()>;
            fn set_coord(&mut self, coord: Coord) -> RobotResult<()>;
            fn with_coord(&mut self, coord: Coord) -> &mut $name;
            fn set_speed(&mut self, speed: f64) -> RobotResult<()>;
            fn with_speed(&mut self, speed: f64) -> &mut $name;
            fn with_velocity(&mut self, joint_vel: &[f64; $dof]) -> &mut $name;
            fn with_cartesian_velocity(&mut self, cartesian_vel: f64) -> &mut $name;
            fn with_acceleration(&mut self, joint_acc: &[f64; $dof]) -> &mut $name;
            fn with_jerk(&mut self, joint_jerk: &[f64; $dof]) -> &mut $name;
        }
    };
}

#[macro_export]
macro_rules! c_arm_param {
    ($cname: ident<{ $dof: expr }>($name: ident)) => {
        impl_self! {
            fn dh(&self) -> [[f64; 4]; $dof];
            fn joint_default(&self) -> [f64; $dof];
            fn joint_min(&self) -> [f64; $dof];
            fn joint_max(&self) -> [f64; $dof];
            fn joint_vel_bound(&self) -> [f64; $dof];
            fn joint_acc_bound(&self) -> [f64; $dof];
            fn joint_jerk_bound(&self) -> [f64; $dof];
            fn cartesian_vel_bound(&self) -> f64;
            fn cartesian_acc_bound(&self) -> f64;
            fn torque_bound(&self) -> [f64; $dof];
            fn torque_dot_bound(&self) -> [f64; $dof];
        }

        fn forward_kinematics(q: &[f64; $dof]) -> Pose {
            $name::forward_kinematics(q)
        }
        // fn inverse_kinematics(pose: Pose) -> RobotResult<[f64; $dof]> {
        //     $name::inverse_kinematics(pose)
        // }
    };
}

#[macro_export]
macro_rules! c_arm_preplanned_motion_impl {
    ($cname: ident<{ $dof: expr }>($name: ident)) => {
        impl_self! {
            fn move_joint(&mut self, target: &[f64; $dof]) -> RobotResult<()>;
            fn move_joint_async(&mut self, target: &[f64; $dof]) -> RobotResult<()>;

            fn move_cartesian(&mut self, target: &Pose) -> RobotResult<()>;
            fn move_cartesian_async(&mut self, target: &Pose) -> RobotResult<()>;
        }
    };
}

#[macro_export]
macro_rules! c_arm_preplanned_motion {
    ($cname: ident<{ $dof: expr }>($name: ident)) => {
        impl_self! {
            fn move_to(&mut self, target: MotionType<$dof>) -> RobotResult<()>;
            fn move_to_async(&mut self, target: MotionType<$dof>) -> RobotResult<()>;
            fn move_rel(&mut self, target: MotionType<$dof>) -> RobotResult<()>;
            fn move_rel_async(&mut self, target: MotionType<$dof>) -> RobotResult<()>;
            fn move_int(&mut self, target: MotionType<$dof>) -> RobotResult<()>;
            fn move_int_async(&mut self, target: MotionType<$dof>) -> RobotResult<()>;
            fn move_path(&mut self, path: Vec<MotionType<$dof>>) -> RobotResult<()>;
            fn move_path_async(&mut self, path: Vec<MotionType<$dof>>) -> RobotResult<()>;
            fn move_path_prepare(&mut self, path: Vec<MotionType<$dof>>) -> RobotResult<()>;
            fn move_path_start(&mut self, start: MotionType<$dof>) -> RobotResult<()>;
        }
    };
}

#[macro_export]
macro_rules! c_arm_preplanned_motion_ext {
    ($cname: ident<{ $dof: expr }>($name: ident)) => {
        impl_self! {
            fn move_joint_rel(&mut self, target: &[f64; $dof]) -> RobotResult<()>;
            fn move_joint_rel_async(&mut self, target: &[f64; $dof]) -> RobotResult<()>;
            fn move_joint_path(&mut self, path: Vec<[f64; $dof]>) -> RobotResult<()>;
            fn move_cartesian_rel(&mut self, target: &Pose) -> RobotResult<()>;
            fn move_cartesian_rel_async(&mut self, target: &Pose) -> RobotResult<()>;
            fn move_cartesian_int(&mut self, target: &Pose) -> RobotResult<()>;
            fn move_cartesian_int_async(&mut self, target: &Pose) -> RobotResult<()>;
            fn move_cartesian_path(&mut self, path: Vec<Pose>) -> RobotResult<()>;

            fn move_linear_with_euler(&mut self, pose: [f64; 6]) -> RobotResult<()>;
            fn move_linear_with_euler_async(&mut self, pose: [f64; 6]) -> RobotResult<()>;
            fn move_linear_with_euler_rel(&mut self, pose: [f64; 6]) -> RobotResult<()>;
            fn move_linear_with_euler_rel_async(&mut self, pose: [f64; 6]) -> RobotResult<()>;
            fn move_linear_with_euler_int(&mut self, pose: [f64; 6]) -> RobotResult<()>;
            fn move_linear_with_euler_int_async(&mut self, pose: [f64; 6]) -> RobotResult<()>;
            fn move_linear_with_quat(&mut self, target: &na::Isometry3<f64>) -> RobotResult<()>;
            fn move_linear_with_quat_async(&mut self, target: &na::Isometry3<f64>) -> RobotResult<()>;
            fn move_linear_with_quat_rel(&mut self, target: &na::Isometry3<f64>) -> RobotResult<()>;
            fn move_linear_with_quat_rel_async(&mut self, target: &na::Isometry3<f64>) -> RobotResult<()>;
            fn move_linear_with_quat_int(&mut self, target: &na::Isometry3<f64>) -> RobotResult<()>;
            fn move_linear_with_quat_int_async(&mut self, target: &na::Isometry3<f64>) -> RobotResult<()>;
            fn move_linear_with_homo(&mut self, target: [f64; 16]) -> RobotResult<()>;
            fn move_linear_with_homo_async(&mut self, target: [f64; 16]) -> RobotResult<()>;
            fn move_linear_with_homo_rel(&mut self, target: [f64; 16]) -> RobotResult<()>;
            fn move_linear_with_homo_rel_async(&mut self, target: [f64; 16]) -> RobotResult<()>;
            fn move_linear_with_homo_int(&mut self, target: [f64; 16]) -> RobotResult<()>;
            fn move_linear_with_homo_int_async(&mut self, target: [f64; 16]) -> RobotResult<()>;

            fn move_path_prepare_from_file(&mut self, path: &str) -> RobotResult<()>;
            fn move_path_from_file(&mut self, path: &str) -> RobotResult<()>;
        }
    };
}

#[macro_export]
macro_rules! c_arm_streaming_handle {
    ($cname: ident<{ $dof: expr }>($name: ident)) => {
        impl_self! {
            fn last_motion(&self) -> RobotResult<MotionType<$dof>>;
            fn move_to(&mut self, target: MotionType<$dof>) -> RobotResult<()>;

            fn last_control(&self) -> RobotResult<ControlType<$dof>>;
            fn control_with(&mut self, control: ControlType<$dof>) -> RobotResult<()>;
        }
    };
}

#[macro_export]
macro_rules! c_arm_streaming_motion {
    ($cname: ident<{ $dof: expr }>($name: ident) -> $handle_name: ty) => {
        impl_self! {
            fn start_streaming(&mut self) -> RobotResult<$handle_name>;
            fn end_streaming(&mut self) -> RobotResult<()>;

            fn move_to_target(&mut self) -> Arc<Mutex<Option<MotionType<$dof>>>>;
            fn control_with_target(&mut self) -> Arc<Mutex<Option<ControlType<$dof>>>>;
        }
    };
}

#[macro_export]
macro_rules! c_arm_streaming_motion_ext {
    ($cname: ident<{ $dof: expr }>($name: ident)) => {};
}

#[macro_export]
macro_rules! c_arm_realtime_control {
    ($cname: ident<{ $dof: expr }>($name: ident)) => {
        impl_self! {}
    };
}

#[cfg(all(test, feature = "to_cxx"))]
mod test {
    struct TestRobot;
    struct CTestRobot(TestRobot);

    impl CTestRobot {
        c_robot_behavior!(CTestRobot(TestRobot));
        c_arm_behavior!(CTestRobot<{0}>(TestRobot));
        c_arm_param!(CTestRobot<{0}>(TestRobot));
        c_arm_preplanned_motion_impl!(CTestRobot<{0}>(TestRobot));
        c_arm_preplanned_motion!(CTestRobot<{0}>(TestRobot));
        c_arm_preplanned_motion_ext!(CTestRobot<{0}>(TestRobot));
    }

    macro_rules! unimpl {
        // For methods with return type only (no arguments)
        ($(fn $name:ident(&self) -> $ret:ty;)+) => {
            $(
                fn $name(&self) -> $ret {
                    unimplemented!()
                }
            )+
        };
        ($(fn $name:ident(&mut self) -> $ret:ty;)+) => {
            $(
                fn $name(&mut self) -> $ret {
                    unimplemented!()
                }
            )+
        };
        ($(fn $name:ident(&mut self, $($arg: ident: $arg_ty: ty),*) -> $ret:ty;)+) => {
            $(
                fn $name(&mut self, $($arg: $arg_ty),* ) -> $ret {
                    unimplemented!()
                }
            )+
        };
    }

    impl RobotBehavior for TestRobot {
        type State = ();
        unimpl! {
            fn version(&self) -> String;
        }
        unimpl! {
            fn is_moving(&mut self) -> bool;
            fn init(&mut self) -> RobotResult<()>;
            fn shutdown(&mut self) -> RobotResult<()>;
            fn enable(&mut self) -> RobotResult<()>;
            fn disable(&mut self) -> RobotResult<()>;
            fn reset(&mut self) -> RobotResult<()>;
            fn stop(&mut self) -> RobotResult<()>;
            fn pause(&mut self) -> RobotResult<()>;
            fn resume(&mut self) -> RobotResult<()>;
            fn emergency_stop(&mut self) -> RobotResult<()>;
            fn clear_emergency_stop(&mut self) -> RobotResult<()>;
            fn read_state(&mut self) -> RobotResult<Self::State>;
        }
    }

    impl ArmBehavior<0> for TestRobot {
        unimpl!(
            fn state(&mut self) -> RobotResult<ArmState<0>>;
        );
        unimpl!(
            fn set_load(&mut self, _load: LoadState) -> RobotResult<()>;
            fn set_coord(&mut self, _coord: Coord) -> RobotResult<()>;
            fn with_coord(&mut self, _coord: Coord) -> &mut Self;
            fn set_speed(&mut self, _speed: f64) -> RobotResult<()>;
            fn with_speed(&mut self, _speed: f64) -> &mut Self;

            fn with_velocity(&mut self, _joint_vel: &[f64; 0]) -> &mut Self;
            fn with_cartesian_velocity(&mut self, _cartesian_vel: f64) -> &mut Self;
            fn with_acceleration(&mut self, _joint_acc: &[f64; 0]) -> &mut Self;
            fn with_jerk(&mut self, _joint_jerk: &[f64; 0]) -> &mut Self;
        );
    }

    impl ArmParam<0> for TestRobot {
        const DH: [[f64; 4]; 0] = [];
        const JOINT_MIN: [f64; 0] = [];
        const JOINT_MAX: [f64; 0] = [];
    }

    impl ArmPreplannedMotionImpl<0> for TestRobot {
        unimpl!(
            fn move_joint(&mut self, _target: &[f64; 0]) -> RobotResult<()>;
            fn move_joint_async(&mut self, _target: &[f64; 0]) -> RobotResult<()>;

            fn move_cartesian(&mut self, _target: &Pose) -> RobotResult<()>;
            fn move_cartesian_async(&mut self, _target: &Pose) -> RobotResult<()>;
        );
    }

    impl ArmPreplannedMotion<0> for TestRobot {
        #[rustfmt::skip]
        unimpl!(
            fn move_path(&mut self, _path: Vec<MotionType<0>>) -> RobotResult<()>;
            fn move_path_async(&mut self, _path: Vec<MotionType<0>>) -> RobotResult<()>;
            fn move_path_prepare(&mut self, _path: Vec<MotionType<0>>) -> RobotResult<()>;
            fn move_path_start(&mut self, _start: MotionType<0>) -> RobotResult<()>;
        );
    }

    impl ArmStreamingHandle<0> for TestRobotHandle {
        unimpl!(
            fn last_motion(&self) -> RobotResult<MotionType<0>>;
            fn last_control(&self) -> RobotResult<ControlType<0>>;
        );
        unimpl!(
            fn move_to(&mut self, _target: MotionType<0>) -> RobotResult<()>;
            fn control_with(&mut self, _target: ControlType<0>) -> RobotResult<()>;
        );
    }

    impl ArmStreamingMotion<0> for TestRobot {
        type Handle = TestRobotHandle;
        unimpl!(
            fn start_streaming(&mut self) -> RobotResult<Self::Handle>;
            fn end_streaming(&mut self) -> RobotResult<()>;

            fn move_to_target(&mut self) -> Arc<Mutex<Option<MotionType<0>>>>;
            fn control_with_target(&mut self) -> Arc<Mutex<Option<ControlType<0>>>>;
        );
    }

    impl ArmRealtimeControl<0> for TestRobot {
        fn control_with_closure<FC>(&mut self, _closure: FC) -> RobotResult<()>
        where
            FC: FnMut(ArmState<0>, std::time::Duration) -> (ControlType<0>, bool) + Send + 'static,
        {
            unimplemented!()
        }
        fn move_with_closure<FM>(&mut self, _closure: FM) -> RobotResult<()>
        where
            FM: FnMut(ArmState<0>, std::time::Duration) -> (MotionType<0>, bool) + Send + 'static,
        {
            unimplemented!()
        }
    }

    impl ArmRealtimeControlExt<0> for TestRobot {}
}
