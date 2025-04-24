#[macro_export]
macro_rules! py_robot_behavior {
    ($pyname: ident($name: ident)) => {
        #[pyo3::pymethods]
        impl $pyname {
            fn version(&self) -> String {
                self.0.version()
            }
            fn init(&mut self) -> pyo3::PyResult<()> {
                self.0.init().map_err(Into::into)
            }
            fn shutdown(&mut self) -> pyo3::PyResult<()> {
                self.0.shutdown().map_err(Into::into)
            }
            fn enable(&mut self) -> pyo3::PyResult<()> {
                self.0.enable().map_err(Into::into)
            }
            fn disable(&mut self) -> pyo3::PyResult<()> {
                self.0.disable().map_err(Into::into)
            }
            fn reset(&mut self) -> pyo3::PyResult<()> {
                self.0.reset().map_err(Into::into)
            }
            fn is_moving(&mut self) -> bool {
                self.0.is_moving()
            }
            fn stop(&mut self) -> pyo3::PyResult<()> {
                self.0.stop().map_err(Into::into)
            }
            fn pause(&mut self) -> pyo3::PyResult<()> {
                self.0.pause().map_err(Into::into)
            }
            fn resume(&mut self) -> pyo3::PyResult<()> {
                self.0.resume().map_err(Into::into)
            }
            fn emergency_stop(&mut self) -> pyo3::PyResult<()> {
                self.0.emergency_stop().map_err(Into::into)
            }
            fn clear_emergency_stop(&mut self) -> pyo3::PyResult<()> {
                self.0.clear_emergency_stop().map_err(Into::into)
            }
        }
    };
}

#[macro_export]
macro_rules! py_arm_behavior {
    ($pyname: ident<{$dof: expr}>($name: ident)) => {
        #[pyo3::pymethods]
        impl $pyname {
            fn state(&mut self) -> pyo3::PyResult<$crate::PyArmState> {
                self.0.state().map(Into::into).map_err(Into::into)
            }
            fn set_load(&mut self, load: $crate::LoadState) -> pyo3::PyResult<()> {
                self.0.set_load(load).map_err(Into::into)
            }
        }
    };
}

#[macro_export]
macro_rules! py_arm_preplanned_motion {
    ($pyname: ident<{ $dof: literal }>($name: ident)) => {
        #[pyo3::pymethods]
        impl $pyname {
            fn move_to(
                &mut self,
                target: ::paste::paste! {$crate::[<MotionType $dof>]},
                speed: f64,
            ) -> pyo3::PyResult<()> {
                self.0.move_to(target.into(), speed).map_err(Into::into)
            }
            fn move_to_async(
                &mut self,
                target: ::paste::paste! {$crate::[<MotionType $dof>]},
                speed: f64,
            ) -> pyo3::PyResult<()> {
                self.0
                    .move_to_async(target.into(), speed)
                    .map_err(Into::into)
            }
            fn move_rel(
                &mut self,
                target: ::paste::paste! {$crate::[<MotionType $dof>]},
                speed: f64,
            ) -> pyo3::PyResult<()> {
                self.0.move_rel(target.into(), speed).map_err(Into::into)
            }
            fn move_rel_async(
                &mut self,
                target: ::paste::paste! {$crate::[<MotionType $dof>]},
                speed: f64,
            ) -> pyo3::PyResult<()> {
                self.0
                    .move_rel_async(target.into(), speed)
                    .map_err(Into::into)
            }
        }
    };
}

#[macro_export]
macro_rules! py_arm_preplanned_motion_ext {
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {
        #[pyo3::pymethods]
        impl $pyname {
            fn move_joint(&mut self, target: [f64; $dof], speed: f64) -> pyo3::PyResult<()> {
                self.0.move_joint(&target, speed).map_err(Into::into)
            }
            fn move_joint_async(&mut self, target: [f64; $dof], speed: f64) -> pyo3::PyResult<()> {
                self.0.move_joint_async(&target, speed).map_err(Into::into)
            }
            fn move_joint_rel(&mut self, target: [f64; $dof], speed: f64) -> pyo3::PyResult<()> {
                self.0.move_joint_rel(&target, speed).map_err(Into::into)
            }
            fn move_joint_rel_async(
                &mut self,
                target: [f64; $dof],
                speed: f64,
            ) -> pyo3::PyResult<()> {
                self.0
                    .move_joint_rel_async(&target, speed)
                    .map_err(Into::into)
            }
            fn move_joint_path(
                &mut self,
                target: Vec<[f64; $dof]>,
                speed: f64,
            ) -> pyo3::PyResult<()> {
                self.0.move_joint_path(target, speed).map_err(Into::into)
            }

            fn move_cartesian(&mut self, target: $crate::PyPose, speed: f64) -> pyo3::PyResult<()> {
                self.0
                    .move_cartesian(&target.into(), speed)
                    .map_err(Into::into)
            }
            fn move_cartesian_async(
                &mut self,
                target: $crate::PyPose,
                speed: f64,
            ) -> pyo3::PyResult<()> {
                self.0
                    .move_cartesian_async(&target.into(), speed)
                    .map_err(Into::into)
            }
            fn move_cartesian_rel(
                &mut self,
                target: $crate::PyPose,
                speed: f64,
            ) -> pyo3::PyResult<()> {
                self.0
                    .move_cartesian_rel(&target.into(), speed)
                    .map_err(Into::into)
            }
            fn move_cartesian_rel_async(
                &mut self,
                target: $crate::PyPose,
                speed: f64,
            ) -> pyo3::PyResult<()> {
                self.0
                    .move_cartesian_rel_async(&target.into(), speed)
                    .map_err(Into::into)
            }
            fn move_cartesian_path(
                &mut self,
                target: Vec<$crate::PyPose>,
                speed: f64,
            ) -> pyo3::PyResult<()> {
                self.0
                    .move_cartesian_path(target.into_iter().map(Into::into).collect(), speed)
                    .map_err(Into::into)
            }

            fn move_path_from_file(&mut self, path: &str, speed: f64) -> pyo3::PyResult<()> {
                self.0.move_path_from_file(path, speed).map_err(Into::into)
            }
            fn move_path_prepare(
                &mut self,
                path: Vec<::paste::paste! {$crate::[<MotionType $dof>]}>,
            ) -> pyo3::PyResult<()> {
                self.0
                    .move_path_prepare(path.into_iter().map(Into::into).collect())
                    .map_err(Into::into)
            }
            fn move_path_start(&mut self) -> pyo3::PyResult<()> {
                self.0.move_path_start().map_err(Into::into)
            }
            fn move_path_prepare_from_file(&mut self, path: &str) -> pyo3::PyResult<()> {
                self.0.move_path_prepare_from_file(path).map_err(Into::into)
            }
        }
    };
}

#[macro_export]
macro_rules! py_arm_streaming_handle {
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {
        #[pyo3::pymethods]
        impl $pyname {
            fn move_to(
                &mut self,
                target: ::paste::paste! {$crate::[<MotionType $dof>]},
            ) -> pyo3::PyResult<()> {
                self.0.move_to(target.into()).map_err(Into::into)
            }
            fn last_motion(&self) -> pyo3::PyResult<::paste::paste! {$crate::[<MotionType $dof>]}> {
                self.0.last_motion().map(Into::into).map_err(Into::into)
            }

            fn control_with(
                &mut self,
                target: ::paste::paste! {$crate::[<ControlType $dof>]},
            ) -> pyo3::PyResult<()> {
                self.0.control_with(target.into()).map_err(Into::into)
            }
            fn last_control(
                &self,
            ) -> pyo3::PyResult<::paste::paste! {$crate::[<ControlType $dof>]}> {
                self.0.last_control().map(Into::into).map_err(Into::into)
            }
        }
    };
}

#[macro_export]
macro_rules! py_arm_streaming_motion {
    ($pyname: ident<{ $dof: expr }>($name: ident) -> $handle_name: ident) => {
        #[pyo3::pymethods]
        impl $pyname {
            fn start_streaming(&mut self) -> pyo3::PyResult<$handle_name> {
                self.0.start_streaming().map_err(Into::into)
            }
            fn stop_streaming(&mut self) -> pyo3::PyResult<()> {
                self.0.stop_streaming().map_err(Into::into)
            }
        }
    };
}

#[macro_export]
macro_rules! py_arm_streaming_motion_ext {
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {};
}

#[macro_export]
macro_rules! py_arm_real_time_control {
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {};
}

#[macro_export]
macro_rules! py_arm_real_time_control_ext {
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {};
}

// macro_rules! py_arm_state {
//     ($name: ident, $dof: expr) => {
//         #[pyo3::pyclass]
//         pub struct $name(ArmState<$dof>);

//         #[pyo3::pymethods]
//         impl $name {
//             fn echo(&self) -> pyo3::PyResult<String> {
//                 Ok(format!("{}", self.0))
//             }

//             #[getter]
//             fn joint(&self) -> pyo3::PyResult<Option<[f64; $dof]>> {
//                 Ok(self.0.joint)
//             }

//             #[getter]
//             fn joint_vel(&self) -> pyo3::PyResult<Option<[f64; $dof]>> {
//                 Ok(self.0.joint_vel)
//             }

//             #[getter]
//             fn joint_acc(&self) -> pyo3::PyResult<Option<[f64; $dof]>> {
//                 Ok(self.0.joint_acc)
//             }
//         }

//         impl From<ArmState<$dof>> for $name {
//             fn from(state: ArmState<$dof>) -> Self {
//                 $name(state)
//             }
//         }
//     };
// }

// py_arm_state!(ArmState1, 1);
// py_arm_state!(ArmState2, 2);
// py_arm_state!(ArmState3, 3);
// py_arm_state!(ArmState4, 4);
// py_arm_state!(ArmState5, 5);
// py_arm_state!(ArmState6, 6);
// py_arm_state!(ArmState7, 7);

#[cfg(all(test, feature = "to_py"))]
mod test {
    use pyo3;

    use crate::{LoadState, RobotBehavior, RobotResult, arm::*, types::*};

    struct TestRobot;

    #[pyo3::pyclass]
    struct PyTestRobot(TestRobot);

    py_robot_behavior!(PyTestRobot(TestRobot));
    py_arm_behavior!(PyTestRobot<{0}>(TestRobot));
    py_arm_preplanned_motion!(PyTestRobot<{0}>(TestRobot));
    py_arm_preplanned_motion_ext!(PyTestRobot<{0}>(TestRobot));

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
        );
    }

    impl ArmPreplannedMotion<0> for TestRobot {
        unimpl!(
            fn move_to(&mut self, _target: MotionType<0>, _speed: f64) -> RobotResult<()>;
            fn move_to_async(&mut self, _target: MotionType<0>, _speed: f64) -> RobotResult<()>;
            fn move_rel(&mut self, _target: MotionType<0>, _speed: f64) -> RobotResult<()>;
            fn move_rel_async(&mut self, _target: MotionType<0>, _speed: f64) -> RobotResult<()>;
            fn move_path(&mut self, _path: Vec<MotionType<0>>, _speed: f64) -> RobotResult<()>;
            fn control_with(&mut self, _control: ControlType<0>) -> RobotResult<()>;
        );
    }

    impl ArmPreplannedMotionExt<0> for TestRobot {
        unimpl!(
            fn move_path_prepare(&mut self, _path: Vec<MotionType<0>>) -> RobotResult<()>;
        );
        unimpl!(
            fn move_path_start(&mut self) -> RobotResult<()>;
        );
    }
}
