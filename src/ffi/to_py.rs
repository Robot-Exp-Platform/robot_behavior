#[macro_export]
macro_rules! py_robot_behavior {
    ($pyname: ident($name: ident)) => {
        #[pyo3::pymethods]
        impl $pyname {
            #[staticmethod]
            fn version() -> String {
                $name::version()
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
            fn set_coord(&mut self, coord: String) -> pyo3::PyResult<()> {
                self.0.set_coord(coord.as_str().into()).map_err(Into::into)
            }
            fn with_coord(mut self_: pyo3::PyRefMut<'_, Self>, coord: String) -> pyo3::Py<Self> {
                self_.0.with_coord(coord.as_str().into());
                self_.into()
            }

            fn set_speed(&mut self, speed: f64) -> pyo3::PyResult<()> {
                self.0.set_speed(speed).map_err(Into::into)
            }

            fn with_speed(mut self_: pyo3::PyRefMut<'_, Self>, speed: f64) -> pyo3::Py<Self> {
                self_.0.with_speed(speed);
                self_.into()
            }
            fn with_velocity(
                mut self_: pyo3::PyRefMut<'_, Self>,
                joint_vel: [f64; $dof],
            ) -> pyo3::Py<Self> {
                self_.0.with_velocity(&joint_vel);
                self_.into()
            }
            fn with_acceleration(
                mut self_: pyo3::PyRefMut<'_, Self>,
                joint_acc: [f64; $dof],
            ) -> pyo3::Py<Self> {
                self_.0.with_acceleration(&joint_acc);
                self_.into()
            }
            fn with_jerk(
                mut self_: pyo3::PyRefMut<'_, Self>,
                joint_jerk: [f64; $dof],
            ) -> pyo3::Py<Self> {
                self_.0.with_jerk(&joint_jerk);
                self_.into()
            }
            fn with_cartesian_velocity(
                mut self_: pyo3::PyRefMut<'_, Self>,
                cartesian_vel: f64,
            ) -> pyo3::Py<Self> {
                self_.0.with_cartesian_velocity(cartesian_vel);
                self_.into()
            }
            fn with_cartesian_acceleration(
                mut self_: pyo3::PyRefMut<'_, Self>,
                cartesian_acc: f64,
            ) -> pyo3::Py<Self> {
                self_.0.with_cartesian_acceleration(cartesian_acc);
                self_.into()
            }
            fn with_cartesian_jerk(
                mut self_: pyo3::PyRefMut<'_, Self>,
                cartesian_jerk: f64,
            ) -> pyo3::Py<Self> {
                self_.0.with_cartesian_jerk(cartesian_jerk);
                self_.into()
            }
        }
    };
}

#[macro_export]
macro_rules! py_arm_param {
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {
        #[pyo3::pymethods]
        impl $pyname {
            #[staticmethod]
            fn dh() -> [[f64; 4]; $dof] {
                $name::DH
            }
            #[staticmethod]
            fn joint_default() -> [f64; $dof] {
                $name::JOINT_DEFAULT
            }
            #[staticmethod]
            fn joint_min() -> [f64; $dof] {
                $name::JOINT_MIN
            }
            #[staticmethod]
            fn joint_max() -> [f64; $dof] {
                $name::JOINT_MAX
            }
            #[staticmethod]
            fn joint_vel_bound() -> [f64; $dof] {
                $name::JOINT_VEL_BOUND
            }
            #[staticmethod]
            fn joint_acc_bound() -> [f64; $dof] {
                $name::JOINT_ACC_BOUND
            }
            #[staticmethod]
            fn joint_jerk_bound() -> [f64; $dof] {
                $name::JOINT_JERK_BOUND
            }
            #[staticmethod]
            fn cartesian_vel_bound() -> f64 {
                $name::CARTESIAN_VEL_BOUND
            }
            #[staticmethod]
            fn cartesian_acc_bound() -> f64 {
                $name::CARTESIAN_ACC_BOUND
            }
            #[staticmethod]
            fn cartesian_jerk_bound() -> f64 {
                $name::CARTESIAN_JERK_BOUND
            }
            #[staticmethod]
            fn rotation_vel_bound() -> f64 {
                $name::ROTATION_VEL_BOUND
            }
            #[staticmethod]
            fn rotation_acc_bound() -> f64 {
                $name::ROTATION_ACC_BOUND
            }
            #[staticmethod]
            fn rotation_jerk_bound() -> f64 {
                $name::ROTATION_JERK_BOUND
            }
            #[staticmethod]
            fn torque_bound() -> [f64; $dof] {
                $name::TORQUE_BOUND
            }
            #[staticmethod]
            fn torque_dot_bound() -> [f64; $dof] {
                $name::TORQUE_DOT_BOUND
            }

            #[staticmethod]
            fn forward_kinematics(q: [f64; $dof]) -> $crate::PyPose {
                $name::forward_kinematics(&q).into()
            }

            #[allow(deprecated)]
            fn inverse_kinematics(&self, pose: $crate::PyPose) -> pyo3::PyResult<[f64; $dof]> {
                let pose: $crate::Pose = pose.into();
                $name::inverse_kinematics(pose).map_err(Into::into)
            }
        }
    };
}

#[macro_export]
macro_rules! py_arm_preplanned_motion_impl {
    ($pyname: ident<{ $dof: literal }>($name: ident)) => {
        #[pyo3::pymethods]
        impl $pyname {
            fn move_joint(&mut self, target: [f64; $dof]) -> pyo3::PyResult<()> {
                self.0.move_joint(&target).map_err(Into::into)
            }
            fn move_joint_async(&mut self, target: [f64; $dof]) -> pyo3::PyResult<()> {
                self.0.move_joint_async(&target).map_err(Into::into)
            }

            fn move_cartesian(&mut self, target: $crate::PyPose) -> pyo3::PyResult<()> {
                self.0.move_cartesian(&target.into()).map_err(Into::into)
            }
            fn move_cartesian_async(&mut self, target: $crate::PyPose) -> pyo3::PyResult<()> {
                self.0
                    .move_cartesian_async(&target.into())
                    .map_err(Into::into)
            }
        }
    };
}

#[macro_export]
macro_rules! py_arm_preplanned_motion {
    ($pyname: ident<{ $dof: literal }>($name: ident)) => {
        #[pyo3::pymethods]
        impl $pyname {
            fn move_to(&mut self, target: $crate::PyMotionType) -> pyo3::PyResult<()> {
                self.0.move_to(target.into()).map_err(Into::into)
            }
            fn move_to_async(&mut self, target: $crate::PyMotionType) -> pyo3::PyResult<()> {
                self.0.move_to_async(target.into()).map_err(Into::into)
            }
            fn move_rel(&mut self, target: $crate::PyMotionType) -> pyo3::PyResult<()> {
                self.0.move_rel(target.into()).map_err(Into::into)
            }
            fn move_rel_async(&mut self, target: $crate::PyMotionType) -> pyo3::PyResult<()> {
                self.0.move_rel_async(target.into()).map_err(Into::into)
            }
            fn move_int(&mut self, target: $crate::PyMotionType) -> pyo3::PyResult<()> {
                self.0.move_int(target.into()).map_err(Into::into)
            }
            fn move_int_async(&mut self, target: $crate::PyMotionType) -> pyo3::PyResult<()> {
                self.0.move_int_async(target.into()).map_err(Into::into)
            }
            fn move_path(&mut self, path: Vec<$crate::PyMotionType>) -> pyo3::PyResult<()> {
                self.0
                    .move_path(path.into_iter().map(Into::into).collect())
                    .map_err(Into::into)
            }
            fn move_path_async(&mut self, path: Vec<$crate::PyMotionType>) -> pyo3::PyResult<()> {
                self.0
                    .move_path_async(path.into_iter().map(Into::into).collect())
                    .map_err(Into::into)
            }
            fn move_path_prepare(&mut self, path: Vec<$crate::PyMotionType>) -> pyo3::PyResult<()> {
                self.0
                    .move_path_prepare(path.into_iter().map(Into::into).collect())
                    .map_err(Into::into)
            }
            fn move_path_start(&mut self, start: $crate::PyMotionType) -> pyo3::PyResult<()> {
                self.0.move_path_start(start.into()).map_err(Into::into)
            }
        }
    };
}

#[macro_export]
macro_rules! py_arm_preplanned_motion_ext {
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {
        #[pyo3::pymethods]
        impl $pyname {
            fn move_joint_rel(&mut self, target: [f64; $dof]) -> pyo3::PyResult<()> {
                self.0.move_joint_rel(&target).map_err(Into::into)
            }
            fn move_joint_rel_async(&mut self, target: [f64; $dof]) -> pyo3::PyResult<()> {
                self.0.move_joint_rel_async(&target).map_err(Into::into)
            }
            fn move_joint_path(&mut self, target: Vec<[f64; $dof]>) -> pyo3::PyResult<()> {
                self.0.move_joint_path(target).map_err(Into::into)
            }

            fn move_cartesian_rel(&mut self, target: $crate::PyPose) -> pyo3::PyResult<()> {
                self.0
                    .move_cartesian_rel(&target.into())
                    .map_err(Into::into)
            }
            fn move_cartesian_rel_async(&mut self, target: $crate::PyPose) -> pyo3::PyResult<()> {
                self.0
                    .move_cartesian_rel_async(&target.into())
                    .map_err(Into::into)
            }
            fn move_cartesian_int(&mut self, target: $crate::PyPose) -> pyo3::PyResult<()> {
                self.0
                    .move_cartesian_int(&target.into())
                    .map_err(Into::into)
            }
            fn move_cartesian_int_async(&mut self, target: $crate::PyPose) -> pyo3::PyResult<()> {
                self.0
                    .move_cartesian_int_async(&target.into())
                    .map_err(Into::into)
            }
            fn move_cartesian_path(&mut self, target: Vec<$crate::PyPose>) -> pyo3::PyResult<()> {
                self.0
                    .move_cartesian_path(target.into_iter().map(Into::into).collect())
                    .map_err(Into::into)
            }

            fn move_linear_with_euler(&mut self, target: [f64; 6]) -> pyo3::PyResult<()> {
                self.0.move_linear_with_euler(target).map_err(Into::into)
            }
            fn move_linear_with_euler_async(&mut self, target: [f64; 6]) -> pyo3::PyResult<()> {
                self.0
                    .move_linear_with_euler_async(target)
                    .map_err(Into::into)
            }
            fn move_linear_with_euler_rel(&mut self, target: [f64; 6]) -> pyo3::PyResult<()> {
                self.0
                    .move_linear_with_euler_rel(target)
                    .map_err(Into::into)
            }
            fn move_linear_with_euler_rel_async(&mut self, target: [f64; 6]) -> pyo3::PyResult<()> {
                self.0
                    .move_linear_with_euler_rel_async(target)
                    .map_err(Into::into)
            }
            fn move_linear_with_euler_int(&mut self, target: [f64; 6]) -> pyo3::PyResult<()> {
                self.0
                    .move_linear_with_euler_int(target)
                    .map_err(Into::into)
            }
            fn move_linear_with_euler_int_async(&mut self, target: [f64; 6]) -> pyo3::PyResult<()> {
                self.0
                    .move_linear_with_euler_int_async(target)
                    .map_err(Into::into)
            }

            fn move_path_from_file(&mut self, path: &str) -> pyo3::PyResult<()> {
                self.0.move_path_from_file(path).map_err(Into::into)
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
            fn move_to(&mut self, target: $crate::PyMotionType) -> pyo3::PyResult<()> {
                self.0.move_to(target.into()).map_err(Into::into)
            }
            fn last_motion(&self) -> pyo3::PyResult<$crate::PyMotionType> {
                self.0.last_motion().map(Into::into).map_err(Into::into)
            }

            fn control_with(&mut self, target: $crate::PyControlType) -> pyo3::PyResult<()> {
                self.0.control_with(target.into()).map_err(Into::into)
            }
            fn last_control(&self) -> pyo3::PyResult<$crate::PyControlType> {
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
                self.0.start_streaming().map_err(Into::into).map(Into::into)
            }
            fn end_streaming(&mut self) -> pyo3::PyResult<()> {
                self.0.end_streaming().map_err(Into::into)
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
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {
        #[pyo3::pymethods]
        impl $pyname {
            fn move_with_closure(&mut self, closure: pyo3::Py<pyo3::PyAny>) -> pyo3::PyResult<()> {
                self.0
                    .move_with_closure(move |state, duration| {
                        let state = $crate::PyArmState::from(state);
                        let duration = duration.as_secs_f64();
                        pyo3::Python::with_gil(|py| {
                            closure
                                .call1(py, (state, duration))
                                .unwrap()
                                .bind(py)
                                .extract::<($crate::PyMotionType, bool)>()
                                .map(|(motion, stop)| (motion.into(), stop))
                                .unwrap()
                        })
                    })
                    .map_err(Into::into)
            }
            fn control_with_closure(
                &mut self,
                closure: pyo3::Py<pyo3::PyAny>,
            ) -> pyo3::PyResult<()> {
                self.0
                    .control_with_closure(move |state, duration| {
                        let state = $crate::PyArmState::from(state);
                        let duration = duration.as_secs_f64();
                        pyo3::Python::with_gil(|py| {
                            closure
                                .call1(py, (state, duration))
                                .unwrap()
                                .bind(py)
                                .extract::<($crate::PyControlType, bool)>()
                                .map(|(control, stop)| (control.into(), stop))
                                .unwrap()
                        })
                    })
                    .map_err(Into::into)
            }
        }
    };
}

#[macro_export]
macro_rules! py_arm_real_time_control_ext {
    ($pyname: ident<{ $dof: expr }>($name: ident)) => {
        #[pyo3::pymethods]
        impl $pyname {
            fn move_joint_with_closure(
                &mut self,
                closure: pyo3::Py<pyo3::PyAny>,
            ) -> pyo3::PyResult<()> {
                self.0
                    .move_joint_with_closure(move |state, duration| {
                        let state = $crate::PyArmState::from(state);
                        let duration = duration.as_secs_f64();
                        pyo3::Python::with_gil(|py| {
                            closure
                                .call1(py, (state, duration))
                                .unwrap()
                                .bind(py)
                                .extract::<([f64; $dof], bool)>()
                                .unwrap()
                        })
                    })
                    .map_err(Into::into)
            }

            fn move_joint_vel_with_closure(
                &mut self,
                closure: pyo3::Py<pyo3::PyAny>,
            ) -> pyo3::PyResult<()> {
                self.0
                    .move_joint_vel_with_closure(move |state, duration| {
                        let state = $crate::PyArmState::from(state);
                        let duration = duration.as_secs_f64();
                        pyo3::Python::with_gil(|py| {
                            closure
                                .call1(py, (state, duration))
                                .unwrap()
                                .bind(py)
                                .extract::<([f64; $dof], bool)>()
                                .unwrap()
                        })
                    })
                    .map_err(Into::into)
            }

            fn move_cartesian_with_closure(
                &mut self,
                closure: pyo3::Py<pyo3::PyAny>,
            ) -> pyo3::PyResult<()> {
                self.0
                    .move_cartesian_with_closure(move |state, duration| {
                        let state = $crate::PyArmState::from(state);
                        let duration = duration.as_secs_f64();
                        pyo3::Python::with_gil(|py| {
                            closure
                                .call1(py, (state, duration))
                                .unwrap()
                                .bind(py)
                                .extract::<($crate::PyPose, bool)>()
                                .map(|(pose, stop)| (pose.into(), stop))
                                .unwrap()
                        })
                    })
                    .map_err(Into::into)
            }

            fn move_cartesian_vel_with_closure(
                &mut self,
                closure: pyo3::Py<pyo3::PyAny>,
            ) -> pyo3::PyResult<()> {
                self.0
                    .move_cartesian_vel_with_closure(move |state, duration| {
                        let state = $crate::PyArmState::from(state);
                        let duration = duration.as_secs_f64();
                        pyo3::Python::with_gil(|py| {
                            closure
                                .call1(py, (state, duration))
                                .unwrap()
                                .bind(py)
                                .extract::<([f64; 6], bool)>()
                                .unwrap()
                        })
                    })
                    .map_err(Into::into)
            }
        }
    };
}

#[cfg(all(test, feature = "to_py"))]
mod test {
    use pyo3::types::PyAnyMethods;
    use std::sync::{Arc, Mutex};

    use crate::{LoadState, RobotResult, arm::*, behavior::*, types::*};

    struct TestRobot;

    #[pyo3::pyclass]
    struct PyTestRobot(TestRobot);

    py_robot_behavior!(PyTestRobot(TestRobot));
    py_arm_behavior!(PyTestRobot<{0}>(TestRobot));
    py_arm_param!(PyTestRobot<{0}>(TestRobot));

    py_arm_preplanned_motion_impl!(PyTestRobot<{0}>(TestRobot));
    py_arm_preplanned_motion!(PyTestRobot<{0}>(TestRobot));
    py_arm_preplanned_motion_ext!(PyTestRobot<{0}>(TestRobot));

    #[pyo3::pyclass]
    struct TestRobotHandle;
    py_arm_streaming_motion!(PyTestRobot<{0}>(TestRobot) -> TestRobotHandle);
    py_arm_streaming_motion_ext!(PyTestRobot<{0}>(TestRobot));

    py_arm_real_time_control!(PyTestRobot<{0}>(TestRobot));
    py_arm_real_time_control_ext!(PyTestRobot<{0}>(TestRobot));

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
        fn version() -> String {
            "TestRobot v0.1.0".to_string()
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
            fn with_acceleration(&mut self, _joint_acc: &[f64; 0]) -> &mut Self;
            fn with_jerk(&mut self, _joint_jerk: &[f64; 0]) -> &mut Self;
            fn with_cartesian_velocity(&mut self, _cartesian_vel: f64) -> &mut Self;
            fn with_cartesian_acceleration(&mut self, _cartesian_acc: f64) -> &mut Self;
            fn with_cartesian_jerk(&mut self, _cartesian_jerk: f64) -> &mut Self;
            fn with_rotation_velocity(&mut self, _rotation_vel: f64) -> &mut Self;
            fn with_rotation_acceleration(&mut self, _rotation_acc: f64) -> &mut Self;
            fn with_rotation_jerk(&mut self, _rotation_jerk: f64) -> &mut Self;
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
