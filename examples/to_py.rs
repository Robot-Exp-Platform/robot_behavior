fn main() {}

#[cfg(feature = "to_py")]
mod to_py {
    use pyo3::types::{PyAnyMethods, PyModule, PyModuleMethods};
    use robot_behavior::*;
    use std::sync::{Arc, Mutex};

    struct ExRobot;

    #[pyo3::pyclass]
    struct PyExRobot(ExRobot);

    py_robot_behavior!(PyExRobot(ExRobot));
    py_arm_behavior!(PyExRobot<{0}>(ExRobot));
    py_arm_param!(PyExRobot<{0}>(ExRobot));

    py_arm_preplanned_motion_impl!(PyExRobot<{0}>(ExRobot));
    py_arm_preplanned_motion!(PyExRobot<{0}>(ExRobot));
    py_arm_preplanned_motion_ext!(PyExRobot<{0}>(ExRobot));

    #[pyo3::pyclass]
    struct ExRobotHandle;
    py_arm_streaming_motion!(PyExRobot<{0}>(ExRobot) -> ExRobotHandle);
    py_arm_streaming_motion_ext!(PyExRobot<{0}>(ExRobot));

    py_arm_real_time_control!(PyExRobot<{0}>(ExRobot));
    py_arm_real_time_control_ext!(PyExRobot<{0}>(ExRobot));

    #[pyo3::pymodule]
    fn ex_robot(m: &pyo3::Bound<'_, PyModule>) -> pyo3::PyResult<()> {
        m.add_class::<PyExRobot>()?;
        m.add_class::<PyPose>()?;
        m.add_class::<PyArmState>()?;
        m.add_class::<LoadState>()?;
        Ok(())
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

    impl RobotBehavior for ExRobot {
        type State = ();
        fn version() -> String {
            "ExRobot v0.1.0".to_string()
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

    impl ArmBehavior<0> for ExRobot {
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

    impl ArmParam<0> for ExRobot {
        const DH: [[f64; 4]; 0] = [];
        const JOINT_MIN: [f64; 0] = [];
        const JOINT_MAX: [f64; 0] = [];
    }

    impl ArmPreplannedMotionImpl<0> for ExRobot {
        unimpl!(
            fn move_joint(&mut self, _target: &[f64; 0]) -> RobotResult<()>;
            fn move_joint_async(&mut self, _target: &[f64; 0]) -> RobotResult<()>;

            fn move_cartesian(&mut self, _target: &Pose) -> RobotResult<()>;
            fn move_cartesian_async(&mut self, _target: &Pose) -> RobotResult<()>;
        );
    }

    impl ArmPreplannedMotion<0> for ExRobot {
        #[rustfmt::skip]
        unimpl!(
                fn move_path(&mut self, _path: Vec<MotionType<0>>) -> RobotResult<()>;
                fn move_path_async(&mut self, _path: Vec<MotionType<0>>) -> RobotResult<()>;
                fn move_path_prepare(&mut self, _path: Vec<MotionType<0>>) -> RobotResult<()>;
                fn move_path_start(&mut self, _start: MotionType<0>) -> RobotResult<()>;
            );
    }

    impl ArmStreamingHandle<0> for ExRobotHandle {
        unimpl!(
            fn last_motion(&self) -> RobotResult<MotionType<0>>;
            fn last_control(&self) -> RobotResult<ControlType<0>>;
        );
        unimpl!(
            fn move_to(&mut self, _target: MotionType<0>) -> RobotResult<()>;
            fn control_with(&mut self, _target: ControlType<0>) -> RobotResult<()>;
        );
    }

    impl ArmStreamingMotion<0> for ExRobot {
        type Handle = ExRobotHandle;
        unimpl!(
            fn start_streaming(&mut self) -> RobotResult<Self::Handle>;
            fn end_streaming(&mut self) -> RobotResult<()>;

            fn move_to_target(&mut self) -> Arc<Mutex<Option<MotionType<0>>>>;
            fn control_with_target(&mut self) -> Arc<Mutex<Option<ControlType<0>>>>;
        );
    }

    impl ArmRealtimeControl<0> for ExRobot {
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

    impl ArmRealtimeControlExt<0> for ExRobot {}
}
