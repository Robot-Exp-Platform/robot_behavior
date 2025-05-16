fn main() {
    #[cfg(all(feature = "to_py"))]
    {
        use pyo3;
        use robot_behavior::*;

        struct ExRobot;

        #[pyo3::pyclass]
        struct PyExRobot(ExRobot);

        py_robot_behavior!(PyExRobot(ExRobot));
        py_arm_behavior!(PyExRobot<{0}>(ExRobot));
        py_arm_param!(PyExRobot<{0}>(ExRobot));
        py_arm_preplanned_motion!(PyExRobot<{0}>(ExRobot));
        py_arm_preplanned_motion_ext!(PyExRobot<{0}>(ExRobot));

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

        impl ArmBehavior<0> for ExRobot {
            unimpl!(
                fn state(&mut self) -> RobotResult<ArmState<0>>;
            );
            unimpl!(
                fn set_load(&mut self, _load: LoadState) -> RobotResult<()>;
            );
            unimpl!(
                fn set_coord_description(&mut self, _desc: Desc) -> RobotResult<()>;
            );
        }

        impl ArmParam<0> for ExRobot {
            const DH: [[f64; 4]; 0] = [];
            const JOINT_MIN: [f64; 0] = [];
            const JOINT_MAX: [f64; 0] = [];
        }

        impl ArmPreplannedMotion<0> for ExRobot {
            #[rustfmt::skip]
            unimpl!(
                fn move_to(&mut self, _target: MotionType<0>, _speed: f64) -> RobotResult<()>;
                fn move_to_async(&mut self, _target: MotionType<0>, _speed: f64) -> RobotResult<()>;
                fn move_rel(&mut self, _target: MotionType<0>, _speed: f64) -> RobotResult<()>;
                fn move_rel_async(&mut self, _target: MotionType<0>, _speed: f64) -> RobotResult<()>;
                fn move_path(&mut self, _path: Vec<MotionType<0>>, _speed: f64) -> RobotResult<()>;
                fn control_with(&mut self, _control: ControlType<0>) -> RobotResult<()>;
            );
        }

        impl ArmPreplannedMotionExt<0> for ExRobot {
            unimpl!(
                fn move_path_prepare(&mut self, _path: Vec<MotionType<0>>) -> RobotResult<()>;
            );
            unimpl!(
                fn move_path_start(&mut self) -> RobotResult<()>;
            );
        }
    }
}
