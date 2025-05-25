fn main() {}

#[cfg(feature = "to_cxx")]
mod to_cxx {
    struct TestRobot;
    struct CTestRobot(TestRobot);

    #[cxx::bridge]
    mod ffi {
        extern "Rust" {
            type CTestRobot;
            fn create() -> CTestRobot;
            fn version(&self) -> String;
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
    }

    impl CTestRobot {
        fn create() -> Self {
            CTestRobot(TestRobot)
        }
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
