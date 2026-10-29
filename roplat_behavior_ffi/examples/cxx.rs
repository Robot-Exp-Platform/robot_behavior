//! Minimal current C++ lifecycle bridge. Compile with `--features cxx`.
//! Producing a standalone C++ application additionally requires generated glue.
fn main() {}

#[cfg(feature = "cxx")]
mod to_cxx {
    use roplat_behavior::{Robot, RobotResult};
    pub struct ExampleRobot;
    impl Robot for ExampleRobot {
        type State = ();
        const CONTROL_PERIOD: std::time::Duration = std::time::Duration::from_millis(1);
        fn version() -> String {
            "mock".into()
        }
        fn read_state(&mut self) -> RobotResult<()> {
            Ok(())
        }
    }
    impl ExampleRobot {
        roplat_behavior_ffi::cxx_robot_api!(ExampleRobot);
    }
    #[cxx::bridge]
    mod ffi {
        extern "Rust" {
            type ExampleRobot;
            #[Self = "ExampleRobot"]
            fn version() -> String;
            fn init(&mut self) -> Result<()>;
            fn enable(&mut self) -> Result<()>;
            fn disable(&mut self) -> Result<()>;
            fn shutdown(&mut self) -> Result<()>;
            fn reset(&mut self) -> Result<()>;
            fn stop(&mut self) -> Result<()>;
            fn emergency_stop(&mut self) -> Result<()>;
            fn waiting_for_finish(&mut self) -> Result<()>;
            fn is_moving(&mut self) -> Result<bool>;
        }
    }
}
