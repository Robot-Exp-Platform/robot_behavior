//! Minimal current C++ lifecycle bridge. Compile with `--features to_cxx`.
//! Producing a standalone C++ application additionally requires generated glue.
fn main() {}

#[cfg(feature = "to_cxx")]
mod to_cxx {
    use robot_behavior::{Robot, RobotResult};
    pub struct ExampleRobot;
    impl Robot for ExampleRobot {
        type State = ();
        const CONTROL_PERIOD: f64 = 0.001;
        fn version() -> String {
            "mock".into()
        }
        fn read_state(&mut self) -> RobotResult<()> {
            Ok(())
        }
    }
    impl ExampleRobot {
        robot_behavior::cxx_robot_api!(ExampleRobot);
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
