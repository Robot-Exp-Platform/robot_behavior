//! Minimal current Python wrapper. Compile with `--features to_py`.
//! This is a mock type, not a device connection or Python extension package.
fn main() {}

#[cfg(feature = "to_py")]
mod to_py {
    use pyo3::types::PyModuleMethods;
    use robot_behavior::{Robot, RobotResult, py_robot, py_robot_wrapper};

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
    py_robot_wrapper!(PyExampleRobot(ExampleRobot));
    py_robot!(PyExampleRobot(ExampleRobot));

    #[pyo3::pymodule]
    fn example_robot(m: &pyo3::Bound<'_, pyo3::types::PyModule>) -> pyo3::PyResult<()> {
        m.add_class::<PyExampleRobot>()?;
        Ok(())
    }
}
