// PyO3 0.28 generates Clone-based FromPyObject even for Copy PyPose.
#[allow(clippy::clone_on_copy)]
pub mod types {
    use pyo3::{pyclass, pymethods};
    use roplat_behavior::Pose;

    #[derive(Debug, Clone)]
    #[pyclass(name = "Desc", from_py_object)]
    pub struct PyDesc {
        #[pyo3(get, set)]
        pub from: String,
        #[pyo3(get, set)]
        pub to: String,
    }

    #[pymethods]
    impl PyDesc {
        #[new]
        fn new(from: String, to: String) -> Self {
            PyDesc { from, to }
        }
    }

    #[derive(Debug, Clone, Copy)]
    #[pyclass(name = "Pose", from_py_object)]

    pub enum PyPose {
        #[pyo3(constructor = (_0, _1))]
        Euler([f64; 3], [f64; 3]),
        #[pyo3(constructor = (_0, _1))]
        Quat([f64; 3], [f64; 4]),
        #[pyo3(constructor = (_0))]
        Homo([f64; 16]),
        #[pyo3(constructor = (_0, _1, _2))]
        AxisAngle([f64; 3], [f64; 3], f64),
        #[pyo3(constructor = (_0))]
        Position([f64; 3]),
    }

    #[pymethods]
    impl PyPose {
        fn euler(&self) -> ([f64; 3], [f64; 3]) {
            Into::<Pose>::into(*self).euler()
        }

        fn quat(&self) -> ([f64; 3], [f64; 4]) {
            Into::<Pose>::into(*self).quat_array()
        }

        fn homo(&self) -> [f64; 16] {
            Into::<Pose>::into(*self).homo()
        }

        fn axis_angle(&self) -> ([f64; 3], [f64; 3], f64) {
            Into::<Pose>::into(*self).axis_angle()
        }

        fn position(&self) -> [f64; 3] {
            Into::<Pose>::into(*self).position()
        }
    }

    /// A pose has one representation since 0.9; Python receives it in the
    /// `Quat(position, xyzw)` form.
    impl From<Pose> for PyPose {
        fn from(value: Pose) -> Self {
            let (tran, rot) = value.quat_array();
            PyPose::Quat(tran, rot)
        }
    }

    impl From<PyPose> for Pose {
        fn from(value: PyPose) -> Self {
            match value {
                PyPose::Euler(tran, rot) => Pose::from_euler(tran, rot),
                PyPose::Quat(tran, rot) => Pose::from_quat_array(tran, rot),
                PyPose::Homo(pose) => Pose::from_homo(pose),
                PyPose::AxisAngle(tran, axis, angle) => Pose::from_axis_angle(tran, axis, angle),
                PyPose::Position(tran) => Pose::from_position(tran),
            }
        }
    }

    #[derive(Debug, Clone)]
    #[pyclass(name = "MotionType", from_py_object)]
    pub enum PyMotionType {
        #[pyo3(constructor = (_0))]
        Joint(Vec<f64>),
        #[pyo3(constructor = (_0))]
        JointVel(Vec<f64>),
        #[pyo3(constructor = (_0))]
        Cartesian(PyPose),
        #[pyo3(constructor = (_0))]
        CartesianVel(Vec<f64>),
        #[pyo3(constructor = (_0))]
        Position(Vec<f64>),
        #[pyo3(constructor = (_0))]
        PositionVel(Vec<f64>),
        #[pyo3(constructor = ())]
        Stop(),
    }
}
