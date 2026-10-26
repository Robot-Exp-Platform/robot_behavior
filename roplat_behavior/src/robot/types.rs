use serde::{Deserialize, Serialize};

pub use roplat_msgs::Pose;

#[derive(Debug, Default, Serialize, Deserialize, PartialEq, Clone, Copy)]
/// The reference frame in which a Cartesian command is interpreted.
///
/// Parsed from a string via [`From<&str>`](Coord::from): `"OCS"` → [`Coord::OCS`],
/// `"Shot"` → [`Coord::Relative`], `"Inertial"` → [`Coord::Inertial`], and any
/// other string is decoded as a JSON [`Pose`] into [`Coord::Other`].
pub enum Coord {
    /// Object / base coordinate system (the default world frame).
    #[default]
    OCS,
    /// Relative to the robot's current pose (incremental motion).
    Relative,
    /// A fixed inertial frame.
    Inertial,
    /// An arbitrary user-supplied frame given as a [`Pose`].
    Other(Pose),
}

impl From<&str> for Coord {
    fn from(s: &str) -> Self {
        match s {
            "OCS" => Coord::OCS,
            "Shot" => Coord::Relative,
            "Inertial" => Coord::Inertial,
            _ => Coord::Other(serde_json::from_str(s).unwrap()),
        }
    }
}

// PyO3 0.28 generates Clone-based FromPyObject even for Copy PyPose.
#[cfg(feature = "to_py")]
#[allow(clippy::clone_on_copy)]
mod to_py {

    use super::*;
    use pyo3::{pyclass, pymethods};

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

#[cfg(feature = "to_py")]
pub use to_py::*;

#[cfg(feature = "to_cxx")]
#[allow(unused_imports)]
mod to_cxx {
    use super::*;

    #[cxx::bridge]
    mod to_cxx_bridge {
        extern "Rust" {
            type CxxPose;
            #[Self = "CxxPose"]
            fn Euler(tran: [f64; 3], rot: [f64; 3]) -> Box<CxxPose>;
            #[Self = "CxxPose"]
            fn Quat(tran: [f64; 3], rot: [f64; 4]) -> Box<CxxPose>;
            #[Self = "CxxPose"]
            fn Homo(pose: [f64; 16]) -> Box<CxxPose>;
            #[Self = "CxxPose"]
            fn AxisAngle(tran: [f64; 3], axis: [f64; 3], angle: f64) -> Box<CxxPose>;
            #[Self = "CxxPose"]
            fn Position(tran: [f64; 3]) -> Box<CxxPose>;
            fn to_vec(&self) -> Vec<f64>;
            #[Self = "CxxPose"]
            fn from_vec(vec: Vec<f64>) -> Box<CxxPose>;
        }
    }

    pub use to_cxx_bridge::*;

    struct CxxPose(Pose);

    impl CxxPose {
        #[allow(non_snake_case)]
        fn Euler(tran: [f64; 3], rot: [f64; 3]) -> Box<Self> {
            Box::new(CxxPose(Pose::from_euler(tran, rot)))
        }
        #[allow(non_snake_case)]
        fn Quat(tran: [f64; 3], rot: [f64; 4]) -> Box<Self> {
            Box::new(CxxPose(Pose::from_quat_array(tran, rot)))
        }
        #[allow(non_snake_case)]
        fn Homo(pose: [f64; 16]) -> Box<Self> {
            Box::new(CxxPose(Pose::from_homo(pose)))
        }
        #[allow(non_snake_case)]
        fn AxisAngle(tran: [f64; 3], axis: [f64; 3], angle: f64) -> Box<Self> {
            Box::new(CxxPose(Pose::from_axis_angle(tran, axis, angle)))
        }
        #[allow(non_snake_case)]
        fn Position(tran: [f64; 3]) -> Box<Self> {
            Box::new(CxxPose(Pose::from_position(tran)))
        }

        fn to_vec(&self) -> Vec<f64> {
            self.0.into()
        }
        fn from_vec(vec: Vec<f64>) -> Box<Self> {
            Box::new(CxxPose(vec.into()))
        }
    }
}

#[cfg(feature = "to_cxx")]
#[allow(unused_imports)]
pub use to_cxx::*;
