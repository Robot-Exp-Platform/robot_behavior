use std::ops::{Div, Mul};

use nalgebra as na;
use serde::{Deserialize, Serialize};
use serde_with::serde_as;

use crate::{
    ArmState,
    utils::{combine_array, homo_to_isometry},
};

#[derive(Debug, Default, Serialize, Deserialize, PartialEq, Clone, Copy)]
pub enum Coord {
    #[default]
    OCS,
    Relative,
    Inertial,
    Other(Pose),
}

#[derive(Debug, Serialize, Deserialize, PartialEq, Clone, Copy)]
pub enum Pose {
    Euler([f64; 3], [f64; 3]),
    Quat(na::Isometry3<f64>),
    Homo([f64; 16]),
    AxisAngle([f64; 3], [f64; 3], f64),
    Position([f64; 3]),
}

#[serde_as]
#[derive(Debug, Serialize, Deserialize, Clone, Copy, Default)]
pub enum ControlType<const N: usize> {
    #[default]
    Zero,
    Torque(#[serde_as(as = "[_; N]")] [f64; N]),
}

#[serde_as]
#[derive(Debug, Serialize, Deserialize, Clone, Copy, Default)]
pub enum MotionType<const N: usize> {
    Joint(#[serde_as(as = "[_; N]")] [f64; N]),
    JointVel(#[serde_as(as = "[_; N]")] [f64; N]),
    Cartesian(Pose),
    CartesianVel([f64; 6]),
    Position([f64; 3]),
    PositionVel([f64; 3]),
    #[default]
    Stop,
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

impl Default for Pose {
    fn default() -> Self {
        Pose::Quat(na::Isometry3::identity())
    }
}

impl Pose {
    pub fn euler(&self) -> ([f64; 3], [f64; 3]) {
        match self {
            Pose::Euler(tran, rot) => (*tran, *rot),
            Pose::Quat(pose) => (
                pose.translation.vector.into(),
                pose.rotation.euler_angles().into(),
            ),
            Pose::Homo(pose) => {
                let iso = homo_to_isometry(pose);
                (
                    iso.translation.vector.into(),
                    iso.rotation.euler_angles().into(),
                )
            }
            Pose::AxisAngle(tran, axis, angle) => {
                let rotation = na::UnitQuaternion::from_axis_angle(
                    &na::Unit::new_normalize(na::Vector3::from(*axis)),
                    *angle,
                );
                (*tran, rotation.euler_angles().into())
            }
            Pose::Position(tran) => (*tran, [0.0, 0.0, 0.0]),
        }
    }

    pub fn quat(&self) -> na::Isometry3<f64> {
        match self {
            Pose::Euler(tran, rot) => na::Isometry3::<f64>::from_parts(
                na::Translation3::new(tran[0], tran[1], tran[2]),
                na::UnitQuaternion::from_euler_angles(rot[0], rot[1], rot[2]),
            ),
            Pose::Homo(pose) => homo_to_isometry(pose),
            Pose::Quat(pose) => *pose,
            Pose::AxisAngle(tran, axis, angle) => na::Isometry3::from_parts(
                na::Translation3::new(tran[0], tran[1], tran[2]),
                na::UnitQuaternion::from_axis_angle(
                    &na::Unit::new_normalize(na::Vector3::from(*axis)),
                    *angle,
                ),
            ),
            Pose::Position(tran) => na::Isometry3::from_parts(
                na::Translation3::new(tran[0], tran[1], tran[2]),
                na::UnitQuaternion::identity(),
            ),
        }
    }

    pub fn quat_array(&self) -> ([f64; 3], [f64; 4]) {
        let quat = self.quat();
        (quat.translation.vector.into(), quat.rotation.coords.into())
    }

    pub fn homo(&self) -> [f64; 16] {
        match self {
            Pose::Euler(tran, rot) => na::IsometryMatrix3::from_parts(
                na::Translation3::new(tran[0], tran[1], tran[2]),
                na::Rotation3::from_euler_angles(rot[0], rot[1], rot[2]),
            )
            .to_homogeneous()
            .as_slice()
            .try_into()
            .unwrap(),
            Pose::Homo(pose) => *pose,
            Pose::Quat(pose) => pose.to_homogeneous().as_slice().try_into().unwrap(),
            Pose::AxisAngle(tran, axis, angle) => na::IsometryMatrix3::from_parts(
                na::Translation3::new(tran[0], tran[1], tran[2]),
                na::Rotation3::from_axis_angle(
                    &na::Unit::new_normalize(na::Vector3::from(*axis)),
                    *angle,
                ),
            )
            .to_homogeneous()
            .as_slice()
            .try_into()
            .unwrap(),
            Pose::Position(tran) => {
                let mut pose = [0.; 16];
                (pose[0], pose[5], pose[10], pose[15]) = (1., 1., 1., 1.);
                (pose[12], pose[13], pose[14]) = (tran[0], tran[1], tran[2]);
                pose
            }
        }
    }

    pub fn axis_angle(&self) -> ([f64; 3], [f64; 3], f64) {
        match self {
            Pose::Euler(tran, rot) => {
                let rotation = na::UnitQuaternion::from_euler_angles(rot[0], rot[1], rot[2]);
                let (axis, angle) = rotation.axis_angle().unwrap();
                (*tran, axis.into_inner().into(), angle)
            }
            Pose::Quat(pose) => {
                let (axis, angle) = pose.rotation.axis_angle().unwrap();
                (
                    pose.translation.vector.into(),
                    axis.into_inner().into(),
                    angle,
                )
            }
            Pose::Homo(pose) => {
                let iso = homo_to_isometry(pose);
                let (axis, angle) = iso.rotation.axis_angle().unwrap();
                (
                    iso.translation.vector.into(),
                    axis.into_inner().into(),
                    angle,
                )
            }
            Pose::AxisAngle(tran, axis, angle) => (*tran, *axis, *angle),
            Pose::Position(tran) => (*tran, [1.0, 0.0, 0.0], 0.0),
        }
    }

    pub fn position(&self) -> [f64; 3] {
        match self {
            Pose::Euler(tran, _) => *tran,
            Pose::Quat(pose) => pose.translation.vector.into(),
            Pose::Homo(pose) => pose[12..15].try_into().unwrap(),
            Pose::AxisAngle(tran, _, _) => *tran,
            Pose::Position(tran) => *tran,
        }
    }
}

impl From<[f64; 3]> for Pose {
    fn from(value: [f64; 3]) -> Self {
        Pose::Position(value)
    }
}

impl From<[f64; 6]> for Pose {
    fn from(value: [f64; 6]) -> Self {
        Pose::Euler(
            [value[0], value[1], value[2]],
            [value[3], value[4], value[5]],
        )
    }
}

impl From<([f64; 3], [f64; 3])> for Pose {
    fn from(value: ([f64; 3], [f64; 3])) -> Self {
        Pose::Euler(value.0, value.1)
    }
}

impl From<([f64; 3], [f64; 4])> for Pose {
    fn from(value: ([f64; 3], [f64; 4])) -> Self {
        Pose::Quat(na::Isometry3::from_parts(
            na::Translation3::new(value.0[0], value.0[1], value.0[2]),
            na::UnitQuaternion::from_quaternion(na::Quaternion::from(value.1)),
        ))
    }
}

impl From<([f64; 3], [f64; 3], f64)> for Pose {
    fn from(value: ([f64; 3], [f64; 3], f64)) -> Self {
        Pose::AxisAngle(value.0, value.1, value.2)
    }
}

impl From<[f64; 7]> for Pose {
    fn from(value: [f64; 7]) -> Self {
        let tran = [value[0], value[1], value[2]];
        let rot = [value[3], value[4], value[5], value[6]];
        let rot_norm = rot.iter().map(|x| x.powi(2)).sum::<f64>();
        if rot_norm == 0. {
            Pose::Position(tran)
        } else if rot_norm == 1. {
            Pose::Quat(na::Isometry3::from_parts(
                na::Translation3::new(tran[0], tran[1], tran[2]),
                na::UnitQuaternion::from_quaternion(na::Quaternion::from(rot)),
            ))
        } else {
            Pose::AxisAngle(
                [value[0], value[1], value[2]],
                [value[3], value[4], value[5]],
                value[6],
            )
        }
    }
}

impl From<na::Isometry3<f64>> for Pose {
    fn from(value: na::Isometry3<f64>) -> Self {
        Pose::Quat(value)
    }
}

impl From<[f64; 16]> for Pose {
    fn from(value: [f64; 16]) -> Self {
        Pose::Homo(value)
    }
}

impl From<&[f64]> for Pose {
    fn from(value: &[f64]) -> Self {
        value.to_vec().into()
    }
}

impl From<Vec<f64>> for Pose {
    fn from(value: Vec<f64>) -> Self {
        match value.len() {
            3 => Pose::Position(value.try_into().unwrap()),
            6 => Pose::Euler(
                [value[0], value[1], value[2]],
                [value[3], value[4], value[5]],
            ),
            7 => Pose::Quat(na::Isometry3::from_parts(
                na::Translation3::new(value[0], value[1], value[2]),
                na::UnitQuaternion::from_quaternion(na::Quaternion::from([
                    value[3], value[4], value[5], value[6],
                ])),
            )),
            16 => Pose::Homo(value.try_into().unwrap()),
            _ => panic!("Invalid Vec length for Pose"),
        }
    }
}

impl From<Pose> for [f64; 3] {
    fn from(value: Pose) -> Self {
        value.position()
    }
}

impl From<Pose> for ([f64; 3], [f64; 4]) {
    fn from(value: Pose) -> Self {
        let pose = value.quat();
        let rotation = pose.rotation;
        (
            pose.translation.into(),
            [rotation.i, rotation.j, rotation.k, rotation.w],
        )
    }
}

impl From<Pose> for [f64; 6] {
    fn from(value: Pose) -> Self {
        let euler = value.euler();
        combine_array(&euler.0, &euler.1)
    }
}

impl From<Pose> for [f64; 16] {
    fn from(value: Pose) -> Self {
        value.homo()
    }
}

impl From<Pose> for Vec<f64> {
    fn from(value: Pose) -> Self {
        match value {
            Pose::Euler(tran, rot) => {
                let mut vec = Vec::with_capacity(6);
                vec.extend_from_slice(&tran);
                vec.extend_from_slice(&rot);
                vec
            }
            Pose::Quat(pose) => {
                let mut vec = Vec::with_capacity(7);
                vec.extend_from_slice(&pose.translation.vector.as_slice());
                vec.extend_from_slice(&pose.rotation.vector().as_slice());
                vec
            }
            Pose::Homo(pose) => pose.as_slice().to_vec(),
            Pose::AxisAngle(tran, axis, angle) => {
                let mut vec = Vec::with_capacity(7);
                vec.extend_from_slice(&tran);
                vec.extend_from_slice(&axis);
                vec.push(angle);
                vec
            }
            Pose::Position(tran) => tran.as_slice().to_vec(),
        }
    }
}

impl Div for Pose {
    type Output = f64;
    fn div(self, rhs: Self) -> Self::Output {
        self.position()
            .iter()
            .zip(rhs.position().iter())
            .map(|(a, b)| (a - b).exp2())
            .sum()
    }
}

impl Div for &Pose {
    type Output = f64;
    fn div(self, rhs: Self) -> Self::Output {
        self.position()
            .iter()
            .zip(rhs.position().iter())
            .map(|(a, b)| (a - b).exp2())
            .sum()
    }
}

impl Mul for Pose {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self::Output {
        match (self, rhs) {
            (Pose::Homo(pose1), Pose::Homo(pose2)) => Pose::Homo(
                (na::Matrix4::from_column_slice(&pose1) * na::Matrix4::from_column_slice(&pose2))
                    .as_slice()
                    .try_into()
                    .unwrap(),
            ),
            _ => Pose::Quat(self.quat() * rhs.quat()),
        }
    }
}

impl<const N: usize> MotionType<N> {
    pub fn with_coord(self, coord: &Coord, state: &ArmState<N>) -> MotionType<N> {
        match (self, coord) {
            (_, Coord::OCS) => self,
            (MotionType::Joint(joint), _) => {
                let mut result = [0.0; N];
                for i in 0..N {
                    result[i] = joint[i] + state.joint.unwrap()[i]
                }
                MotionType::Joint(result)
            }
            (MotionType::Cartesian(pose), Coord::Relative) => {
                MotionType::Cartesian(state.pose_o_to_ee.unwrap() * pose)
            }
            (MotionType::Cartesian(pose), Coord::Inertial) => {
                MotionType::Cartesian(Pose::Position(state.pose_o_to_ee.unwrap().position()) * pose)
            }
            _ => self,
        }
    }
}

#[cfg(feature = "to_py")]
mod to_py {

    use super::*;
    use pyo3::{pyclass, pymethods};

    #[derive(Debug, Clone)]
    #[pyclass(name = "Desc")]
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
    #[pyclass(name = "Pose")]

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

    impl From<Pose> for PyPose {
        fn from(value: Pose) -> Self {
            match value {
                Pose::Euler(tran, rot) => PyPose::Euler(tran, rot),
                Pose::Quat(pose) => {
                    PyPose::Quat(pose.translation.vector.into(), pose.rotation.coords.into())
                }
                Pose::Homo(pose) => PyPose::Homo(pose),
                Pose::AxisAngle(tran, axis, angle) => PyPose::AxisAngle(tran, axis, angle),
                Pose::Position(tran) => PyPose::Position(tran),
            }
        }
    }

    impl From<PyPose> for Pose {
        fn from(value: PyPose) -> Self {
            match value {
                PyPose::Euler(tran, rot) => Pose::Euler(tran, rot),
                PyPose::Quat(tran, rot) => Pose::Quat(na::Isometry3::from_parts(
                    na::Translation3::new(tran[0], tran[1], tran[2]),
                    na::UnitQuaternion::from_quaternion(na::Quaternion::from(rot)),
                )),
                PyPose::Homo(pose) => Pose::Homo(pose),
                PyPose::AxisAngle(tran, axis, angle) => Pose::AxisAngle(tran, axis, angle),
                PyPose::Position(tran) => Pose::Position(tran),
            }
        }
    }

    #[derive(Debug, Clone)]
    #[pyclass(name = "MotionType")]
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

    impl<const N: usize> From<MotionType<N>> for PyMotionType {
        fn from(value: MotionType<N>) -> Self {
            match value {
                MotionType::Joint(joint) => PyMotionType::Joint(joint.to_vec()),
                MotionType::JointVel(joint_vel) => PyMotionType::JointVel(joint_vel.to_vec()),
                MotionType::Cartesian(pose) => PyMotionType::Cartesian(pose.into()),
                MotionType::CartesianVel(cartesian_vel) => {
                    PyMotionType::CartesianVel(cartesian_vel.to_vec())
                }
                MotionType::Position(position) => PyMotionType::Position(position.to_vec()),
                MotionType::PositionVel(position_vel) => {
                    PyMotionType::PositionVel(position_vel.to_vec())
                }
                MotionType::Stop => PyMotionType::Stop(),
            }
        }
    }

    impl<const N: usize> TryFrom<PyMotionType> for MotionType<N> {
        type Error = Vec<f64>;
        fn try_from(value: PyMotionType) -> Result<Self, Self::Error> {
            Ok(match value {
                PyMotionType::Joint(joint) => MotionType::Joint(joint.try_into()?),
                PyMotionType::JointVel(joint_vel) => MotionType::JointVel(joint_vel.try_into()?),
                PyMotionType::Cartesian(pose) => MotionType::Cartesian(pose.into()),
                PyMotionType::CartesianVel(cartesian_vel) => {
                    MotionType::CartesianVel(cartesian_vel.try_into()?)
                }
                PyMotionType::Position(position) => MotionType::Position(position.try_into()?),
                PyMotionType::PositionVel(position_vel) => {
                    MotionType::PositionVel(position_vel.try_into()?)
                }
                PyMotionType::Stop() => MotionType::Stop,
            })
        }
    }

    #[derive(Debug, Clone)]
    #[pyclass(name = "ControlType")]
    pub enum PyControlType {
        #[pyo3(constructor = ())]
        Zero(),
        #[pyo3(constructor = (_0))]
        Torque(Vec<f64>),
    }

    impl<const N: usize> From<ControlType<N>> for PyControlType {
        fn from(value: ControlType<N>) -> Self {
            match value {
                ControlType::Zero => PyControlType::Zero(),
                ControlType::Torque(torque) => PyControlType::Torque(torque.to_vec()),
            }
        }
    }

    impl<const N: usize> TryFrom<PyControlType> for ControlType<N> {
        type Error = Vec<f64>;
        fn try_from(value: PyControlType) -> Result<Self, Self::Error> {
            Ok(match value {
                PyControlType::Zero() => ControlType::Zero,
                PyControlType::Torque(torque) => ControlType::Torque(torque.try_into()?),
            })
        }
    }
}

#[cfg(feature = "to_py")]
pub use to_py::*;

#[cfg(feature = "to_cxx")]
mod to_cxx {
    use super::*;

    #[cxx::bridge]
    mod to_cxx_bridge {
        pub struct CxxMotionType {
            pub mode: CxxMotionTypeMode,
            pub values: Vec<f64>,
        }
        pub enum CxxMotionTypeMode {
            Joint,
            JointVel,
            Cartesian,
            CartesianVel,
            Position,
            PositionVel,
        }
        pub struct CxxControlType {
            pub mode: CxxControlTypeMode,
            pub values: Vec<f64>,
        }
        pub enum CxxControlTypeMode {
            Zero,
            Torque,
        }
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
            Box::new(CxxPose(Pose::Euler(tran, rot)))
        }
        #[allow(non_snake_case)]
        fn Quat(tran: [f64; 3], rot: [f64; 4]) -> Box<Self> {
            Box::new(CxxPose(Pose::Quat(na::Isometry3::from_parts(
                na::Translation3::new(tran[0], tran[1], tran[2]),
                na::UnitQuaternion::from_quaternion(na::Quaternion::from(rot)),
            ))))
        }
        #[allow(non_snake_case)]
        fn Homo(pose: [f64; 16]) -> Box<Self> {
            Box::new(CxxPose(Pose::Homo(pose)))
        }
        #[allow(non_snake_case)]
        fn AxisAngle(tran: [f64; 3], axis: [f64; 3], angle: f64) -> Box<Self> {
            Box::new(CxxPose(Pose::AxisAngle(tran, axis, angle)))
        }
        #[allow(non_snake_case)]
        fn Position(tran: [f64; 3]) -> Box<Self> {
            Box::new(CxxPose(Pose::Position(tran)))
        }

        fn to_vec(&self) -> Vec<f64> {
            self.0.into()
        }
        fn from_vec(vec: Vec<f64>) -> Box<Self> {
            Box::new(CxxPose(vec.into()))
        }
    }

    impl<const N: usize> From<CxxMotionType> for MotionType<N> {
        fn from(cxx: CxxMotionType) -> Self {
            match cxx.mode {
                CxxMotionTypeMode::Joint => MotionType::Joint(cxx.values.try_into().unwrap()),
                CxxMotionTypeMode::JointVel => MotionType::JointVel(cxx.values.try_into().unwrap()),
                CxxMotionTypeMode::Cartesian => {
                    MotionType::Cartesian(cxx.values.try_into().unwrap())
                }
                CxxMotionTypeMode::CartesianVel => {
                    MotionType::CartesianVel(cxx.values.try_into().unwrap())
                }
                CxxMotionTypeMode::Position => MotionType::Position(cxx.values.try_into().unwrap()),
                CxxMotionTypeMode::PositionVel => {
                    MotionType::PositionVel(cxx.values.try_into().unwrap())
                }
                _ => panic!("Invalid mode for MotionType"),
            }
        }
    }

    impl<const N: usize> From<CxxControlType> for ControlType<N> {
        fn from(cxx: CxxControlType) -> Self {
            match cxx.mode {
                CxxControlTypeMode::Zero => ControlType::Zero,
                CxxControlTypeMode::Torque => ControlType::Torque(cxx.values.try_into().unwrap()),
                _ => panic!("Invalid mode for ControlType"),
            }
        }
    }
}

#[cfg(feature = "to_cxx")]
pub use to_cxx::*;
