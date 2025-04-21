use nalgebra as na;
use serde::{Deserialize, Serialize};
use serde_with::serde_as;

use crate::utils::homo_to_isometry;

#[derive(Debug, Serialize, Deserialize, PartialEq, Clone, Copy)]
pub enum Pose {
    Euler([f64; 3], [f64; 3]),
    Quat(na::Isometry3<f64>),
    Homo([f64; 16]),
    AxisAngle([f64; 3], [f64; 3], f64),
    Position([f64; 3]),
}

pub enum ControlType<const N: usize> {
    Zero,
    Torque([f64; N]),
}

#[serde_as]
#[derive(Serialize, Deserialize, Clone, Copy)]
pub enum MotionType<const N: usize> {
    Joint(#[serde_as(as = "[_; N]")] [f64; N]),
    JointVel(#[serde_as(as = "[_; N]")] [f64; N]),
    Cartesian(Pose),
    CartesianVel([f64; 6]),
    Position([f64; 3]),
    PositionVel([f64; 3]),
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

impl Default for Pose {
    fn default() -> Self {
        Pose::Quat(na::Isometry3::identity())
    }
}

#[cfg(feature = "to_py")]
use pyo3::{Bound, FromPyObject, IntoPyObject, IntoPyObjectRef, PyAny, PyErr};

#[cfg(feature = "to_py")]
#[derive(IntoPyObject, IntoPyObjectRef, FromPyObject)]
enum PyPose {
    Euler([f64; 3], [f64; 3]),
    Quat([f64; 3], [f64; 4]),
    Homo([f64; 16]),
    AxisAngle([f64; 3], [f64; 3], f64),
    Position([f64; 3]),
}

#[cfg(feature = "to_py")]
impl From<Pose> for PyPose {
    fn from(value: Pose) -> Self {
        match value {
            Pose::Euler(tran, rot) => PyPose::Euler(tran, rot),
            Pose::Quat(tran) => {
                PyPose::Quat(tran.translation.vector.into(), tran.rotation.coords.into())
            }
            Pose::Homo(pose) => PyPose::Homo(pose),
            Pose::AxisAngle(tran, axis, angle) => PyPose::AxisAngle(tran, axis, angle),
            Pose::Position(tran) => PyPose::Position(tran),
        }
    }
}

#[cfg(feature = "to_py")]
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

#[cfg(feature = "to_py")]
impl<'py> IntoPyObject<'py> for Pose {
    type Target = PyAny;
    type Output = Bound<'py, Self::Target>;
    type Error = PyErr;

    fn into_pyobject(self, py: pyo3::Python<'py>) -> Result<Self::Output, Self::Error> {
        Into::<PyPose>::into(self).into_pyobject(py)
    }
}

#[cfg(feature = "to_py")]
impl<'py> IntoPyObject<'py> for &Pose {
    type Target = PyAny;
    type Output = Bound<'py, Self::Target>;
    type Error = PyErr;

    fn into_pyobject(self, py: pyo3::Python<'py>) -> Result<Self::Output, Self::Error> {
        Into::<PyPose>::into(*self).into_pyobject(py)
    }
}

#[cfg(feature = "to_py")]
impl<'py> FromPyObject<'py> for Pose {
    fn extract_bound(ob: &Bound<'py, PyAny>) -> pyo3::PyResult<Self> {
        Ok(PyPose::extract_bound(ob)?.into())
    }
}
