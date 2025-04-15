use std::ffi::{CString, c_char};

use crate::{ControlType, MotionType, RobotResult};

#[repr(C)]
pub struct CError {
    code: i32,
    message: CString,
}

impl<T> From<RobotResult<T>> for CError {
    fn from(result: RobotResult<T>) -> Self {
        match result {
            Ok(_) => CError {
                code: 0,
                message: CString::new("".to_string()).unwrap(),
            },
            Err(e) => {
                let message = CString::new(e.to_string()).unwrap();
                let mut c_message = [0; 256];
                message
                    .as_bytes()
                    .iter()
                    .enumerate()
                    .for_each(|(i, &c)| c_message[i] = c as c_char);
                CError { code: 1, message }
            }
        }
    }
}

#[repr(C)]
pub enum CMotionType {
    Joint,
    JointVel,
    CartesianEuler,
    CartesianVel,
    Position,
    PositionVel,
}

impl<const N: usize> Into<MotionType<N>> for (CMotionType, [f64; N]) {
    fn into(self) -> MotionType<N> {
        match self.0 {
            CMotionType::Joint => MotionType::Joint(self.1),
            CMotionType::JointVel => MotionType::JointVel(self.1),
            CMotionType::CartesianEuler => {
                MotionType::CartesianEuler(self.1[0..6].try_into().unwrap())
            }
            CMotionType::CartesianVel => MotionType::CartesianVel(self.1[0..6].try_into().unwrap()),
            CMotionType::Position => MotionType::Position(self.1[0..3].try_into().unwrap()),
            CMotionType::PositionVel => MotionType::PositionVel(self.1[0..3].try_into().unwrap()),
        }
    }
}
#[repr(C)]
pub enum CControlType {
    Zero,
    Torque,
}

impl<const N: usize> Into<ControlType<N>> for (CControlType, [f64; N]) {
    fn into(self) -> ControlType<N> {
        match self.0 {
            CControlType::Zero => ControlType::Zero,
            CControlType::Torque => ControlType::Torque(self.1),
        }
    }
}
