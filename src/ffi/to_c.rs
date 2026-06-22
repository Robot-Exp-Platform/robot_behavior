use crate::{Pose, RobotException, RobotResult};

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CStatusCode {
    Ok = 0,
    NullPointer = 1,
    Error = 2,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CStatus {
    pub code: CStatusCode,
}

impl CStatus {
    pub const OK: Self = Self { code: CStatusCode::Ok };
    pub const NULL_POINTER: Self = Self { code: CStatusCode::NullPointer };

    pub fn from_result(result: RobotResult<()>) -> Self {
        match result {
            Ok(()) => Self::OK,
            Err(_) => Self { code: CStatusCode::Error },
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CPoseKind {
    Euler = 0,
    Quat = 1,
    Homo = 2,
    AxisAngle = 3,
    Position = 4,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CPose {
    pub kind: CPoseKind,
    pub values: [f64; 16],
    pub len: usize,
}

impl From<Pose> for CPose {
    fn from(value: Pose) -> Self {
        match value {
            Pose::Euler(tran, rot) => {
                let mut values = [0.0; 16];
                values[..3].copy_from_slice(&tran);
                values[3..6].copy_from_slice(&rot);
                Self { kind: CPoseKind::Euler, values, len: 6 }
            }
            Pose::Quat(pose) => {
                let mut values = [0.0; 16];
                values[..3].copy_from_slice(pose.translation.vector.as_slice());
                values[3..7].copy_from_slice(pose.rotation.coords.as_slice());
                Self { kind: CPoseKind::Quat, values, len: 7 }
            }
            Pose::Homo(pose) => Self { kind: CPoseKind::Homo, values: pose, len: 16 },
            Pose::AxisAngle(tran, axis, angle) => {
                let mut values = [0.0; 16];
                values[..3].copy_from_slice(&tran);
                values[3..6].copy_from_slice(&axis);
                values[6] = angle;
                Self { kind: CPoseKind::AxisAngle, values, len: 7 }
            }
            Pose::Position(tran) => {
                let mut values = [0.0; 16];
                values[..3].copy_from_slice(&tran);
                Self { kind: CPoseKind::Position, values, len: 3 }
            }
        }
    }
}

impl TryFrom<CPose> for Pose {
    type Error = RobotException;

    fn try_from(value: CPose) -> Result<Self, Self::Error> {
        match value.kind {
            CPoseKind::Euler => Ok(Pose::Euler(
                value.values[..3].try_into().unwrap(),
                value.values[3..6].try_into().unwrap(),
            )),
            CPoseKind::Quat => Ok(Pose::from([
                value.values[0],
                value.values[1],
                value.values[2],
                value.values[3],
                value.values[4],
                value.values[5],
                value.values[6],
            ])),
            CPoseKind::Homo => Ok(Pose::Homo(value.values)),
            CPoseKind::AxisAngle => Ok(Pose::AxisAngle(
                value.values[..3].try_into().unwrap(),
                value.values[3..6].try_into().unwrap(),
                value.values[6],
            )),
            CPoseKind::Position => Ok(Pose::Position(value.values[..3].try_into().unwrap())),
        }
    }
}

#[macro_export]
macro_rules! c_robot_lifecycle_api {
    ($prefix:ident, $ty:ty) => {
        paste::paste! {
            #[unsafe(no_mangle)]
            pub unsafe extern "C" fn [<$prefix _init>](robot: *mut $ty) -> $crate::ffi::to_c::CStatus {
                let Some(robot) = (unsafe { robot.as_mut() }) else {
                    return $crate::ffi::to_c::CStatus::NULL_POINTER;
                };
                $crate::ffi::to_c::CStatus::from_result(<$ty as $crate::Robot>::init(robot))
            }

            #[unsafe(no_mangle)]
            pub unsafe extern "C" fn [<$prefix _enable>](robot: *mut $ty) -> $crate::ffi::to_c::CStatus {
                let Some(robot) = (unsafe { robot.as_mut() }) else {
                    return $crate::ffi::to_c::CStatus::NULL_POINTER;
                };
                $crate::ffi::to_c::CStatus::from_result(<$ty as $crate::Robot>::enable(robot))
            }

            #[unsafe(no_mangle)]
            pub unsafe extern "C" fn [<$prefix _disable>](robot: *mut $ty) -> $crate::ffi::to_c::CStatus {
                let Some(robot) = (unsafe { robot.as_mut() }) else {
                    return $crate::ffi::to_c::CStatus::NULL_POINTER;
                };
                $crate::ffi::to_c::CStatus::from_result(<$ty as $crate::Robot>::disable(robot))
            }

            #[unsafe(no_mangle)]
            pub unsafe extern "C" fn [<$prefix _shutdown>](robot: *mut $ty) -> $crate::ffi::to_c::CStatus {
                let Some(robot) = (unsafe { robot.as_mut() }) else {
                    return $crate::ffi::to_c::CStatus::NULL_POINTER;
                };
                $crate::ffi::to_c::CStatus::from_result(<$ty as $crate::Robot>::shutdown(robot))
            }

            #[unsafe(no_mangle)]
            pub unsafe extern "C" fn [<$prefix _reset>](robot: *mut $ty) -> $crate::ffi::to_c::CStatus {
                let Some(robot) = (unsafe { robot.as_mut() }) else {
                    return $crate::ffi::to_c::CStatus::NULL_POINTER;
                };
                $crate::ffi::to_c::CStatus::from_result(<$ty as $crate::Robot>::reset(robot))
            }

            #[unsafe(no_mangle)]
            pub unsafe extern "C" fn [<$prefix _stop>](robot: *mut $ty) -> $crate::ffi::to_c::CStatus {
                let Some(robot) = (unsafe { robot.as_mut() }) else {
                    return $crate::ffi::to_c::CStatus::NULL_POINTER;
                };
                $crate::ffi::to_c::CStatus::from_result(<$ty as $crate::Robot>::stop(robot))
            }

            #[unsafe(no_mangle)]
            pub unsafe extern "C" fn [<$prefix _emergency_stop>](
                robot: *mut $ty,
            ) -> $crate::ffi::to_c::CStatus {
                let Some(robot) = (unsafe { robot.as_mut() }) else {
                    return $crate::ffi::to_c::CStatus::NULL_POINTER;
                };
                $crate::ffi::to_c::CStatus::from_result(<$ty as $crate::Robot>::emergency_stop(robot))
            }
        }
    };
}

#[macro_export]
macro_rules! c_arm_motion_api {
    ($prefix:ident, $ty:ty, $dof:expr) => {
        paste::paste! {
            #[unsafe(no_mangle)]
            pub unsafe extern "C" fn [<$prefix _move_joint>](
                robot: *mut $ty,
                target: *const f64,
                len: usize,
            ) -> $crate::ffi::to_c::CStatus {
                let Some(robot) = (unsafe { robot.as_mut() }) else {
                    return $crate::ffi::to_c::CStatus::NULL_POINTER;
                };
                if target.is_null() || len != $dof {
                    return $crate::ffi::to_c::CStatus { code: $crate::ffi::to_c::CStatusCode::Error };
                }
                let target = unsafe { std::slice::from_raw_parts(target, len) };
                let Ok(target) = <[f64; $dof]>::try_from(target) else {
                    return $crate::ffi::to_c::CStatus { code: $crate::ffi::to_c::CStatusCode::Error };
                };
                $crate::ffi::to_c::CStatus::from_result(
                    <$ty as $crate::MoveTo<$crate::JointSpace<$dof>>>::move_to(robot, target),
                )
            }

            #[unsafe(no_mangle)]
            pub unsafe extern "C" fn [<$prefix _move_flange>](
                robot: *mut $ty,
                target: $crate::ffi::to_c::CPose,
            ) -> $crate::ffi::to_c::CStatus {
                let Some(robot) = (unsafe { robot.as_mut() }) else {
                    return $crate::ffi::to_c::CStatus::NULL_POINTER;
                };
                let Ok(target) = $crate::Pose::try_from(target) else {
                    return $crate::ffi::to_c::CStatus { code: $crate::ffi::to_c::CStatusCode::Error };
                };
                $crate::ffi::to_c::CStatus::from_result(
                    <$ty as $crate::MoveTo<$crate::FlangeSpace>>::move_to(robot, target),
                )
            }
        }
    };
}
