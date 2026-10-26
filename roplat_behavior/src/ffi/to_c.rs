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

/// A pose has one representation since 0.9; C receives the `Quat` form
/// `[x, y, z, qx, qy, qz, qw]`.
impl From<Pose> for CPose {
    fn from(value: Pose) -> Self {
        let (tran, rot) = value.quat_array();
        let mut values = [0.0; 16];
        values[..3].copy_from_slice(&tran);
        values[3..7].copy_from_slice(&rot);
        Self { kind: CPoseKind::Quat, values, len: 7 }
    }
}

impl TryFrom<CPose> for Pose {
    type Error = RobotException;

    fn try_from(value: CPose) -> Result<Self, Self::Error> {
        let v = value.values;
        Ok(match value.kind {
            CPoseKind::Euler => Pose::from_euler([v[0], v[1], v[2]], [v[3], v[4], v[5]]),
            CPoseKind::Quat => Pose::from_quat_array([v[0], v[1], v[2]], [v[3], v[4], v[5], v[6]]),
            CPoseKind::Homo => Pose::from_homo(v),
            CPoseKind::AxisAngle => {
                Pose::from_axis_angle([v[0], v[1], v[2]], [v[3], v[4], v[5]], v[6])
            }
            CPoseKind::Position => Pose::from_position([v[0], v[1], v[2]]),
        })
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
