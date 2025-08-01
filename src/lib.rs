#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(portable_simd)]

mod arm;
mod exception;
#[cfg(feature = "ffi")]
pub mod ffi;
mod load;
mod logger;
mod once;
mod params;
mod realtime;
mod robot;
mod types;
pub mod utils;

pub use arm::*;
pub use exception::*;
pub use load::*;
pub use once::*;
pub use params::*;
pub use realtime::*;
pub use robot::*;
pub use types::*;

pub mod behavior {
    pub use crate::arm::{
        ArmBehavior, ArmParam, ArmPreplannedMotion, ArmPreplannedMotionExt,
        ArmPreplannedMotionImpl, ArmRealtimeControl, ArmRealtimeControlExt, ArmStreamingHandle,
        ArmStreamingMotion, ArmStreamingMotionExt,
    };
    pub use crate::robot::RobotBehavior;
}

#[cfg(feature = "to_py")]
#[pyo3::pymodule]
mod robot_behavior {
    #[pymodule_export]
    use super::{LoadState, PyArmState, PyControlType, PyMotionType, PyPose};
}
