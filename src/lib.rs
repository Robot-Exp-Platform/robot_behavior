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
mod physics_engine;
mod realtime;
mod renderer;
mod robot;
mod types;
pub mod utils;
mod world;

pub use arm::*;
pub use exception::*;
pub use load::*;
pub use once::*;
pub use params::*;
pub use physics_engine::*;
pub use realtime::*;
pub use renderer::*;
pub use robot::*;
pub use types::*;

pub mod behavior {
    pub use crate::arm::{
        Arm, ArmDOF, ArmParam, ArmPreplannedMotion, ArmPreplannedMotionExt,
        ArmPreplannedMotionImpl, ArmRealtimeControl, ArmRealtimeControlExt, ArmStreamingHandle,
        ArmStreamingMotion, ArmStreamingMotionExt,
    };
    pub use crate::robot::{Robot, RobotFile};

    pub use crate::physics_engine::{PhysicsEngine, PhysicsEngineRobot, RobotBuilder};
    pub use crate::renderer::{AttachFrom, Renderer, RendererRobot};
}

#[cfg(feature = "to_py")]
#[pyo3::pymodule]
mod robot_behavior {
    #[pymodule_export]
    use super::{LoadState, PyArmState, PyControlType, PyMotionType, PyPose};
}
