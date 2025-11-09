#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(portable_simd)]

mod exception;
#[cfg(feature = "ffi")]
pub mod ffi;

mod physics_engine;
mod renderer;
mod robot;
pub mod utils;
mod world;

pub use exception::*;
pub use physics_engine::*;
pub use renderer::*;
pub use robot::*;
pub use utils::*;
pub use world::*;

pub const ROPLAT_ASCII: &str = r#"
   #####     #####     ##### 
  #     #   #     #   #     #
  #     #   #     #   #     #
  #     #   #     #   #     # 
   #####    #     #    ##### 
  #   #     #     #   #      
  #    #    #     #   #      
  #     #    #####    #      
"#;

pub mod behavior {
    pub use crate::robot::{
        Arm, ArmDOF, ArmForwardKinematics, ArmInverseKinematics, ArmParam, ArmPreplannedMotion,
        ArmPreplannedMotionExt, ArmRealtimeControl, ArmRealtimeControlExt, ArmStreamingHandle,
        ArmStreamingMotion, ArmStreamingMotionExt, Robot, RobotFile,
    };

    pub use crate::physics_engine::{AddSearchPath, PhysicsEngine};
    pub use crate::renderer::{AttachFrom, Renderer};
    pub use crate::world::{AddCollision, AddRobot, AddVisual, EntityBuilder};
}

#[cfg(feature = "to_py")]
#[pyo3::pymodule]
mod robot_behavior {
    #[pymodule_export]
    use super::{LoadState, PyArmState, PyControlType, PyMotionType, PyPose};
}
