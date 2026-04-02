#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

mod exception;
#[cfg(feature = "ffi")]
pub mod ffi;

mod physics_engine;
mod renderer;
mod robot;
pub mod utils;
mod world;

use std::env;
use std::path::PathBuf;

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
        Arm, ArmDOF, ArmForwardKinematics, ArmImpedance, ArmInverseKinematics, ArmParam,
        ArmPreplannedMotion, ArmPreplannedMotionExt, ArmPreplannedPath, ArmRealtimeControl,
        ArmRealtimeControlExt, ArmStreamingHandle, ArmStreamingMotion, ArmStreamingMotionExt,
        CartesianImpedance, CartesianImpedanceHandle, JointImpedance, JointImpedanceHandle, Robot,
        RobotFile,
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

pub fn roplat_data_dir() -> Option<PathBuf> {
    #[cfg(target_os = "windows")]
    {
        env::var_os("LOCALAPPDATA")
            .map(PathBuf::from)
            .or_else(|| env::var_os("USERPROFILE").map(PathBuf::from))
    }

    #[cfg(target_os = "macos")]
    {
        env::var_os("HOME")
            .map(PathBuf::from)
            .map(|home| home.join("Library").join("Application Support"))
    }

    #[cfg(all(not(target_os = "windows"), not(target_os = "macos")))]
    {
        env::var_os("HOME")
            .map(PathBuf::from)
            .map(|home| home.join(".local").join("share"))
    }
}
