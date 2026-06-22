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

#[cfg(feature = "to_py")]
pub use ffi::to_py::{PyArmState, PyJointSample, PyJointState, PySpatialSample, PySpatialState};
#[cfg(feature = "to_py")]
pub use robot::types::{PyDesc, PyMotionType, PyPose};

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
        Arm, ArmDynamics, ArmForwardKinematics, ArmInverseKinematics, ArmState, ArmTorqueControl,
        BalanceControl, BasePoseSpace, BaseState, BaseVelocityControl, BaseVelocitySpace,
        CartesianPoseControl, CartesianVelocityControl, CenterOfMassSpace, ContactState, Control,
        ControlSpace, Coord, CoriolisInput, CoriolisInputSpace, DhParam, DynamicsModel,
        EndEffectorState, EndPoint, EndSpace, FlangeSpace, FootSpace, ForwardKinematics,
        GaitCommand, GaitSpace, GravityInput, GravityInputSpace, HandSpace, Humanoid,
        HumanoidState, Inertial, InverseKinematics, JacobianModel, JacobianSpace,
        JointPositionControl, JointSample, JointSpace, JointState, JointTorqueSpace,
        JointVelocityControl, Joints, LoadState, MassMatrixSpace, MobileBase, MobileBaseState,
        Motion, MotionFile, MotionSpace, Pose, Quadruped, QuadrupedState, Relative, Robot,
        RobotDescription, SpaceMap, SpatialSample, StateView, TcpSpace, TorqueControl,
        TypedSpaceMap, WholeBodyJointSpace, WholeBodyTorqueSpace, WholeBodyVelocitySpace,
    };

    pub use crate::physics_engine::{AddSearchPath, PhysicsEngine};
    pub use crate::renderer::{AttachFrom, Renderer};
    pub use crate::world::{AddCollision, AddRobot, AddVisual, EntityBuilder};
}

pub mod driver {
    pub use crate::behavior::*;
    pub use crate::robot::{ControlWith, MoveTo, MoveTraj};
}

pub mod controller {
    pub use crate::utils::controller::dynamics::*;
    pub use crate::utils::controller::impedance::*;
    pub use crate::utils::controller::pid::*;
}

#[cfg(feature = "to_py")]
#[pyo3::pymodule]
mod robot_behavior {
    #[pymodule_export]
    use super::{
        LoadState, PyArmState, PyDesc, PyJointSample, PyJointState, PyMotionType, PyPose,
        PySpatialSample, PySpatialState,
    };
}

pub fn roplat_data_dir() -> Option<PathBuf> {
    {
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
    .map(|path| path.join("roplat").join("assets"))
}
