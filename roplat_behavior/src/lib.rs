//! Typed robot capability contracts.
//!
//! Drivers implement [`Robot`] plus the motion ([`MoveTo`], [`MoveTraj`]) and
//! control ([`ControlWith`], [`AsyncControlWith`]) traits for the spaces and
//! channels they support; applications program against those traits. Sample and
//! geometry types come from [`msgs`] (`roplat_msgs`). Kinematics, controllers
//! and trajectories live in `roplat_kinematics`, `roplat_control` and
//! `roplat_trajectory`. Feature `roplat` adds `Node` / `Rhythm` adapters.

#![feature(async_trait_bounds)]

#[cfg(feature = "conformance")]
pub mod conformance;
mod exception;
#[cfg(feature = "ffi")]
pub mod ffi;

mod physics_engine;
mod renderer;
mod robot;
#[cfg(feature = "roplat")]
pub mod roplat;
mod simulation;
pub mod utils;
mod world;

use std::env;
use std::path::PathBuf;

pub use exception::*;
pub use physics_engine::*;
pub use renderer::*;
pub use robot::*;
/// The shared message vocabulary, re-exported.
pub use roplat_msgs as msgs;
pub use roplat_msgs::{Accel, FrameId, Header, Quaternion, Stamp, Stamped, Twist, Vector3, Wrench};
pub use simulation::*;
pub use utils::*;
pub use world::*;

#[cfg(feature = "to_py")]
pub use ffi::to_py::{
    PyArmState, PyJointSample, PyJointState, PyLoadState, PySpatialSample, PySpatialState,
};
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
        Arm, ArmState, ArmTorqueControl, AsyncControl, AsyncControlCallback, BalanceControl,
        BasePoseSpace, BaseState, BaseVelocityControl, BaseVelocitySpace, Camera, CartesianLimits,
        CartesianPoseControl, CartesianVelocityControl, CenterOfMassSpace, ContactState, Control,
        ControlObservation, ControlObserver, ControlSpace, ControlStep, Coord, CoriolisInput,
        CoriolisInputSpace, DynamicsModel, EndEffectorState, EndPoint, EndSpace, FlangeSpace,
        FootSpace, ForceTorqueSensor, ForwardKinematics, GaitCommand, GaitSpace, GravityInput,
        GravityInputSpace, Gripper, HandSpace, Humanoid, HumanoidState, ImuSensor, Inertial,
        InverseKinematics, JacobianModel, JacobianSpace, JointLimits, JointPositionControl,
        JointSample, JointSpace, JointState, JointTorqueSpace, JointVelocityControl, Joints,
        LoadState, MassMatrixSpace, MobileBase, MobileBaseState, Motion, MotionFile, MotionLimits,
        MotionSpace, Pose, Quadruped, QuadrupedState, Relative, Robot, RobotDescription, SpaceMap,
        SpatialSample, StateView, TcpSpace, TorqueControl, TypedSpaceMap, WholeBodyJointSpace,
        WholeBodyTorqueSpace, WholeBodyVelocitySpace,
    };

    pub use crate::physics_engine::{AddSearchPath, PhysicsEngine};
    pub use crate::renderer::{AttachFrom, Renderer};
    pub use crate::simulation::{
        ModelFormat, ModelSelectionError, ModelSource, SelectedSimulationModel, SimulationBackend,
        SimulationModel, select_simulation_model,
    };
    pub use crate::world::{AddCollision, AddRobot, AddVisual, EntityBuilder};
}

pub mod driver {
    pub use crate::behavior::*;
    pub use crate::robot::{AsyncControlWith, ControlWith, MoveTo, MoveTraj};
}

#[cfg(feature = "to_py")]
#[pyo3::pymodule]
mod roplat_behavior {
    #[pymodule_export]
    use super::{
        PyArmState, PyDesc, PyJointSample, PyJointState, PyLoadState, PyMotionType, PyPose,
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
