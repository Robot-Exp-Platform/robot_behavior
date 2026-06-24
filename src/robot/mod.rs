pub mod arm;
pub mod category;
pub mod control;
pub mod dh;
pub mod endpoint;
pub mod joint;
pub mod kinematics_dynamics;
pub mod load;
pub mod model;
pub mod motion;
pub mod observe;
pub mod spaces;
pub mod state;
pub mod types;

pub use arm::{Arm, ArmState};
pub use category::{Humanoid, MobileBase, Quadruped};
pub use control::{
    ArmTorqueControl, BalanceControl, BaseVelocityControl, CartesianPoseControl,
    CartesianVelocityControl, Control, ControlSpace, ControlWith, JointPositionControl,
    JointVelocityControl, TorqueControl,
};
pub use dh::DhParam;
pub use endpoint::EndPoint;
pub use joint::Joints;
pub use kinematics_dynamics::{
    AnalyticFamily, ArmDynamics, ArmForwardKinematics, ArmInverseKinematics, ArmKineCache,
    CommonStop, IKMethod, Iso3, JMat, JVec, Jaco, Joint, JointType, Link, Twist, Wrench,
};
pub use load::LoadState;
pub use model::{
    CoriolisInput, DynamicsModel, ForwardKinematics, GravityInput, InverseKinematics,
    JacobianModel, SpaceMap, TypedSpaceMap,
};
pub use motion::{Motion, MotionFile, MotionSpace, MoveTo, MoveTraj};
pub use observe::{ControlObservation, ControlObserver};
pub use spaces::{
    BasePoseSpace, BaseVelocitySpace, CenterOfMassSpace, CoriolisInputSpace, EndSpace, FlangeSpace,
    FootSpace, GaitCommand, GaitSpace, GravityInputSpace, HandSpace, Inertial, JacobianSpace,
    JointSpace, JointTorqueSpace, MassMatrixSpace, Relative, TcpSpace, WholeBodyJointSpace,
    WholeBodyTorqueSpace, WholeBodyVelocitySpace,
};
pub use state::{
    BaseState, ContactSample, ContactState, EndEffectorState, HumanoidState, JointSample,
    JointState, MobileBaseState, QuadrupedState, SpatialSample, StateView,
};
pub use types::{Coord, Pose};

use crate::RobotResult;

/// # Robot 鈥?the single core behavior every device shares
///
/// `Robot` is the one root trait of the whole capability stack: it defines the
/// per-tick measured [`State`](Robot::State) plus a universally meaningful
/// lifecycle. Every capability trait has `Robot` as its super-trait,
/// so anything controllable can also be brought up, read and stopped.
///
/// Only [`State`](Robot::State), [`version`](Robot::version) and
/// [`read_state`](Robot::read_state) are required; the remaining lifecycle hooks
/// have safe no-op defaults so a minimal driver (or a simulator) overrides just
/// what it actually needs.
pub trait Robot {
    /// One frame of measured robot state.
    type State;

    const CONTROL_PERIOD: f64;

    /// Driver / firmware version string.
    fn version() -> String;

    /// Read the latest measured state.
    fn read_state(&mut self) -> RobotResult<Self::State>;

    /// One-time bring-up (connect, allocate handles, 鈥?.
    fn init(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Tear down and release resources.
    fn shutdown(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Power-on / engage actuators.
    fn enable(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Disengage actuators.
    fn disable(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Reset the robot to a known state.
    fn reset(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Whether the robot is currently executing a motion.
    fn is_moving(&mut self) -> RobotResult<bool> {
        Ok(false)
    }

    /// Block until the current action finishes.
    fn waiting_for_finish(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Stop the current action (recoverable).
    fn stop(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Pause the current action.
    fn pause(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Resume a paused action.
    fn resume(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Emergency stop (latched). Defaults to a plain [`stop`](Robot::stop).
    fn emergency_stop(&mut self) -> RobotResult<()> {
        self.stop()
    }

    /// Clear a latched emergency stop.
    fn clear_emergency_stop(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

/// A robot description, used for dynamic dispatch and configuration.
pub trait RobotDescription {
    /// A robot with a URDF file.
    /// This path is relative to the [`roplat_data_dir`](roplat_data_dir) directory.
    const URDF: Option<&'static str> = None;
}
