//! End-effector and sensor contracts.
//!
//! These are not [`Robot`](crate::Robot) subtraits: a gripper or sensor may be
//! part of an arm driver (Franka's hand) or a device of its own, so a driver
//! implements them on whichever type owns the connection. Readings carry a
//! [`Header`](crate::Header) with the producer's clock and frame.

use roplat_msgs::{CameraInfo, ForceTorque, GripperCommand, GripperState, Imu, Stamped};

use crate::{RobotException, RobotResult};

/// A parallel or adaptive gripper driven by width, speed and force.
pub trait Gripper {
    fn read_gripper(&mut self) -> RobotResult<GripperState>;

    /// Moves to `command.width`; a positive `command.force` grasps. Returns
    /// when the motion ends; `GripperState::grasped` reports the outcome.
    fn move_gripper(&mut self, command: GripperCommand) -> RobotResult<()>;

    /// Calibrates the finger range, when the device needs it.
    fn home_gripper(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Stops the current motion or releases a held grasp force.
    fn stop_gripper(&mut self) -> RobotResult<()> {
        Err(RobotException::UnprocessableInstructionError(
            "stopping the gripper is not implemented for this driver".to_string(),
        ))
    }
}

/// A six-axis force/torque sensor; the wrench is in the sensor frame named by
/// the header.
pub trait ForceTorqueSensor {
    fn read_wrench(&mut self) -> RobotResult<ForceTorque>;

    /// Zeroes the current reading (tare), when the device supports it.
    fn bias_wrench(&mut self) -> RobotResult<()> {
        Err(RobotException::UnprocessableInstructionError(
            "biasing the wrench is not implemented for this driver".to_string(),
        ))
    }
}

/// An inertial measurement unit; absent fields are cleared in `Imu`'s mask.
pub trait ImuSensor {
    fn read_imu(&mut self) -> RobotResult<Stamped<Imu>>;
}

/// A camera with fixed intrinsics.
///
/// The frame type is the driver's until the shared image types land with the
/// data-stream work; the header carries the capture time and optical frame.
pub trait Camera {
    type Frame;

    fn camera_info(&self) -> CameraInfo;

    /// Blocks for the next frame.
    fn capture(&mut self) -> RobotResult<Stamped<Self::Frame>>;
}
