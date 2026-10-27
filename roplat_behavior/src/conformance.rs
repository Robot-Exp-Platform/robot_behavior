//! Contract conformance checks for driver authors.
//!
//! Each check drives a device through its contract and reports the first
//! violation as a [`Violation`] naming what was expected. `advance` is how the
//! caller lets time pass between samples: step a simulator, let a mock
//! transport emit the next message, or wait for a real device.
//!
//! The checks cover what the contracts promise for every implementation:
//! readings of one sensor share a clock and a frame, sequence numbers rise
//! and stamps do not go back, values are finite, an IMU orientation is a unit
//! quaternion, a tared static sensor reads zero, and a gripper reaches the
//! widths it is sent. Device-specific behavior (ranges, noise, timing) stays
//! in the driver's own tests.

use std::fmt;

use roplat_msgs::{GripperCommand, Header, Imu};

use crate::{ForceTorqueSensor, Gripper, ImuSensor, RobotException};

/// A broken contract promise.
#[derive(Debug)]
pub struct Violation {
    /// The check that failed, e.g. `"force_torque: seq"`.
    pub check: &'static str,
    pub detail: String,
}

impl fmt::Display for Violation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.check, self.detail)
    }
}

impl std::error::Error for Violation {}

fn violation(check: &'static str, detail: impl Into<String>) -> Violation {
    Violation { check, detail: detail.into() }
}

fn device(check: &'static str, error: RobotException) -> Violation {
    violation(check, format!("the device returned an error: {error}"))
}

/// Headers of consecutive readings from one stream: one clock, one frame,
/// rising sequence numbers, stamps that do not go back.
pub fn check_header_stream(check: &'static str, headers: &[Header]) -> Result<(), Violation> {
    for pair in headers.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        if a.stamp.clock != b.stamp.clock {
            return Err(violation(
                check,
                format!("clock changed from {} to {}", a.stamp.clock, b.stamp.clock),
            ));
        }
        if a.frame != b.frame {
            return Err(violation(
                check,
                format!("frame changed from {:?} to {:?}", a.frame, b.frame),
            ));
        }
        if b.seq <= a.seq {
            return Err(violation(
                check,
                format!("seq did not rise: {} then {}", a.seq, b.seq),
            ));
        }
        if b.stamp.nanos < a.stamp.nanos {
            return Err(violation(
                check,
                format!(
                    "stamp went back: {} then {} ns",
                    a.stamp.nanos, b.stamp.nanos
                ),
            ));
        }
    }
    Ok(())
}

/// Reads `samples` wrenches with `advance` between them. With
/// `static_load`, the sensor's load must not change while the check runs:
/// if the device supports [`bias_wrench`](ForceTorqueSensor::bias_wrench),
/// the next reading must then be zero within `tolerance` on every axis.
pub fn check_force_torque_sensor<S: ForceTorqueSensor>(
    sensor: &mut S,
    mut advance: impl FnMut(),
    samples: usize,
    static_load: bool,
    tolerance: f64,
) -> Result<(), Violation> {
    const CHECK: &str = "force_torque";
    let mut headers = Vec::with_capacity(samples);
    for _ in 0..samples {
        advance();
        let reading = sensor.read_wrench().map_err(|e| device(CHECK, e))?;
        if reading.value.to_array().iter().any(|v| !v.is_finite()) {
            return Err(violation(
                CHECK,
                format!("non-finite wrench {:?}", reading.value),
            ));
        }
        headers.push(reading.header);
    }
    check_header_stream(CHECK, &headers)?;

    if static_load {
        match sensor.bias_wrench() {
            Err(RobotException::UnprocessableInstructionError(_)) => {}
            Err(e) => return Err(device(CHECK, e)),
            Ok(()) => {
                advance();
                let tared = sensor.read_wrench().map_err(|e| device(CHECK, e))?.value;
                if tared.to_array().iter().any(|v| v.abs() > tolerance) {
                    return Err(violation(
                        "force_torque: bias",
                        format!("a tared static load reads {:?}", tared.to_array()),
                    ));
                }
            }
        }
    }
    Ok(())
}

/// Reads `samples` IMU messages with `advance` between them: header stream,
/// finite present fields, unit orientation, and a mask that never sets bits
/// the message does not define.
pub fn check_imu<S: ImuSensor>(
    sensor: &mut S,
    mut advance: impl FnMut(),
    samples: usize,
) -> Result<(), Violation> {
    const CHECK: &str = "imu";
    let known = Imu::ORIENTATION | Imu::ANGULAR_VELOCITY | Imu::LINEAR_ACCELERATION;
    let mut headers = Vec::with_capacity(samples);
    for _ in 0..samples {
        advance();
        let reading = sensor.read_imu().map_err(|e| device(CHECK, e))?;
        let imu = reading.value;
        if imu.valid_mask() & !known != 0 {
            return Err(violation(
                CHECK,
                format!("unknown mask bits {:#x}", imu.valid_mask()),
            ));
        }
        if let Some(q) = imu.orientation() {
            let norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
            if !norm.is_finite() || (norm - 1.0).abs() > 1e-6 {
                return Err(violation(CHECK, format!("orientation norm {norm}")));
            }
        }
        for v in [imu.angular_velocity(), imu.linear_acceleration()]
            .into_iter()
            .flatten()
        {
            if ![v.x, v.y, v.z].iter().all(|c| c.is_finite()) {
                return Err(violation(CHECK, format!("non-finite vector {v:?}")));
            }
        }
        headers.push(reading.header);
    }
    check_header_stream(CHECK, &headers)
}

/// Opens the gripper to each of `widths` (scaled by the reported
/// `max_width`, so `1.0` is fully open) with zero force, and requires the
/// reported width within `tolerance` (m) after each move. `advance` runs
/// after each move, for devices whose state reaches the reader later.
pub fn check_gripper<G: Gripper>(
    gripper: &mut G,
    mut advance: impl FnMut(),
    widths: &[f64],
    speed: f64,
    tolerance: f64,
) -> Result<(), Violation> {
    const CHECK: &str = "gripper";
    let state = gripper.read_gripper().map_err(|e| device(CHECK, e))?;
    if !(state.max_width.is_finite() && state.max_width > 0.0) {
        return Err(violation(CHECK, format!("max_width {}", state.max_width)));
    }
    if !(0.0..=state.max_width + tolerance).contains(&state.width) {
        return Err(violation(
            CHECK,
            format!("width {} outside [0, {}]", state.width, state.max_width),
        ));
    }
    for &fraction in widths {
        let width = fraction.clamp(0.0, 1.0) * state.max_width;
        gripper
            .move_gripper(GripperCommand { width, speed, force: 0.0 })
            .map_err(|e| device(CHECK, e))?;
        advance();
        let reached = gripper.read_gripper().map_err(|e| device(CHECK, e))?.width;
        if (reached - width).abs() > tolerance {
            return Err(violation(
                "gripper: width",
                format!("sent {width} m, reads {reached} m"),
            ));
        }
    }
    Ok(())
}
