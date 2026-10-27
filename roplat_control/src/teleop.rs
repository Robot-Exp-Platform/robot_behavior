//! Teleoperation: operator input to a velocity command.
//!
//! [`axes_to_velocity`] maps one [`InputAxes`] sample (a 3D mouse or a
//! gamepad, axes in `[-1, 1]`) to `[vx, vy, vz, wx, wy, wz]`.
//! [`cartesian_teleop_control`] wraps it as a controller for
//! `CartesianVelocityControl` or `BaseVelocityControl`, adding what an
//! operator device needs on a real robot: a dead-man button, zero command when
//! input stops arriving, and an acceleration limit.

use std::time::Duration;

use roplat_behavior::msgs::InputAxes;

/// How operator input becomes a velocity command.
#[derive(Clone, Copy, Debug)]
pub struct TeleopMapping {
    /// Axis magnitude read as zero (stick drift); the rest of the range is
    /// rescaled so the command starts from zero at the edge of the deadzone.
    pub deadzone: f64,
    /// Speed at full deflection of axes 0–2 (m/s).
    pub linear_speed: f64,
    /// Speed at full deflection of axes 3–5 (rad/s).
    pub angular_speed: f64,
    /// Per-axis sign, to match how the device is mounted.
    pub signs: [f64; 6],
    /// Button that must be held for any motion (dead-man switch); `None`
    /// moves whenever the axes are deflected.
    pub enable_button: Option<u32>,
    /// With no new sample for this long, the command decays to zero.
    pub input_timeout: Duration,
    /// Largest change of the linear (m/s²) and angular (rad/s²) command per
    /// second; `f64::INFINITY` disables the limit.
    pub max_linear_accel: f64,
    pub max_angular_accel: f64,
}

impl Default for TeleopMapping {
    fn default() -> Self {
        Self {
            deadzone: 0.1,
            linear_speed: 0.1,
            angular_speed: 0.5,
            signs: [1.0; 6],
            enable_button: None,
            input_timeout: Duration::from_millis(200),
            max_linear_accel: 0.5,
            max_angular_accel: 2.5,
        }
    }
}

/// The velocity one sample asks for. Out-of-range axes are clamped to
/// `[-1, 1]`, a non-finite axis reads as zero, and an unpressed enable button
/// gives zero.
pub fn axes_to_velocity(input: &InputAxes, mapping: &TeleopMapping) -> [f64; 6] {
    if let Some(button) = mapping.enable_button
        && (button >= 32 || input.buttons & (1 << button) == 0)
    {
        return [0.0; 6];
    }
    let span = (1.0 - mapping.deadzone).max(f64::EPSILON);
    std::array::from_fn(|i| {
        let a = input.axes[i];
        let a = if a.is_finite() {
            a.clamp(-1.0, 1.0)
        } else {
            0.0
        };
        let shaped = (a.abs() - mapping.deadzone).max(0.0) / span * a.signum();
        let speed = if i < 3 {
            mapping.linear_speed
        } else {
            mapping.angular_speed
        };
        mapping.signs[i] * shaped * speed
    })
}

/// A velocity controller following the operator. `input` is polled once per
/// cycle and returns the newest sample, or `None` if none arrived since the
/// last poll. The command follows [`axes_to_velocity`] within the
/// acceleration limits, and returns to zero (within the same limits) after
/// `input_timeout` without samples. `done` reports `true` once `stop`
/// returns `true` and the command has reached zero.
pub fn cartesian_teleop_control<S, F, D>(
    mut input: F,
    mapping: TeleopMapping,
    mut stop: D,
) -> impl FnMut(S, Duration) -> ([f64; 6], bool)
where
    F: FnMut() -> Option<InputAxes>,
    D: FnMut() -> bool,
{
    let mut command = [0.0; 6];
    let mut target = [0.0; 6];
    let mut since_input = Duration::MAX;
    move |_state, period| {
        match input() {
            Some(sample) => {
                target = axes_to_velocity(&sample, &mapping);
                since_input = Duration::ZERO;
            }
            None => since_input = since_input.saturating_add(period),
        }
        let stopping = stop();
        if stopping || since_input > mapping.input_timeout {
            target = [0.0; 6];
        }
        let dt = period.as_secs_f64();
        for i in 0..6 {
            let accel = if i < 3 {
                mapping.max_linear_accel
            } else {
                mapping.max_angular_accel
            };
            let step = accel * dt;
            command[i] += (target[i] - command[i]).clamp(-step, step);
        }
        (command, stopping && command.iter().all(|v| *v == 0.0))
    }
}
