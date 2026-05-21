use std::time::Duration;

use crate::{ArmState, ControlType};

pub fn joint_space_impedance_torque<const N: usize>(
    target: [f64; N],
    stiffness: [f64; N],
    damping: [f64; N],
    state: &ArmState<N>,
) -> [f64; N] {
    let joint = state.measured.joint.unwrap_or(target);
    let joint_vel = state.measured.joint_vel.unwrap_or([0.0; N]);
    let mut torque = [0.0; N];

    for i in 0..N {
        let position_error = target[i] - joint[i];
        torque[i] = stiffness[i] * position_error - damping[i] * joint_vel[i];
    }

    torque
}

pub fn joint_impedance_control<const N: usize>(
    target: [f64; N],
    stiffness: [f64; N],
    damping: [f64; N],
) -> impl FnMut(ArmState<N>, Duration) -> (ControlType<N>, bool) + Send + 'static {
    move |state, _duration| {
        let torque = joint_space_impedance_torque(target, stiffness, damping, &state);
        (ControlType::Torque(torque), false)
    }
}