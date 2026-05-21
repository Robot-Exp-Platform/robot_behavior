use std::time::Duration;

use crate::{ArmState, ControlType};

pub fn joint_traj_pid_control<const N: usize>(
    traj: Vec<[f64; N]>,
    kp: [f64; N],
    ki: [f64; N],
    kd: [f64; N],
) -> impl FnMut(ArmState<N>, Duration) -> (ControlType<N>, bool) + Send + 'static {
    let mut step = 0usize;
    let mut integral = [0.0; N];

    move |state, duration| {
        if traj.is_empty() {
            return (ControlType::Zero, true);
        }
        if duration == Duration::ZERO && step == 0 {
            return (ControlType::Torque([0.0; N]), false);
        }

        let last_index = traj.len() - 1;
        let index = step.min(last_index);
        let q_ref = traj[index];
        let q_next = traj[(index + 1).min(last_index)];
        let q = state.measured.joint.unwrap_or(q_ref);
        let dq = state.measured.joint_vel.unwrap_or([0.0; N]);

        let dt = duration.as_secs_f64();
        let mut dq_ref = [0.0; N];
        if dt > 0.0 {
            for i in 0..N {
                dq_ref[i] = (q_next[i] - q_ref[i]) / dt;
            }
        }

        let mut torque = [0.0; N];
        for i in 0..N {
            let position_error = q_ref[i] - q[i];
            integral[i] += position_error * dt;
            torque[i] = kp[i] * position_error + ki[i] * integral[i] + kd[i] * (dq_ref[i] - dq[i]);
        }

        if duration > Duration::ZERO {
            step += 1;
        }

        (ControlType::Torque(torque), step >= traj.len())
    }
}