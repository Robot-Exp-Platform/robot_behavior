use std::time::Duration;

use crate::{BaseState, JointState, utils::path_generate::constant_joint_target_fn};

/// Builds a joint-space PD torque controller toward one fixed joint target.
///
/// The returned closure observes [`JointState<N>`] and returns a torque-like
/// `[f64; N]` command plus a completion flag. It is suitable for
/// `TorqueControl<N>` or `TorqueControl<N>` when the driver interprets
/// the command as joint effort.
///
/// For motion to a fixed position through the driver's own position servo,
/// prefer `MoveTo<JointSpace<N>>`; this helper is for realtime torque / force
/// control loops.
pub fn joint_pd_control<const N: usize>(
    target: [f64; N],
    kp: [f64; N],
    kd: [f64; N],
) -> impl FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static {
    joint_pd_tracking_control(constant_joint_target_fn(target), kp, kd)
}

/// Builds a joint-space PD controller around a dynamic target source.
///
/// `target_fn` is called once per control period with the same joint state and
/// elapsed cycle duration as the controller. This makes it possible to reuse
/// sampled trajectories, online path generators, or user-defined target
/// closures without introducing a controller struct.
///
/// Control law per joint:
///
/// `tau = kp * (q_ref - q) - kd * dq`
pub fn joint_pd_tracking_control<F, const N: usize>(
    mut target_fn: F,
    kp: [f64; N],
    kd: [f64; N],
) -> impl FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static
where
    F: FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static,
{
    move |state, duration| {
        let (target, done) = target_fn(state, duration);
        let q = state.meas.q.unwrap_or(target);
        let dq = state.meas.dq.unwrap_or([0.0; N]);
        let mut torque = [0.0; N];

        for i in 0..N {
            torque[i] = kp[i] * (target[i] - q[i]) - kd[i] * dq[i];
        }

        (torque, done)
    }
}

/// Builds a joint-space PD controller that follows an already sampled
/// trajectory.
///
/// Each non-zero-duration tick advances by one sample. The reference velocity
/// is estimated by finite-differencing the current and next trajectory sample,
/// so dense, evenly sampled trajectories work best.
pub fn joint_traj_pd_control<const N: usize>(
    traj: Vec<[f64; N]>,
    kp: [f64; N],
    kd: [f64; N],
) -> impl FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static {
    let mut step = 0usize;

    move |state, duration| {
        if traj.is_empty() {
            return ([0.0; N], true);
        }
        if duration == Duration::ZERO && step == 0 {
            return ([0.0; N], false);
        }

        let last = traj.len() - 1;
        let index = step.min(last);
        let q_ref = traj[index];
        let q_next = traj[(index + 1).min(last)];
        let q = state.meas.q.unwrap_or(q_ref);
        let dq = state.meas.dq.unwrap_or([0.0; N]);

        let dt = duration.as_secs_f64();
        let mut dq_ref = [0.0; N];
        if dt > 0.0 {
            for i in 0..N {
                dq_ref[i] = (q_next[i] - q_ref[i]) / dt;
            }
        }

        let mut torque = [0.0; N];
        for i in 0..N {
            torque[i] = kp[i] * (q_ref[i] - q[i]) + kd[i] * (dq_ref[i] - dq[i]);
        }

        if duration > Duration::ZERO {
            step += 1;
        }

        (torque, step >= traj.len())
    }
}

/// Builds a joint-space PID torque controller toward one fixed joint target.
///
/// The integral term is kept inside the returned closure. Dropping the closure
/// resets the accumulated integral state. This helper intentionally does not
/// clamp or anti-windup the integral term; drivers or applications with strict
/// torque limits should wrap or tune it accordingly.
pub fn joint_pid_control<const N: usize>(
    target: [f64; N],
    kp: [f64; N],
    ki: [f64; N],
    kd: [f64; N],
) -> impl FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static {
    joint_pid_tracking_control(constant_joint_target_fn(target), kp, ki, kd)
}

/// Builds a joint-space PID controller around a dynamic target source.
///
/// `target_fn` may be a closure returned by `utils::path_generate`, a custom
/// online planner, or a handle-backed source. It returns `(target, done)`; the
/// controller forwards `done` unchanged.
///
/// Control law per joint:
///
/// `tau = kp * e + ki * integral(e) - kd * dq`
pub fn joint_pid_tracking_control<F, const N: usize>(
    mut target_fn: F,
    kp: [f64; N],
    ki: [f64; N],
    kd: [f64; N],
) -> impl FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static
where
    F: FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static,
{
    let mut integral = [0.0; N];

    move |state, duration| {
        let (target, done) = target_fn(state, duration);
        let q = state.meas.q.unwrap_or(target);
        let dq = state.meas.dq.unwrap_or([0.0; N]);
        let dt = duration.as_secs_f64();
        let mut torque = [0.0; N];

        for i in 0..N {
            let error = target[i] - q[i];
            integral[i] += error * dt;
            torque[i] = kp[i] * error + ki[i] * integral[i] - kd[i] * dq[i];
        }

        (torque, done)
    }
}

/// Builds a joint-space PID controller for an already sampled trajectory.
///
/// The controller consumes one trajectory sample per non-zero-duration tick and
/// estimates `dq_ref` from adjacent samples. This is the trajectory variant to
/// use when the application already owns the sampled path and wants controller
/// tracking rather than a fixed setpoint controller.
pub fn joint_traj_pid_control<const N: usize>(
    traj: Vec<[f64; N]>,
    kp: [f64; N],
    ki: [f64; N],
    kd: [f64; N],
) -> impl FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static {
    let mut step = 0usize;
    let mut integral = [0.0; N];

    move |state, duration| {
        if traj.is_empty() {
            return ([0.0; N], true);
        }
        if duration == Duration::ZERO && step == 0 {
            return ([0.0; N], false);
        }

        let last = traj.len() - 1;
        let index = step.min(last);
        let q_ref = traj[index];
        let q_next = traj[(index + 1).min(last)];
        let q = state.meas.q.unwrap_or(q_ref);
        let dq = state.meas.dq.unwrap_or([0.0; N]);

        let dt = duration.as_secs_f64();
        let mut dq_ref = [0.0; N];
        if dt > 0.0 {
            for i in 0..N {
                dq_ref[i] = (q_next[i] - q_ref[i]) / dt;
            }
        }

        let mut torque = [0.0; N];
        for i in 0..N {
            let error = q_ref[i] - q[i];
            integral[i] += error * dt;
            torque[i] = kp[i] * error + ki[i] * integral[i] + kd[i] * (dq_ref[i] - dq[i]);
        }

        if duration > Duration::ZERO {
            step += 1;
        }

        (torque, step >= traj.len())
    }
}

/// Builds a PID controller for base spatial velocity.
///
/// The observed state is [`BaseState`], and the command is a six-dimensional
/// spatial velocity-like vector `[vx, vy, vz, wx, wy, wz]`. Use this with
/// `BaseVelocityControl` for mobile bases or floating-base abstractions.
pub fn base_velocity_pid_control(
    target: [f64; 6],
    kp: [f64; 6],
    ki: [f64; 6],
    kd: [f64; 6],
) -> impl FnMut(BaseState, Duration) -> ([f64; 6], bool) + Send + 'static {
    base_velocity_pid_tracking_control(move |_state, _duration| (target, false), kp, ki, kd)
}

/// Builds a PID base-velocity controller around a dynamic velocity target.
///
/// Unlike the joint PID helper, the derivative term is estimated from target
/// error differences because base state currently exposes measured spatial
/// velocity but not a measured base acceleration sample.
pub fn base_velocity_pid_tracking_control<F>(
    mut target_fn: F,
    kp: [f64; 6],
    ki: [f64; 6],
    kd: [f64; 6],
) -> impl FnMut(BaseState, Duration) -> ([f64; 6], bool) + Send + 'static
where
    F: FnMut(BaseState, Duration) -> ([f64; 6], bool) + Send + 'static,
{
    let mut integral = [0.0; 6];
    let mut last_error = [0.0; 6];

    move |state, duration| {
        let (target, done) = target_fn(state, duration);
        let velocity = state.meas.vel.unwrap_or([0.0; 6]);
        let dt = duration.as_secs_f64();
        let mut command = [0.0; 6];

        for i in 0..6 {
            let error = target[i] - velocity[i];
            integral[i] += error * dt;
            let derivative = if dt > 0.0 {
                (error - last_error[i]) / dt
            } else {
                0.0
            };
            command[i] = kp[i] * error + ki[i] * integral[i] + kd[i] * derivative;
            last_error[i] = error;
        }

        (command, done)
    }
}
