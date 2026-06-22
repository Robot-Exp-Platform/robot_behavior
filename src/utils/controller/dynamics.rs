use std::time::Duration;

use crate::{DynamicsModel, JVec, JointState};

/// Builds a gravity-compensation torque controller.
///
/// The returned closure observes [`JointState<N>`] and returns the model's
/// gravity torque for the measured joint position. It is ready to pass into
/// `control_with::<TorqueControl<N>, _>(...)` or a whole-body torque
/// channel.
///
/// If the state has no measured `q`, the controller uses `[0.; N]`. If the
/// model query fails, it returns zero torque for that cycle.
pub fn gravity_compensation_control<M, const N: usize>(
    model: M,
    gravity: [f64; 3],
) -> impl FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static
where
    M: DynamicsModel<N> + Send + 'static,
{
    move |state, _duration| {
        let q = state.meas.q.unwrap_or([0.; N]);
        let torque = model.gravity(q, gravity).unwrap_or([0.; N]);
        (torque, false)
    }
}

/// Builds a computed-torque controller around a fixed joint target.
///
/// This is the fixed-reference convenience wrapper over
/// [`computed_torque_tracking_control`]. It is useful for simple model-based
/// regulation when the desired joint position, velocity and acceleration are
/// known constants.
///
/// Command law:
///
/// `tau = M(q) * (ddq_ref + kp * e + kd * de) + C(q, dq) + G(q)`
///
/// The closure is intentionally plain `FnMut`, matching the rest of the
/// controller module, so callers can pass it directly to `control_with`.
pub fn computed_torque_control<M, const N: usize>(
    model: M,
    target: [f64; N],
    target_vel: [f64; N],
    target_acc: [f64; N],
    kp: [f64; N],
    kd: [f64; N],
    gravity: [f64; 3],
) -> impl FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static
where
    M: DynamicsModel<N> + Send + 'static,
{
    computed_torque_tracking_control(
        model,
        move |_state, _duration| (target, target_vel, target_acc, false),
        kp,
        kd,
        gravity,
    )
}

/// Builds a computed-torque controller around a dynamic reference source.
///
/// `target_fn` returns `(q_ref, dq_ref, ddq_ref, done)`. The controller augments
/// the desired acceleration with PD feedback, multiplies by the joint-space
/// mass matrix and adds Coriolis / centrifugal and gravity terms from the
/// supplied [`DynamicsModel`].
///
/// If `model.mass(q)` fails, the controller returns zero torque for that cycle
/// and forwards the `done` flag. Coriolis and gravity failures are treated as
/// zero terms, which keeps the closure usable with partial model
/// implementations during driver bring-up.
pub fn computed_torque_tracking_control<M, F, const N: usize>(
    model: M,
    mut target_fn: F,
    kp: [f64; N],
    kd: [f64; N],
    gravity: [f64; 3],
) -> impl FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static
where
    M: DynamicsModel<N> + Send + 'static,
    F: FnMut(JointState<N>, Duration) -> ([f64; N], [f64; N], [f64; N], bool) + Send + 'static,
{
    move |state, duration| {
        let (target, target_vel, target_acc, done) = target_fn(state, duration);
        let q = state.meas.q.unwrap_or(target);
        let dq = state.meas.dq.unwrap_or([0.; N]);

        let mut desired_acc = target_acc;
        for i in 0..N {
            desired_acc[i] += kp[i] * (target[i] - q[i]) + kd[i] * (target_vel[i] - dq[i]);
        }

        let mass = match model.mass(q) {
            Ok(mass) => mass,
            Err(_) => return ([0.; N], done),
        };
        let coriolis = model.coriolis(q, dq).unwrap_or([0.; N]);
        let gravity_torque = model.gravity(q, gravity).unwrap_or([0.; N]);

        let inertial_torque = mass * JVec::<N>::from_column_slice(&desired_acc);
        let mut torque: [f64; N] = inertial_torque.as_slice().try_into().unwrap();
        for i in 0..N {
            torque[i] += coriolis[i] + gravity_torque[i];
        }

        (torque, done)
    }
}
