//! Target sources for the tracking controllers: closures that give a joint
//! or Cartesian target for each control cycle, from a fixed target, a sampled
//! trajectory or a path indexed by elapsed time.

use std::time::Duration;

use roplat_behavior::{ArmState, JointState, Pose};

/// Build a controller-compatible joint target source around a fixed target.
pub fn constant_joint_target_fn<const N: usize>(
    target: [f64; N],
) -> impl FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static {
    move |_state, _duration| (target, false)
}

/// Build a controller-compatible joint target source around a sampled trajectory.
pub fn joint_traj_target_fn<const N: usize>(
    traj: Vec<[f64; N]>,
) -> impl FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static {
    let mut step = 0usize;

    move |state, duration| {
        if traj.is_empty() {
            return (state.meas.q().unwrap_or([0.0; N]), true);
        }
        if duration == Duration::ZERO && step == 0 {
            return (traj[0], false);
        }

        let index = step.min(traj.len() - 1);
        let target = traj[index];
        if duration > Duration::ZERO {
            step += 1;
        }
        (target, step >= traj.len())
    }
}

/// Wrap a duration-indexed joint path as a controller-compatible target source.
pub fn joint_path_target_fn<F, const N: usize>(
    path: F,
    total: Duration,
) -> impl FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static
where
    F: Fn(Duration) -> [f64; N] + Send + 'static,
{
    let mut elapsed = Duration::ZERO;

    move |_state, duration| {
        elapsed += duration;
        let t = elapsed.min(total);
        (path(t), elapsed >= total)
    }
}

/// Build a controller-compatible Cartesian target source around a fixed pose.
pub fn constant_pose_target_fn<const N: usize>(
    target: Pose,
) -> impl FnMut(ArmState<N>, Duration) -> (Pose, bool) + Send + 'static {
    move |_state, _duration| (target, false)
}

/// Build a controller-compatible Cartesian target source around a sampled pose trajectory.
pub fn pose_traj_target_fn<const N: usize>(
    traj: Vec<Pose>,
) -> impl FnMut(ArmState<N>, Duration) -> (Pose, bool) + Send + 'static {
    let mut step = 0usize;

    move |state, duration| {
        if traj.is_empty() {
            return (state.flange.meas.pose().unwrap_or_default(), true);
        }
        if duration == Duration::ZERO && step == 0 {
            return (traj[0], false);
        }

        let index = step.min(traj.len() - 1);
        let target = traj[index];
        if duration > Duration::ZERO {
            step += 1;
        }
        (target, step >= traj.len())
    }
}

/// Wrap a duration-indexed Cartesian path as a controller-compatible target source.
pub fn pose_path_target_fn<F, const N: usize>(
    path: F,
    total: Duration,
) -> impl FnMut(ArmState<N>, Duration) -> (Pose, bool) + Send + 'static
where
    F: Fn(Duration) -> Pose + Send + 'static,
{
    let mut elapsed = Duration::ZERO;

    move |_state, duration| {
        elapsed += duration;
        let t = elapsed.min(total);
        (path(t), elapsed >= total)
    }
}
