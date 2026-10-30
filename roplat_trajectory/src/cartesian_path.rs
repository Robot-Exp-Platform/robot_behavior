//! Straight tip paths in Cartesian space, checked in joint space.
//!
//! [`plan_cartesian_path`] moves the tip along straight lines through given
//! poses: position interpolated linearly, orientation by slerp, at most
//! `max_step` metres and `max_rotation_step` radians apart. Each pose is
//! solved by the caller's inverse kinematics, seeded with the previous
//! configuration. The path stops before the first pose that has no
//! solution, needs a joint to jump (the solver switched branches), or is not
//! reached by a valid joint segment: as in [`plan_joint_path`], validity is
//! the caller's (a simulator's distance query, primitive geometry, a
//! workspace limit), checked every `resolution` radians. What was reached is
//! returned with the fraction of the requested steps it covers, so a caller
//! can accept a partial path. The waypoints can be timed with
//! [`plan_waypoints_traj_via_copp`](crate::plan_waypoints_traj_via_copp).
//!
//! [`plan_joint_path`]: crate::plan_joint_path

use nalgebra as na;

use crate::segment_is_valid;

/// Steps per segment are rounded up after subtracting this, in units of a step.
const STEP_ROUNDING: f64 = 1e-9;

/// Settings for [`plan_cartesian_path`].
#[derive(Clone, Copy, Debug)]
pub struct CartesianPathOptions {
    /// Largest tip translation between consecutive poses (m).
    pub max_step: f64,
    /// Largest tip rotation between consecutive poses (rad).
    pub max_rotation_step: f64,
    /// Largest change of any joint between consecutive configurations; a
    /// larger one means the solver jumped to another branch.
    pub max_joint_step: f64,
    /// Largest per-joint gap between configurations checked with
    /// `is_valid` along each joint segment.
    pub resolution: f64,
}

impl Default for CartesianPathOptions {
    fn default() -> Self {
        Self {
            max_step: 0.01,
            max_rotation_step: 0.05,
            max_joint_step: 0.3,
            resolution: 0.02,
        }
    }
}

/// Why a [`CartesianPath`] stopped short.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CartesianStop {
    /// The start configuration is not valid.
    InvalidStart,
    /// The inverse kinematics found no (finite) solution for the next pose.
    Unreachable,
    /// The solution for the next pose is more than `max_joint_step` away.
    JointJump,
    /// The joint segment to the next pose is not valid.
    Invalid,
}

/// The result of [`plan_cartesian_path`].
#[derive(Clone, Debug, PartialEq)]
pub struct CartesianPath<const N: usize> {
    /// Configurations from the start, one per interpolated pose reached.
    pub waypoints: Vec<[f64; N]>,
    /// Share of the requested interpolated poses reached, in `[0, 1]`.
    pub fraction: f64,
    /// `None` when the whole path was reached.
    pub stop: Option<CartesianStop>,
}

/// Plans straight tip motion from `start` through each of `poses` in turn;
/// see the [module documentation](self).
///
/// `forward` gives the tip pose of a configuration (used once, for the
/// start); `inverse(pose, seed)` gives a configuration reaching `pose` near
/// `seed`, or `None`. With `roplat_kinematics`, `chain_ik` returning its
/// solution only when `converged` is such a solver.
pub fn plan_cartesian_path<const N: usize>(
    start: [f64; N],
    poses: &[na::Isometry3<f64>],
    forward: impl FnOnce(&[f64; N]) -> na::Isometry3<f64>,
    inverse: impl FnMut(&na::Isometry3<f64>, &[f64; N]) -> Option<[f64; N]>,
    mut is_valid: impl FnMut(&[f64; N]) -> bool,
    options: CartesianPathOptions,
) -> CartesianPath<N> {
    let start_valid = is_valid(&start);
    cartesian(
        start,
        poses,
        forward,
        inverse,
        start_valid,
        |a, b| segment_is_valid(a, b, options.resolution, &mut is_valid),
        options,
    )
}

/// As [`plan_cartesian_path`], with each joint-space step checked whole by
/// `motion_is_valid(from, to)` instead of sampled, such as
/// `PlanningScene::motion_is_free` in `roplat_scene`; the start is checked
/// as the motion from it to itself. `options.resolution` is not used.
pub fn plan_cartesian_motion<const N: usize>(
    start: [f64; N],
    poses: &[na::Isometry3<f64>],
    forward: impl FnOnce(&[f64; N]) -> na::Isometry3<f64>,
    inverse: impl FnMut(&na::Isometry3<f64>, &[f64; N]) -> Option<[f64; N]>,
    mut motion_is_valid: impl FnMut(&[f64; N], &[f64; N]) -> bool,
    options: CartesianPathOptions,
) -> CartesianPath<N> {
    let start_valid = motion_is_valid(&start, &start);
    cartesian(
        start,
        poses,
        forward,
        inverse,
        start_valid,
        motion_is_valid,
        options,
    )
}

/// The path through `poses`, with `segment` deciding whether each joint-space
/// step is valid.
fn cartesian<const N: usize>(
    start: [f64; N],
    poses: &[na::Isometry3<f64>],
    forward: impl FnOnce(&[f64; N]) -> na::Isometry3<f64>,
    mut inverse: impl FnMut(&na::Isometry3<f64>, &[f64; N]) -> Option<[f64; N]>,
    start_valid: bool,
    mut segment: impl FnMut(&[f64; N], &[f64; N]) -> bool,
    options: CartesianPathOptions,
) -> CartesianPath<N> {
    let mut from = forward(&start);
    let mut segments = Vec::with_capacity(poses.len());
    for to in poses {
        let translation = (to.translation.vector - from.translation.vector).norm();
        let rotation = from.rotation.angle_to(&to.rotation);
        // A ratio within rounding of a whole number takes that many steps, so
        // the count does not depend on the platform's `acos`.
        let steps = ((translation / options.max_step).max(rotation / options.max_rotation_step)
            - STEP_ROUNDING)
            .ceil()
            .max(1.0) as usize;
        segments.push((from, *to, steps));
        from = *to;
    }
    let total: usize = segments.iter().map(|s| s.2).sum();

    let mut waypoints = vec![start];
    let stopped = |waypoints: Vec<[f64; N]>, stop| {
        let fraction = if total == 0 {
            0.0
        } else {
            (waypoints.len() - 1) as f64 / total as f64
        };
        CartesianPath { waypoints, fraction, stop: Some(stop) }
    };
    if !start_valid {
        return CartesianPath {
            waypoints,
            fraction: 0.0,
            stop: Some(CartesianStop::InvalidStart),
        };
    }
    for (from, to, steps) in segments {
        for k in 1..=steps {
            let pose = from.lerp_slerp(&to, k as f64 / steps as f64);
            let previous = *waypoints.last().expect("starts with the start");
            let Some(next) = inverse(&pose, &previous).filter(|q| q.iter().all(|v| v.is_finite()))
            else {
                return stopped(waypoints, CartesianStop::Unreachable);
            };
            let jump = (0..N)
                .map(|i| (next[i] - previous[i]).abs())
                .fold(0.0, f64::max);
            if jump > options.max_joint_step {
                return stopped(waypoints, CartesianStop::JointJump);
            }
            if !segment(&previous, &next) {
                return stopped(waypoints, CartesianStop::Invalid);
            }
            waypoints.push(next);
        }
    }
    CartesianPath { waypoints, fraction: 1.0, stop: None }
}
