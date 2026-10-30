//! Path constraints on the tool's orientation.
//!
//! A constraint is a condition on the tool pose that every configuration of
//! a path must meet, such as "keep the cup upright". The planners here take
//! one validity predicate per configuration, so a constraint joins the
//! collision check through [`constrained`]:
//!
//! ```
//! # use nalgebra::{Isometry3, Vector3};
//! # use roplat_trajectory::{AxisConstraint, PlannerOptions, constrained, plan_joint_path};
//! # fn forward(q: &[f64; 2]) -> Isometry3<f64> { Isometry3::rotation(Vector3::z() * q[0]) }
//! # let is_free = |_: &[f64; 2]| true;
//! // The tool's z axis stays within 10° of the world's z axis.
//! let upright = AxisConstraint::new(Vector3::z(), Vector3::z(), 10f64.to_radians());
//! let path = plan_joint_path(
//!     [0.0, 0.0], [1.0, 0.0], [-3.0; 2], [3.0; 2],
//!     constrained(forward, upright, is_free),
//!     PlannerOptions::default(),
//! );
//! assert!(path.is_ok());
//! ```
//!
//! Joint-space planners check a segment at `resolution` steps, so a
//! constraint holds at those samples; between two of them it can be exceeded
//! by roughly what one step turns the tool. Rejection is all the planner
//! does with a constraint: a narrow tolerance leaves few valid samples and
//! makes the search slow or fail, and a constraint that the start or goal
//! breaks is reported as an invalid start or goal.

use nalgebra as na;

/// A condition on the tool pose in the world.
pub trait PoseConstraint {
    fn holds(&self, tool: &na::Isometry3<f64>) -> bool;
}

/// One axis of the tool stays within `tolerance` (rad) of a direction in
/// the world; rotation about that axis is free. "Upright" for a held cup is
/// the tool axis along the cup's opening kept near world `+z`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxisConstraint {
    axis: na::Unit<na::Vector3<f64>>,
    direction: na::Unit<na::Vector3<f64>>,
    tolerance: f64,
}

impl AxisConstraint {
    /// `axis` is in the tool frame, `direction` in the world; neither may
    /// be zero.
    pub fn new(axis: na::Vector3<f64>, direction: na::Vector3<f64>, tolerance: f64) -> Self {
        Self {
            axis: na::Unit::new_normalize(axis),
            direction: na::Unit::new_normalize(direction),
            tolerance,
        }
    }

    /// Angle (rad) between the tool's axis and the direction.
    pub fn deviation(&self, tool: &na::Isometry3<f64>) -> f64 {
        let axis = tool.rotation * self.axis.into_inner();
        axis.dot(&self.direction).clamp(-1.0, 1.0).acos()
    }
}

impl PoseConstraint for AxisConstraint {
    fn holds(&self, tool: &na::Isometry3<f64>) -> bool {
        self.deviation(tool) <= self.tolerance
    }
}

/// The whole tool orientation stays within `tolerance` (rad, the angle of
/// the rotation between them) of a fixed orientation in the world.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OrientationConstraint {
    target: na::UnitQuaternion<f64>,
    tolerance: f64,
}

impl OrientationConstraint {
    pub fn new(target: na::UnitQuaternion<f64>, tolerance: f64) -> Self {
        Self { target, tolerance }
    }

    /// Angle (rad) of the rotation from the target to the tool orientation.
    pub fn deviation(&self, tool: &na::Isometry3<f64>) -> f64 {
        self.target.angle_to(&tool.rotation)
    }
}

impl PoseConstraint for OrientationConstraint {
    fn holds(&self, tool: &na::Isometry3<f64>) -> bool {
        self.deviation(tool) <= self.tolerance
    }
}

/// A validity predicate for the planners: `is_valid(q)`, then the
/// constraint on the tool pose `forward(q)`. `forward` runs only for a
/// configuration that passed `is_valid`.
pub fn constrained<const N: usize, C: PoseConstraint>(
    forward: impl Fn(&[f64; N]) -> na::Isometry3<f64>,
    constraint: C,
    mut is_valid: impl FnMut(&[f64; N]) -> bool,
) -> impl FnMut(&[f64; N]) -> bool {
    move |q| is_valid(q) && constraint.holds(&forward(q))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn tilted(angle: f64) -> na::Isometry3<f64> {
        na::Isometry3::rotation(na::Vector3::x() * angle)
    }

    #[test]
    fn axis_constraint_measures_tilt_and_ignores_twist() {
        let upright = AxisConstraint::new(na::Vector3::z(), na::Vector3::z(), 0.2);
        assert!(upright.deviation(&tilted(0.0)) < 1e-12);
        assert!((upright.deviation(&tilted(0.15)) - 0.15).abs() < 1e-12);
        assert!(upright.holds(&tilted(0.15)));
        assert!(!upright.holds(&tilted(0.25)));
        // Twisting about the constrained axis is free.
        let twisted = na::Isometry3::rotation(na::Vector3::z() * 2.0);
        assert!(upright.holds(&twisted));
        // Upside down is the largest deviation.
        assert!(
            (upright.deviation(&tilted(std::f64::consts::PI)) - std::f64::consts::PI).abs() < 1e-9
        );
    }

    #[test]
    fn orientation_constraint_counts_twist_too() {
        let fixed = OrientationConstraint::new(na::UnitQuaternion::identity(), 0.1);
        assert!(fixed.holds(&tilted(0.05)));
        let twisted = na::Isometry3::rotation(na::Vector3::z() * FRAC_PI_2);
        assert!(!fixed.holds(&twisted));
        assert!((fixed.deviation(&twisted) - FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn constrained_combines_both_checks() {
        // q[0] tilts the tool about x; q[1] has no effect.
        let forward = |q: &[f64; 2]| tilted(q[0]);
        let upright = AxisConstraint::new(na::Vector3::z(), na::Vector3::z(), 0.3);
        let mut free_unless_q1_large = constrained(forward, upright, |q: &[f64; 2]| q[1] < 1.0);
        assert!(free_unless_q1_large(&[0.2, 0.0]));
        assert!(!free_unless_q1_large(&[0.4, 0.0]));
        assert!(!free_unless_q1_large(&[0.2, 1.5]));
    }

    #[test]
    fn planner_detours_to_keep_the_tool_upright() {
        use crate::{PlannerOptions, plan_joint_path, segment_is_valid};
        // q[0] tilts the tool; q[1] slides. Going from slide 0 to slide 1 at
        // tilt 0 is straight, but an obstacle forces q[1] in (0.4, 0.6) to
        // pass with |q[0]| > 0.5 unless the tilt limit forbids it.
        let forward = |q: &[f64; 2]| tilted(q[0]);
        let wall = |q: &[f64; 2]| !(0.4..0.6).contains(&q[1]) || q[0].abs() > 0.5;
        let options = PlannerOptions { seed: 7, ..PlannerOptions::default() };
        // Loose: tilting past the wall is allowed and found.
        let loose = AxisConstraint::new(na::Vector3::z(), na::Vector3::z(), 1.0);
        let path = plan_joint_path(
            [0.0, 0.0],
            [0.0, 1.0],
            [-1.5, 0.0],
            [1.5, 1.0],
            constrained(forward, loose, wall),
            options,
        )
        .unwrap();
        for pair in path.windows(2) {
            assert!(segment_is_valid(
                &pair[0],
                &pair[1],
                options.resolution,
                constrained(forward, loose, wall)
            ));
        }
        // Tight: no configuration both clears the wall and stays upright.
        let tight = AxisConstraint::new(na::Vector3::z(), na::Vector3::z(), 0.3);
        let options = PlannerOptions { max_iterations: 300, ..options };
        assert!(
            plan_joint_path(
                [0.0, 0.0],
                [0.0, 1.0],
                [-1.5, 0.0],
                [1.5, 1.0],
                constrained(forward, tight, wall),
                options
            )
            .is_err()
        );
    }
}
