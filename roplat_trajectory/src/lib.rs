//! Trajectory generation for `roplat_behavior` robots: joint and Cartesian
//! interpolation (linear, trapezoid, S-curve, fourth-order) in
//! [`path_generate`], time-optimal path parameterization through COPP in
//! [`time_scaling`], collision-free joint-space paths in [`path_planning`],
//! straight tip paths checked in joint space in [`cartesian_path`], and
//! following a timed joint trajectory on a position channel in [`follow`].

pub mod cartesian_path;
pub mod follow;
pub mod path_generate;
pub mod path_planning;
pub mod time_scaling;

pub use cartesian_path::{CartesianPath, CartesianPathOptions, CartesianStop, plan_cartesian_path};
pub use follow::{JointTrajectoryFollower, TrackingError, points_from_samples};
pub use path_planning::{PlanError, PlannerOptions, plan_joint_path, segment_is_valid};
pub use time_scaling::*;
