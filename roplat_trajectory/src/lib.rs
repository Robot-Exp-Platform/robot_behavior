//! Trajectory generation for `roplat_behavior` robots: joint and Cartesian
//! interpolation (linear, trapezoid, S-curve, fourth-order) in
//! [`path_generate`], time-optimal path parameterization through COPP in
//! [`time_scaling`], and following a timed joint trajectory on a position
//! channel in [`follow`].

pub mod follow;
pub mod path_generate;
pub mod time_scaling;

pub use follow::{JointTrajectoryFollower, TrackingError, points_from_samples};
pub use time_scaling::*;
