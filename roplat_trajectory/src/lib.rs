//! Trajectory generation for `roplat_behavior` robots: joint and Cartesian
//! interpolation (linear, trapezoid, S-curve, fourth-order) in
//! [`path_generate`], time-optimal path parameterization through COPP in
//! [`time_scaling`], and collision-free joint-space paths in
//! [`path_planning`].

pub mod path_generate;
pub mod path_planning;
pub mod time_scaling;

pub use path_planning::{PlanError, PlannerOptions, plan_joint_path, segment_is_valid};
pub use time_scaling::*;
