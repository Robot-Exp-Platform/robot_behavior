//! Trajectory generation for `roplat_behavior` robots: joint and Cartesian
//! interpolation (linear, trapezoid, S-curve, fourth-order) in
//! [`path_generate`], and time-optimal path parameterization through COPP in
//! [`time_scaling`].

pub mod path_generate;
pub mod time_scaling;

pub use time_scaling::*;
