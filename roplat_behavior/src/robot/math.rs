//! nalgebra aliases used by model and space types.

use nalgebra as na;

/// A rigid-body transform in `SE(3)`.
pub type Iso3 = na::Isometry3<f64>;
/// A joint-space column vector of size `N`.
pub type JVec<const N: usize> = na::SVector<f64, N>;
/// A square `N x N` joint-space matrix (e.g. mass / inertia).
pub type JMat<const N: usize> = na::SMatrix<f64, N, N>;
/// A `6 x N` geometric Jacobian mapping joint rates to an end-effector twist.
pub type Jaco<const N: usize> = na::SMatrix<f64, 6, N>;
