//! Aligning a model's points with sensed ones: iterative closest point
//! (point-to-point, as PCL's `IterativeClosestPoint` and Open3D's default).
//!
//! Given a model of a known object and an initial guess of its pose, such
//! as the pose of the box [`segment_tabletop`](crate::segment_tabletop)
//! fitted around it, ICP refines the pose to bring the model onto the
//! sensed points. It converges to the nearest local fit, so the guess must
//! be within roughly the matching distance and a fraction of a turn.
//!
//! Every source point is matched, so the source should be the partial one:
//! a camera sees an object's near sides only, and the model's hidden sides
//! would be pulled onto whatever is near. Register the sensed points
//! (source) onto the model (target), from the inverse of the guessed pose,
//! and invert the result to get the object's pose.

use nalgebra as na;

use crate::grid::{Grid, is_finite};

/// Settings for [`icp`].
#[derive(Clone, Copy, Debug)]
pub struct IcpOptions {
    /// Pairs farther apart than this (m) are not matched.
    pub max_distance: f64,
    pub max_iterations: usize,
    /// Stop when an iteration moves the model by less than this (m: its
    /// translation plus its rotation times the model's extent).
    pub tolerance: f64,
}

impl Default for IcpOptions {
    fn default() -> Self {
        Self { max_distance: 0.02, max_iterations: 50, tolerance: 1e-6 }
    }
}

/// The result of [`icp`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Registration {
    /// Takes the source points onto the target.
    pub transform: na::Isometry3<f64>,
    /// Root-mean-square distance (m) of the matched pairs.
    pub rmse: f64,
    /// Share of the source points matched within `max_distance`.
    pub fitness: f64,
    pub iterations: usize,
}

/// The rigid transform that takes `source` onto `target`, starting from
/// `initial`: each iteration matches every moved source point with its
/// nearest target point within `max_distance` and solves for the transform
/// that best aligns the pairs (Kabsch). `None` when no pair matches.
pub fn icp(
    source: &[[f64; 3]],
    target: &[[f64; 3]],
    initial: &na::Isometry3<f64>,
    options: &IcpOptions,
) -> Option<Registration> {
    let grid = Grid::new(target, options.max_distance);
    let source: Vec<na::Point3<f64>> = source
        .iter()
        .filter(|p| is_finite(p))
        .map(|p| na::Point3::from(*p))
        .collect();
    let extent = source.iter().map(|p| p.coords.norm()).fold(0.0, f64::max);
    let mut transform = *initial;
    let mut iterations = 0;
    loop {
        let pairs = matches(&grid, target, &source, &transform, options.max_distance);
        if pairs.is_empty() {
            return None;
        }
        if iterations == options.max_iterations {
            return Some(score(&pairs, source.len(), transform, iterations));
        }
        let step = kabsch(&pairs);
        transform = step * transform;
        iterations += 1;
        let moved = step.translation.vector.norm() + step.rotation.angle() * extent;
        if moved < options.tolerance {
            let pairs = matches(&grid, target, &source, &transform, options.max_distance);
            return (!pairs.is_empty()).then(|| score(&pairs, source.len(), transform, iterations));
        }
    }
}

/// Each moved source point with its nearest target point in reach.
fn matches(
    grid: &Grid<'_>,
    target: &[[f64; 3]],
    source: &[na::Point3<f64>],
    transform: &na::Isometry3<f64>,
    reach: f64,
) -> Vec<(na::Point3<f64>, na::Point3<f64>)> {
    source
        .iter()
        .filter_map(|p| {
            let moved = transform * p;
            let (nearest, _) = grid.nearest(&moved.coords.into(), reach)?;
            Some((moved, na::Point3::from(target[nearest])))
        })
        .collect()
}

/// The rigid transform that best takes the first point of each pair onto
/// the second, in the least-squares sense.
fn kabsch(pairs: &[(na::Point3<f64>, na::Point3<f64>)]) -> na::Isometry3<f64> {
    let n = pairs.len() as f64;
    let from = pairs
        .iter()
        .map(|(a, _)| a.coords)
        .sum::<na::Vector3<f64>>()
        / n;
    let to = pairs
        .iter()
        .map(|(_, b)| b.coords)
        .sum::<na::Vector3<f64>>()
        / n;
    let mut covariance = na::Matrix3::zeros();
    for (a, b) in pairs {
        covariance += (a.coords - from) * (b.coords - to).transpose();
    }
    let svd = covariance.svd(true, true);
    let (u, v) = (svd.u.unwrap(), svd.v_t.unwrap().transpose());
    // A reflection is not a rotation: flip the weakest axis.
    let sign = (v * u.transpose()).determinant().signum();
    let rotation =
        v * na::Matrix3::from_diagonal(&na::Vector3::new(1.0, 1.0, sign)) * u.transpose();
    let rotation = na::UnitQuaternion::from_matrix(&rotation);
    na::Isometry3::from_parts(na::Translation3::from(to - rotation * from), rotation)
}

fn score(
    pairs: &[(na::Point3<f64>, na::Point3<f64>)],
    sources: usize,
    transform: na::Isometry3<f64>,
    iterations: usize,
) -> Registration {
    let squared: f64 = pairs.iter().map(|(a, b)| (a - b).norm_squared()).sum();
    Registration {
        transform,
        rmse: (squared / pairs.len() as f64).sqrt(),
        fitness: pairs.len() as f64 / sources.max(1) as f64,
        iterations,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Three plates of an asymmetric bracket: no symmetry for ICP to slide
    /// along.
    fn bracket() -> Vec<[f64; 3]> {
        let mut points = Vec::new();
        for i in 0..20 {
            for j in 0..10 {
                let (a, b) = (0.005 * i as f64, 0.005 * j as f64);
                points.push([a, b, 0.0]);
                points.push([0.0, b, 0.6 * a]);
                points.push([0.4 * a, 0.0, b]);
            }
        }
        points
    }

    #[test]
    fn a_small_offset_is_recovered() {
        let model = bracket();
        let truth = na::Isometry3::new(
            na::Vector3::new(0.004, -0.003, 0.002),
            na::Vector3::new(0.05, -0.03, 0.08),
        );
        let sensed: Vec<[f64; 3]> = model
            .iter()
            .map(|p| (truth * na::Point3::from(*p)).coords.into())
            .collect();
        let found = icp(
            &model,
            &sensed,
            &na::Isometry3::identity(),
            &IcpOptions::default(),
        )
        .unwrap();
        assert!(found.rmse < 1e-6, "{found:?}");
        assert!((found.transform.translation.vector - truth.translation.vector).norm() < 1e-5);
        assert!(found.transform.rotation.angle_to(&truth.rotation) < 1e-4);
        assert!((found.fitness - 1.0).abs() < 1e-12);
    }

    #[test]
    fn nothing_in_reach_gives_none() {
        let far = na::Isometry3::translation(1.0, 0.0, 0.0);
        assert!(icp(&bracket(), &bracket(), &far, &IcpOptions::default()).is_none());
    }
}
