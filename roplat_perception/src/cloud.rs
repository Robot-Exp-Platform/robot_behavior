//! Thinning and filtering a cloud: PCL's voxel grid, crop box and radius
//! outlier filters.
//!
//! Non-finite points (no return) are dropped by all of them.

use std::collections::BTreeMap;

use nalgebra as na;

use crate::grid::{Grid, is_finite};

/// One point per voxel of edge `voxel` (m) that holds any: the centroid of
/// the points in it, so the cloud keeps its shape at an even density.
/// Ordered by voxel, so the result does not depend on the input order.
///
/// # Panics
///
/// If `voxel` is not positive.
pub fn voxel_downsample(points: &[[f64; 3]], voxel: f64) -> Vec<[f64; 3]> {
    assert!(
        voxel > 0.0 && voxel.is_finite(),
        "voxel edge {voxel} must be positive"
    );
    let mut cells: BTreeMap<[i64; 3], ([f64; 3], usize)> = BTreeMap::new();
    for p in points.iter().filter(|p| is_finite(p)) {
        let (sum, count) = cells
            .entry(p.map(|c| (c / voxel).floor() as i64))
            .or_insert(([0.0; 3], 0));
        for k in 0..3 {
            sum[k] += p[k];
        }
        *count += 1;
    }
    cells
        .into_values()
        .map(|(sum, count)| sum.map(|s| s / count as f64))
        .collect()
}

/// The points inside the box of `half_extents` (m) centred at `pose`, such
/// as the workspace in front of a robot.
pub fn crop_box(
    points: &[[f64; 3]],
    pose: &na::Isometry3<f64>,
    half_extents: [f64; 3],
) -> Vec<[f64; 3]> {
    let inverse = pose.inverse();
    points
        .iter()
        .filter(|p| is_finite(p))
        .filter(|p| {
            let local = inverse * na::Point3::from(**p);
            (0..3).all(|k| local[k].abs() <= half_extents[k])
        })
        .copied()
        .collect()
}

/// The points with at least `min_neighbours` other points within `radius`
/// (m): isolated returns, such as the flying pixels a depth camera gives
/// along object edges, are dropped.
pub fn remove_sparse(points: &[[f64; 3]], radius: f64, min_neighbours: usize) -> Vec<[f64; 3]> {
    let grid = Grid::new(points, radius);
    points
        .iter()
        .filter(|p| {
            let mut neighbours = 0;
            grid.within(p, radius, |_| neighbours += 1);
            // `within` counts the point itself.
            is_finite(p) && neighbours > min_neighbours
        })
        .copied()
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn downsampling_keeps_one_centroid_per_voxel() {
        let points = [
            [0.01, 0.01, 0.0],
            [0.03, 0.03, 0.0],
            [0.11, 0.0, 0.0],
            [f64::NAN, 0.0, 0.0],
        ];
        let thin = voxel_downsample(&points, 0.1);
        assert_eq!(thin.len(), 2);
        assert!((thin[0][0] - 0.02).abs() < 1e-12 && (thin[0][1] - 0.02).abs() < 1e-12);
        assert_eq!(thin[1], [0.11, 0.0, 0.0]);
        // The order of the input does not matter.
        let mut reversed = points;
        reversed.reverse();
        assert_eq!(voxel_downsample(&reversed, 0.1), thin);
    }

    #[test]
    fn cropping_uses_the_box_frame() {
        let pose = na::Isometry3::new(
            na::Vector3::new(1.0, 0.0, 0.0),
            na::Vector3::new(0.0, 0.0, std::f64::consts::FRAC_PI_2),
        );
        // Long along the box's x, which is the world's y.
        let kept = crop_box(&[[1.0, 0.4, 0.0], [1.4, 0.0, 0.0]], &pose, [0.5, 0.1, 0.1]);
        assert_eq!(kept, [[1.0, 0.4, 0.0]]);
    }

    #[test]
    fn isolated_points_are_removed() {
        let mut points: Vec<[f64; 3]> = (0..10).map(|i| [0.01 * i as f64, 0.0, 0.0]).collect();
        points.push([1.0, 1.0, 1.0]);
        let kept = remove_sparse(&points, 0.015, 1);
        assert_eq!(kept.len(), 10);
        assert!(!kept.contains(&[1.0, 1.0, 1.0]));
    }
}
