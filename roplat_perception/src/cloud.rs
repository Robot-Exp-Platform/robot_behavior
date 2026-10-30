//! Thinning and filtering a cloud: PCL's voxel grid, crop box, radius
//! outlier and statistical outlier filters.
//!
//! Non-finite points (no return) are dropped by all of them.

use std::collections::BTreeMap;

use nalgebra as na;

use crate::grid::{Grid, is_finite, squared_distance};

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

/// The points whose mean distance to their `k` nearest neighbours is at
/// most `std_ratio` standard deviations above that distance's mean over the
/// cloud, as PCL's `StatisticalOutlierRemoval` (`setMeanK(k)`,
/// `setStddevMulThresh(std_ratio)`). Speckle and the flying pixels between
/// an edge and the background go; the cloud's own density sets the scale,
/// so unlike [`remove_sparse`] no radius is chosen. In input order; with
/// `k = 0` or no more than `k` points, every finite point is kept.
pub fn remove_statistical_outliers(points: &[[f64; 3]], k: usize, std_ratio: f64) -> Vec<[f64; 3]> {
    let finite: Vec<[f64; 3]> = points.iter().filter(|p| is_finite(p)).copied().collect();
    if k == 0 || finite.len() <= k {
        return finite;
    }
    // A radius that holds about k points if the cloud is a surface spanning
    // the two largest extents of its box. Doubled while it holds fewer, up
    // to eight times; past that the whole cloud is searched, which only an
    // isolated point needs.
    let mut extent: Vec<f64> = (0..3)
        .map(|axis| {
            let (lo, hi) = finite
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                    (lo.min(p[axis]), hi.max(p[axis]))
                });
            hi - lo
        })
        .collect();
    extent.sort_by(f64::total_cmp);
    let area = extent[1] * extent[2];
    let start = (area * k as f64 / (std::f64::consts::PI * finite.len() as f64))
        .sqrt()
        .max(1e-6);
    let grid = Grid::new(&finite, start);
    let mut distances = Vec::new();
    let means: Vec<f64> = finite
        .iter()
        .map(|p| {
            let mut radius = start;
            distances.clear();
            loop {
                grid.within(p, radius, |j| {
                    distances.push(squared_distance(&finite[j], p))
                });
                if distances.len() > k {
                    break;
                }
                distances.clear();
                if radius >= 8.0 * start {
                    distances.extend(finite.iter().map(|q| squared_distance(q, p)));
                    break;
                }
                radius *= 2.0;
            }
            // The k + 1 smallest, the point itself (at zero) among them.
            distances.select_nth_unstable_by(k, f64::total_cmp);
            distances[..=k].iter().map(|d| d.sqrt()).sum::<f64>() / k as f64
        })
        .collect();
    let n = means.len() as f64;
    let mean = means.iter().sum::<f64>() / n;
    let deviation = (means.iter().map(|m| (m - mean).powi(2)).sum::<f64>() / n).sqrt();
    let limit = mean + std_ratio * deviation;
    finite
        .into_iter()
        .zip(means)
        .filter(|&(_, m)| m <= limit)
        .map(|(p, _)| p)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn statistical_outliers_are_the_points_far_from_their_neighbours() {
        // A 40 x 40 grid 5 mm apart, and 20 stray points 2 to 10 cm above
        // it, about a centimetre from each other.
        let mut points: Vec<[f64; 3]> = (0..40)
            .flat_map(|i| (0..40).map(move |j| [i as f64 * 0.005, j as f64 * 0.005, 0.0]))
            .collect();
        let strays: Vec<[f64; 3]> = (0..20)
            .map(|k| {
                let (a, b) = ((k % 10) as f64, (k / 10) as f64);
                [0.01 * a + 0.003, 0.02 * b + 0.05, 0.02 + 0.004 * k as f64]
            })
            .collect();
        points.extend(&strays);
        points.push([f64::NAN, 0.0, 0.0]);
        let kept = remove_statistical_outliers(&points, 8, 1.0);
        assert!(strays.iter().all(|s| !kept.contains(s)));
        // The grid stays, in order; at most its corners, whose neighbours
        // are farther, go too.
        assert!(
            kept.len() >= 1596 && kept.iter().all(|p| p[2] == 0.0),
            "{}",
            kept.len()
        );
        assert!(kept.windows(2).all(|w| w[0] < w[1]));
        // Nothing to compare with: every finite point stays.
        assert_eq!(remove_statistical_outliers(&points[..5], 8, 1.0).len(), 5);
        assert_eq!(remove_statistical_outliers(&points, 0, 1.0).len(), 1620);
    }

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
