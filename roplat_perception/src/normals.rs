//! Surface normals and smooth regions, as PCL's `NormalEstimation` and
//! `RegionGrowing`.
//!
//! [`estimate_normals`] takes, for each point, the direction in which its
//! neighbourhood spreads least, turned towards the sensor, and the surface
//! variation `λ₀ / (λ₀ + λ₁ + λ₂)` of the neighbourhood's covariance
//! eigenvalues: 0 on a plane, larger on curved surfaces, edges and noise.
//! [`smooth_regions`] then grows regions from the flattest points across
//! neighbours whose normals turn by less than a threshold, so surfaces that
//! meet at a crease (a box's faces, a can against a box) come apart even
//! where Euclidean clustering joins them.

use std::collections::VecDeque;

use nalgebra as na;

use crate::grid::{Grid, is_finite};

/// A point's surface normal (unit, towards the viewpoint) and its
/// neighbourhood's surface variation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceNormal {
    pub normal: [f64; 3],
    pub curvature: f64,
}

/// The normal of each point from its neighbours within `radius` (m),
/// turned to face `viewpoint` (the sensor's position, in the points'
/// frame). `None` for a point with fewer than three neighbours, itself
/// included, or not finite.
///
/// The radius sets the scale: a few times the point spacing, and smaller
/// than the features to keep (a radius larger than an edge's distance
/// rounds the edge off).
pub fn estimate_normals(
    points: &[[f64; 3]],
    radius: f64,
    viewpoint: [f64; 3],
) -> Vec<Option<SurfaceNormal>> {
    let grid = Grid::new(points, radius);
    let mut neighbours = Vec::new();
    points
        .iter()
        .map(|p| {
            if !is_finite(p) {
                return None;
            }
            neighbours.clear();
            grid.within(p, radius, |j| neighbours.push(j));
            if neighbours.len() < 3 {
                return None;
            }
            let centre: na::Vector3<f64> = neighbours
                .iter()
                .map(|&j| na::Vector3::from(points[j]))
                .sum::<na::Vector3<f64>>()
                / neighbours.len() as f64;
            let mut covariance = na::Matrix3::zeros();
            for &j in &neighbours {
                let d = na::Vector3::from(points[j]) - centre;
                covariance += d * d.transpose();
            }
            let eigen = covariance.symmetric_eigen();
            let (least, _) = eigen
                .eigenvalues
                .iter()
                .enumerate()
                .min_by(|a, b| a.1.total_cmp(b.1))
                .expect("three eigenvalues");
            let mut normal = eigen.eigenvectors.column(least).normalize();
            if normal.dot(&(na::Vector3::from(viewpoint) - na::Vector3::from(*p))) < 0.0 {
                normal = -normal;
            }
            let total = eigen.eigenvalues.sum();
            let curvature = if total > 0.0 {
                eigen.eigenvalues[least].max(0.0) / total
            } else {
                0.0
            };
            normal
                .iter()
                .all(|c| c.is_finite())
                .then_some(SurfaceNormal { normal: [normal.x, normal.y, normal.z], curvature })
        })
        .collect()
}

/// Settings for [`smooth_regions`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RegionOptions {
    /// Neighbours are points within this distance (m).
    pub radius: f64,
    /// A neighbour joins when its normal turns by less than this (rad) from
    /// the point it is reached from.
    pub smoothness: f64,
    /// A point grows the region further only when its surface variation is
    /// below this; points on edges join but stop the growth.
    pub curvature: f64,
    /// Regions with fewer points are left out.
    pub min_points: usize,
}

impl Default for RegionOptions {
    /// 1 cm neighbours, 5°, surface variation 0.05, at least 30 points.
    fn default() -> Self {
        Self {
            radius: 0.01,
            smoothness: 5f64.to_radians(),
            curvature: 0.05,
            min_points: 30,
        }
    }
}

/// Points grouped into smooth surfaces, largest first, each as ascending
/// indices, from `normals` as [`estimate_normals`] gives them for the same
/// points. Growth starts at the flattest point not yet in a region and
/// crosses to neighbours whose normals turn by less than
/// `options.smoothness`; a region is a surface, so a box seen from above
/// gives one region per visible face.
///
/// # Panics
///
/// If `normals` and `points` differ in length.
pub fn smooth_regions(
    points: &[[f64; 3]],
    normals: &[Option<SurfaceNormal>],
    options: &RegionOptions,
) -> Vec<Vec<usize>> {
    assert_eq!(points.len(), normals.len(), "one normal per point");
    let grid = Grid::new(points, options.radius);
    let cos_smooth = options.smoothness.cos();
    let mut order: Vec<usize> = (0..points.len())
        .filter(|&i| normals[i].is_some())
        .collect();
    order.sort_by(|&a, &b| {
        let curvature = |i: usize| normals[i].map_or(f64::INFINITY, |n| n.curvature);
        curvature(a).total_cmp(&curvature(b))
    });
    let mut taken = vec![false; points.len()];
    let mut regions = Vec::new();
    for &start in &order {
        if taken[start] {
            continue;
        }
        taken[start] = true;
        let mut region = vec![start];
        let mut seeds = VecDeque::from([start]);
        while let Some(i) = seeds.pop_front() {
            let Some(here) = normals[i] else { continue };
            grid.within(&points[i], options.radius, |j| {
                let Some(there) = normals[j] else { return };
                if taken[j] {
                    return;
                }
                let turn: f64 = (0..3).map(|k| here.normal[k] * there.normal[k]).sum();
                if turn < cos_smooth {
                    return;
                }
                taken[j] = true;
                region.push(j);
                if there.curvature < options.curvature {
                    seeds.push_back(j);
                }
            });
        }
        if region.len() >= options.min_points {
            region.sort_unstable();
            regions.push(region);
        }
    }
    regions.sort_by_key(|region| std::cmp::Reverse(region.len()));
    regions
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A grid of points on the plane `z = height`, every `step`.
    fn plane(height: f64, step: f64, n: usize) -> Vec<[f64; 3]> {
        (0..n)
            .flat_map(|i| (0..n).map(move |j| [i as f64 * step, j as f64 * step, height]))
            .collect()
    }

    #[test]
    fn a_plane_has_its_normal_towards_the_viewpoint_and_no_curvature() {
        let points = plane(0.5, 0.005, 20);
        for viewpoint in [[0.05, 0.05, 2.0], [0.05, 0.05, -1.0]] {
            let normals = estimate_normals(&points, 0.012, viewpoint);
            let up = if viewpoint[2] > 0.5 { 1.0 } else { -1.0 };
            for n in &normals {
                let n = n.expect("every point has neighbours");
                assert!(
                    (n.normal[2] - up).abs() < 1e-9 && n.curvature < 1e-12,
                    "{n:?}"
                );
            }
        }
        // A lone point and a non-finite one have none.
        let normals = estimate_normals(&[[0.0; 3], [f64::NAN, 0.0, 0.0]], 0.01, [0.0, 0.0, 1.0]);
        assert_eq!(normals, [None, None]);
    }

    #[test]
    fn a_sphere_has_radial_normals() {
        // A Fibonacci sphere of radius 0.1, seen from its centre's outside
        // along each normal: normals point outwards.
        let n = 4000;
        let golden = std::f64::consts::PI * (3.0 - 5f64.sqrt());
        let points: Vec<[f64; 3]> = (0..n)
            .map(|i| {
                let z = 1.0 - 2.0 * (i as f64 + 0.5) / n as f64;
                let r = (1.0 - z * z).sqrt();
                let a = golden * i as f64;
                [0.1 * r * a.cos(), 0.1 * r * a.sin(), 0.1 * z]
            })
            .collect();
        for (p, normal) in points
            .iter()
            .zip(estimate_normals(&points, 0.015, [0.0, 0.0, 0.0]))
        {
            // Viewed from the centre the normals face inwards.
            let normal = normal.unwrap();
            let radial = na::Vector3::from(*p).normalize();
            let along = -na::Vector3::from(normal.normal).dot(&radial);
            assert!(along > 0.995, "{along}");
            assert!(normal.curvature > 1e-4 && normal.curvature < 0.02);
        }
    }

    #[test]
    fn two_planes_meeting_at_a_crease_are_two_regions() {
        // A floor and a wall at right angles, joined at x = 0.1: Euclidean
        // clustering sees one object, region growing two surfaces.
        let mut points = Vec::new();
        for i in 0..=20 {
            for j in 0..=20 {
                points.push([0.005 * i as f64, 0.005 * j as f64, 0.0]);
                if i > 0 {
                    points.push([0.1, 0.005 * j as f64, 0.005 * i as f64]);
                }
            }
        }
        let normals = estimate_normals(&points, 0.008, [0.0, 0.05, 0.5]);
        let regions = smooth_regions(&points, &normals, &RegionOptions::default());
        assert_eq!(
            crate::euclidean_clusters(&points, 0.008, 1, usize::MAX).len(),
            1
        );
        assert_eq!(
            regions.len(),
            2,
            "{:?}",
            regions.iter().map(Vec::len).collect::<Vec<_>>()
        );
        for region in &regions {
            let on_floor = region.iter().filter(|&&i| points[i][2] < 1e-9).count();
            let on_wall = region
                .iter()
                .filter(|&&i| points[i][0] > 0.1 - 1e-9)
                .count();
            // Each region is mostly one plane; the crease's points go to
            // whichever reached them first.
            assert!(
                on_floor.min(on_wall) <= 21,
                "floor {on_floor}, wall {on_wall}"
            );
        }
    }
}
