//! Splitting a cloud: the dominant plane (PCL's RANSAC plane model) and
//! Euclidean clusters (PCL's `EuclideanClusterExtraction`).

use std::collections::VecDeque;

use nalgebra as na;

use crate::grid::{Grid, is_finite};

/// The points `x` with `normal · x = offset`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane {
    pub normal: na::Unit<na::Vector3<f64>>,
    pub offset: f64,
}

impl Plane {
    /// The plane through `point` with `normal`.
    pub fn through(point: &na::Point3<f64>, normal: na::Unit<na::Vector3<f64>>) -> Self {
        Self { normal, offset: normal.dot(&point.coords) }
    }

    /// Distance of `p` from the plane, positive on the side the normal
    /// points to.
    pub fn signed_distance(&self, p: &[f64; 3]) -> f64 {
        self.normal.dot(&na::Vector3::from(*p)) - self.offset
    }

    /// A frame on the plane: its origin is the plane point nearest the
    /// world origin, `z` the normal; `x` and `y` lie in the plane.
    pub fn frame(&self) -> na::Isometry3<f64> {
        let z = self.normal.into_inner();
        // Any direction not along the normal gives x.
        let other = if z.x.abs() < 0.9 {
            na::Vector3::x()
        } else {
            na::Vector3::y()
        };
        let x = (other - z * z.dot(&other)).normalize();
        let rotation = na::Rotation3::from_basis_unchecked(&[x, z.cross(&x), z]);
        na::Isometry3::from_parts(
            na::Translation3::from(z * self.offset),
            na::UnitQuaternion::from_rotation_matrix(&rotation),
        )
    }
}

/// Settings for [`fit_plane`].
#[derive(Clone, Copy, Debug)]
pub struct PlaneOptions {
    /// Largest distance (m) of a point counted on the plane.
    pub threshold: f64,
    /// Random three-point samples to try.
    pub iterations: usize,
    /// If given, only planes whose normal is within `max_tilt` of this
    /// direction (either way) are considered, and the normal returned
    /// points along it: the table under a gravity-aligned world `z`.
    pub up: Option<na::Vector3<f64>>,
    /// Largest angle (rad) between the normal and `up`.
    pub max_tilt: f64,
    pub seed: u64,
}

impl Default for PlaneOptions {
    fn default() -> Self {
        Self {
            threshold: 0.01,
            iterations: 300,
            up: None,
            max_tilt: 0.2,
            seed: 1,
        }
    }
}

/// A plane and the indices of the points on it.
#[derive(Clone, Debug, PartialEq)]
pub struct PlaneFit {
    pub plane: Plane,
    /// Indices into the points given, ascending.
    pub inliers: Vec<usize>,
}

/// The plane through the most points, by RANSAC: planes through random
/// triples of points are scored by how many points lie within the
/// threshold, and the best one is refitted to its points by least squares.
/// `None` with fewer than three finite points, or when no sampled plane
/// meets the `up` constraint. The seed fixes the result.
pub fn fit_plane(points: &[[f64; 3]], options: &PlaneOptions) -> Option<PlaneFit> {
    let finite: Vec<usize> = (0..points.len())
        .filter(|&i| is_finite(&points[i]))
        .collect();
    if finite.len() < 3 {
        return None;
    }
    let up = options.up.and_then(|up| na::Unit::try_new(up, 1e-12));
    let min_cos = options.max_tilt.cos();
    // Oriented along `up`, or `None` when it tilts too far.
    let accept = |normal: na::Unit<na::Vector3<f64>>| match up {
        None => Some(normal),
        Some(up) => {
            let cos = normal.dot(&up);
            (cos.abs() >= min_cos).then(|| if cos < 0.0 { -normal } else { normal })
        }
    };
    let inliers = |plane: &Plane| -> Vec<usize> {
        finite
            .iter()
            .copied()
            .filter(|&i| plane.signed_distance(&points[i]).abs() <= options.threshold)
            .collect()
    };
    let mut rng = XorShift(options.seed.max(1));
    let mut best: Option<(Plane, usize)> = None;
    for _ in 0..options.iterations {
        let pick = |rng: &mut XorShift| finite[(rng.unit() * finite.len() as f64) as usize];
        let (a, b, c) = (pick(&mut rng), pick(&mut rng), pick(&mut rng));
        let [a, b, c] = [a, b, c].map(|i| na::Point3::from(points[i]));
        let Some(normal) = na::Unit::try_new((b - a).cross(&(c - a)), 1e-12) else {
            continue;
        };
        let Some(normal) = accept(normal) else {
            continue;
        };
        let plane = Plane::through(&a, normal);
        let count = finite
            .iter()
            .filter(|&&i| plane.signed_distance(&points[i]).abs() <= options.threshold)
            .count();
        if best.is_none_or(|(_, most)| count > most) {
            best = Some((plane, count));
        }
    }
    let (plane, _) = best?;
    let on = inliers(&plane);
    // The least-squares plane of the inliers: through their centroid,
    // normal to their least spread.
    let refined = least_squares_normal(points, &on)
        .and_then(accept)
        .map(|normal| Plane::through(&centroid(points, &on), normal))
        .unwrap_or(plane);
    Some(PlaneFit { inliers: inliers(&refined), plane: refined })
}

/// Groups of points joined by steps of at most `tolerance` (m), largest
/// first, each as ascending indices; groups outside
/// `min_points..=max_points` are left out. Each group is one object when
/// objects stand at least `tolerance` apart.
pub fn euclidean_clusters(
    points: &[[f64; 3]],
    tolerance: f64,
    min_points: usize,
    max_points: usize,
) -> Vec<Vec<usize>> {
    let grid = Grid::new(points, tolerance);
    let mut seen = vec![false; points.len()];
    let mut clusters = Vec::new();
    for start in 0..points.len() {
        if seen[start] || !is_finite(&points[start]) {
            continue;
        }
        seen[start] = true;
        let mut cluster = vec![start];
        let mut queue = VecDeque::from([start]);
        while let Some(i) = queue.pop_front() {
            grid.within(&points[i], tolerance, |j| {
                if !seen[j] {
                    seen[j] = true;
                    cluster.push(j);
                    queue.push_back(j);
                }
            });
        }
        if (min_points..=max_points).contains(&cluster.len()) {
            cluster.sort_unstable();
            clusters.push(cluster);
        }
    }
    clusters.sort_by_key(|cluster| std::cmp::Reverse(cluster.len()));
    clusters
}

pub(crate) fn centroid(points: &[[f64; 3]], indices: &[usize]) -> na::Point3<f64> {
    let sum: na::Vector3<f64> = indices.iter().map(|&i| na::Vector3::from(points[i])).sum();
    na::Point3::from(sum / indices.len().max(1) as f64)
}

/// The direction in which the points spread least.
fn least_squares_normal(
    points: &[[f64; 3]],
    indices: &[usize],
) -> Option<na::Unit<na::Vector3<f64>>> {
    if indices.len() < 3 {
        return None;
    }
    let centre = centroid(points, indices);
    let mut covariance = na::Matrix3::zeros();
    for &i in indices {
        let d = na::Point3::from(points[i]) - centre;
        covariance += d * d.transpose();
    }
    let eigen = covariance.symmetric_eigen();
    let least = eigen.eigenvalues.imin();
    na::Unit::try_new(eigen.eigenvectors.column(least).into_owned(), 1e-12)
}

/// xorshift64*: small, seedable, the same everywhere.
struct XorShift(u64);

impl XorShift {
    /// Uniform in `[0, 1)`.
    fn unit(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1u64 << 53) as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 1 m square of points 2 cm apart at height `z`, tilted about x by
    /// `tilt` rad.
    fn sheet(z: f64, tilt: f64) -> Vec<[f64; 3]> {
        let mut points = Vec::new();
        for i in 0..50 {
            for j in 0..50 {
                let (x, y) = (i as f64 * 0.02, j as f64 * 0.02);
                points.push([x, y * tilt.cos(), z + y * tilt.sin()]);
            }
        }
        points
    }

    #[test]
    fn the_dominant_plane_wins_and_is_refined() {
        let mut points = sheet(0.7, 0.0);
        // A smaller wall below the table, and some clutter.
        points.extend(
            sheet(-1.5, std::f64::consts::FRAC_PI_2)
                .into_iter()
                .take(800),
        );
        points.extend((0..100).map(|i| [0.3, 0.3, 0.8 + 0.001 * i as f64]));
        let fit = fit_plane(&points, &PlaneOptions::default()).unwrap();
        assert_eq!(fit.inliers.len(), 2500);
        assert!(fit.plane.normal.z.abs() > 1.0 - 1e-9);
        assert!((fit.plane.signed_distance(&[0.0, 0.0, 0.7])).abs() < 1e-9);
        // The same seed, the same plane.
        assert_eq!(fit_plane(&points, &PlaneOptions::default()).unwrap(), fit);
    }

    #[test]
    fn up_selects_the_level_plane_and_orients_it() {
        let mut points = sheet(0.7, 0.0);
        // A larger wall below the table, not level.
        let wall = sheet(-2.0, std::f64::consts::FRAC_PI_2);
        points.extend(wall.iter().map(|p| [p[0], p[1], p[2] + 1.0]));
        points.extend(wall);
        let options = PlaneOptions { up: Some(-na::Vector3::z()), ..Default::default() };
        let fit = fit_plane(&points, &options).unwrap();
        assert!(fit.plane.normal.z < -1.0 + 1e-9, "{:?}", fit.plane);
        assert_eq!(fit.inliers.len(), 2500);
        // Nothing level within a degree of x.
        let sideways = PlaneOptions {
            up: Some(na::Vector3::x()),
            max_tilt: 0.017,
            ..Default::default()
        };
        assert!(fit_plane(&sheet(0.7, 0.0), &sideways).is_none());
    }

    #[test]
    fn the_plane_frame_lies_in_the_plane() {
        let plane = Plane::through(
            &na::Point3::new(0.0, 0.0, 0.7),
            na::Unit::new_normalize(na::Vector3::new(0.1, 0.0, 1.0)),
        );
        let frame = plane.frame();
        for p in [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]] {
            let world = frame * na::Point3::from(p);
            assert!(plane.signed_distance(&world.coords.into()).abs() < 1e-12);
        }
        let z = frame * na::Vector3::z();
        assert!((z - plane.normal.into_inner()).norm() < 1e-12);
    }

    #[test]
    fn clusters_split_at_the_tolerance() {
        let mut points: Vec<[f64; 3]> = (0..30).map(|i| [0.01 * i as f64, 0.0, 0.0]).collect();
        points.extend((0..10).map(|i| [1.0 + 0.01 * i as f64, 0.0, 0.0]));
        points.push([5.0, 5.0, 5.0]);
        points.push([f64::NAN; 3]);
        let clusters = euclidean_clusters(&points, 0.015, 2, 1000);
        assert_eq!(clusters.len(), 2);
        assert_eq!(clusters[0], (0..30).collect::<Vec<_>>());
        assert_eq!(clusters[1], (30..40).collect::<Vec<_>>());
        assert_eq!(euclidean_clusters(&points, 0.015, 11, 1000).len(), 1);
    }
}
