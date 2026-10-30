//! Objects on a table, the perception step of a tabletop pick: the
//! pipeline of ROS's tabletop object detector (and of PCL's cluster
//! extraction tutorial) in one call.
//!
//! 1. Downsample to an even density ([`voxel_downsample`]).
//! 2. Find the table: the plane through the most points, level within a
//!    tilt of `up` ([`fit_plane`]).
//! 3. Keep the points above it whose projection falls on the table's
//!    outline (PCL's polygonal prism), between a least and a largest
//!    height, so the table edge, the floor and walls drop out.
//! 4. Group them into objects standing apart ([`euclidean_clusters`]).
//! 5. Fit each a box or an upright cylinder resting on the table
//!    ([`fit_on`]).
//!
//! The points must be in a frame whose `up` is known, such as the world
//! or the robot base, after moving them from the camera frame with the
//! camera's pose.

use nalgebra as na;

use crate::{
    cloud::voxel_downsample,
    fit::{FittedShape, convex_hull, fit_on},
    segment::{Plane, PlaneOptions, euclidean_clusters, fit_plane},
};

/// Settings for [`segment_tabletop`].
#[derive(Clone, Copy, Debug)]
pub struct TabletopOptions {
    /// The direction away from gravity in the points' frame.
    pub up: na::Vector3<f64>,
    /// Voxel edge (m) for downsampling; 0 keeps every point.
    pub voxel: f64,
    /// Largest distance (m) of a point counted on the table.
    pub table_threshold: f64,
    /// Largest angle (rad) between the table's normal and `up`.
    pub max_tilt: f64,
    /// Points from this height (m) above the table up to `max_height`
    /// belong to objects. Below it is the table's own noise.
    pub min_height: f64,
    pub max_height: f64,
    /// Largest gap (m) within one object; objects closer than this merge.
    pub cluster_tolerance: f64,
    /// Points an object needs after downsampling; smaller clusters are
    /// noise.
    pub min_points: usize,
    /// RANSAC samples for the table.
    pub iterations: usize,
    pub seed: u64,
}

impl Default for TabletopOptions {
    fn default() -> Self {
        Self {
            up: na::Vector3::z(),
            voxel: 0.005,
            table_threshold: 0.01,
            max_tilt: 0.2,
            min_height: 0.01,
            max_height: 0.5,
            cluster_tolerance: 0.02,
            min_points: 20,
            iterations: 300,
            seed: 1,
        }
    }
}

/// The table and the objects on it.
#[derive(Clone, Debug, PartialEq)]
pub struct Tabletop {
    /// The table's plane, its normal along `up`.
    pub table: Plane,
    /// Corners of the table's outline (the convex hull of its points), on
    /// its plane, counter-clockwise seen from above.
    pub outline: Vec<[f64; 3]>,
    /// Largest first.
    pub objects: Vec<TabletopObject>,
}

/// One object on the table.
#[derive(Clone, Debug, PartialEq)]
pub struct TabletopObject {
    /// Its downsampled points.
    pub points: Vec<[f64; 3]>,
    /// The box or upright cylinder resting on the table that fits it.
    pub shape: FittedShape,
}

#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum TabletopError {
    #[error("no plane within the tilt of up")]
    NoTable,
}

/// Finds the table under `points` and the objects standing on it; see the
/// [module](self) docs.
///
/// # Panics
///
/// If `options.voxel` is negative.
pub fn segment_tabletop(
    points: &[[f64; 3]],
    options: &TabletopOptions,
) -> Result<Tabletop, TabletopError> {
    assert!(
        options.voxel >= 0.0,
        "voxel edge {} must not be negative",
        options.voxel
    );
    let points = if options.voxel > 0.0 {
        voxel_downsample(points, options.voxel)
    } else {
        points.to_vec()
    };
    let plane_options = PlaneOptions {
        threshold: options.table_threshold,
        iterations: options.iterations,
        up: Some(options.up),
        max_tilt: options.max_tilt,
        seed: options.seed,
    };
    let fit = fit_plane(&points, &plane_options).ok_or(TabletopError::NoTable)?;
    let table = fit.plane;
    let frame = table.frame();
    let inverse = frame.inverse();
    let flat = |p: &[f64; 3]| {
        let local = inverse * na::Point3::from(*p);
        na::Point2::new(local.x, local.y)
    };
    let outline = convex_hull(fit.inliers.iter().map(|&i| flat(&points[i])).collect());
    let above: Vec<[f64; 3]> = points
        .iter()
        .filter(|p| {
            let height = table.signed_distance(p);
            (options.min_height..=options.max_height).contains(&height)
                && inside(&outline, &flat(p))
        })
        .copied()
        .collect();
    let clusters = euclidean_clusters(
        &above,
        options.cluster_tolerance,
        options.min_points,
        usize::MAX,
    );
    let objects = clusters
        .into_iter()
        .filter_map(|cluster| {
            let points: Vec<[f64; 3]> = cluster.iter().map(|&i| above[i]).collect();
            let shape = fit_on(&points, &table)?;
            Some(TabletopObject { points, shape })
        })
        .collect();
    let outline = outline
        .iter()
        .map(|p| (frame * na::Point3::new(p.x, p.y, 0.0)).coords.into())
        .collect();
    Ok(Tabletop { table, outline, objects })
}

/// Whether `p` is inside or on a counter-clockwise convex polygon.
fn inside(polygon: &[na::Point2<f64>], p: &na::Point2<f64>) -> bool {
    polygon.len() >= 3
        && (0..polygon.len()).all(|k| {
            let (a, b) = (polygon[k], polygon[(k + 1) % polygon.len()]);
            (b - a).perp(&(p - a)) >= 0.0
        })
}
