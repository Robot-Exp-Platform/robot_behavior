//! Sensed geometry: depth images to points, points to scene obstacles.
//!
//! The role of MoveIt's occupancy map: what a depth camera sees becomes an
//! obstacle the planners avoid, without a model of it.
//!
//! ```
//! use nalgebra::Isometry3;
//! use roplat_msgs::CameraInfo;
//! use roplat_scene::points::depth_to_points;
//!
//! // A 4 x 3 depth image of a wall 1 m away, focal length 2 px.
//! let info = CameraInfo { width: 4, height: 3, k: [2.0, 0.0, 1.5, 0.0, 2.0, 1.0, 0.0, 0.0, 1.0], ..Default::default() };
//! let points = depth_to_points(&[1.0; 12], &info, 0.1..=5.0, 1);
//! assert_eq!(points.len(), 12);
//! assert!(points.iter().all(|p| (p[2] - 1.0).abs() < 1e-12));
//! ```
//!
//! [`PlanningScene::add_points`](crate::PlanningScene::add_points) then
//! adds them, as occupied voxels, to the planning scene.

use std::{collections::BTreeSet, ops::RangeInclusive};

use roplat_msgs::CameraInfo;

/// Back-projects a depth image through the pinhole intrinsics `info.k` into
/// points in the camera's optical frame: `x` right, `y` down, `z` along the
/// view.
///
/// `depth` holds metres, row-major, `info.width × info.height` values. A
/// pixel outside `range`, or not finite (no return), gives no point; every
/// `stride`-th pixel in each direction is used (1 for all). Distortion is
/// not removed: give a rectified image, as simulators and most depth
/// cameras' aligned streams are.
pub fn depth_to_points(
    depth: &[f32],
    info: &CameraInfo,
    range: RangeInclusive<f64>,
    stride: usize,
) -> Vec<[f64; 3]> {
    let (width, height) = (info.width as usize, info.height as usize);
    let (fx, fy, cx, cy) = (info.k[0], info.k[4], info.k[2], info.k[5]);
    let stride = stride.max(1);
    let mut points = Vec::new();
    if fx == 0.0 || fy == 0.0 || depth.len() < width * height {
        return points;
    }
    for v in (0..height).step_by(stride) {
        for u in (0..width).step_by(stride) {
            let z = f64::from(depth[v * width + u]);
            if !z.is_finite() || !range.contains(&z) {
                continue;
            }
            points.push([(u as f64 - cx) * z / fx, (v as f64 - cy) * z / fy, z]);
        }
    }
    points
}

/// The voxels of edge `voxel` that contain at least one of `points`, as
/// integer cell coordinates; the cell `(i, j, k)` spans
/// `[i·voxel, (i+1)·voxel)` on each axis.
pub fn occupied_voxels(points: &[[f64; 3]], voxel: f64) -> BTreeSet<[i64; 3]> {
    points
        .iter()
        .filter(|p| p.iter().all(|c| c.is_finite()))
        .map(|p| p.map(|c| (c / voxel).floor() as i64))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn camera(width: u32, height: u32, f: f64) -> CameraInfo {
        let (cx, cy) = ((width as f64 - 1.0) / 2.0, (height as f64 - 1.0) / 2.0);
        CameraInfo {
            width,
            height,
            k: [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0],
            ..Default::default()
        }
    }

    #[test]
    fn back_projection_follows_the_pinhole_model() {
        let info = camera(5, 5, 4.0);
        let mut depth = [2.0_f32; 25];
        depth[0] = f32::NAN; // no return
        depth[24] = 9.0; // beyond range
        let points = depth_to_points(&depth, &info, 0.1..=5.0, 1);
        assert_eq!(points.len(), 23);
        // Top-right pixel (u = 4, v = 0): x = (4 - 2) * 2 / 4 = 1, y = -1.
        assert!(
            points
                .iter()
                .any(|p| (p[0] - 1.0).abs() < 1e-12 && (p[1] + 1.0).abs() < 1e-12)
        );
        // The centre pixel is on the axis.
        assert!(
            points
                .iter()
                .any(|p| p[0].abs() < 1e-12 && p[1].abs() < 1e-12 && (p[2] - 2.0).abs() < 1e-12)
        );
        // Every second pixel each way: 3 x 3, minus both bad corners.
        assert_eq!(depth_to_points(&depth, &info, 0.1..=5.0, 2).len(), 7);
        // A short buffer or no focal length gives nothing.
        assert!(depth_to_points(&depth[..10], &info, 0.1..=5.0, 1).is_empty());
        assert!(depth_to_points(&depth, &camera(5, 5, 0.0), 0.1..=5.0, 1).is_empty());
    }

    #[test]
    fn voxels_merge_nearby_points() {
        let points = [
            [0.01, 0.01, 0.01],
            [0.04, 0.02, 0.03],
            [0.06, 0.0, 0.0],
            [-0.01, 0.0, 0.0],
            [f64::NAN, 0.0, 0.0],
        ];
        let cells = occupied_voxels(&points, 0.05);
        assert_eq!(
            cells.into_iter().collect::<Vec<_>>(),
            vec![[-1, 0, 0], [0, 0, 0], [1, 0, 0]]
        );
    }
}
