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
//!
//! A depth camera beside a colour camera sees from a slightly different
//! place: [`register_depth`] re-projects its image into the colour
//! camera, so that each colour pixel has a depth, as ROS's
//! `depth_image_proc/register` does.

use std::{collections::BTreeSet, ops::RangeInclusive};

use nalgebra as na;
use roplat_msgs::CameraInfo;

/// Back-projects a depth image through the pinhole intrinsics `info.k` into
/// points in the camera's optical frame: `x` right, `y` down, `z` along the
/// view.
///
/// `depth` holds metres, row-major, `info.width × info.height` values. A
/// pixel outside `range`, or not finite (no return), gives no point; every
/// `stride`-th pixel in each direction is used (1 for all). Distortion is
/// not removed: give a rectified image, as simulators and most depth
/// cameras' aligned streams are (`roplat_frames::lens::Rectify::nearest`
/// rectifies a raw one).
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

/// `depth` (as for [`depth_to_points`]) re-projected into the camera
/// `target`, whose optical frame holds the depth camera's optical frame at
/// `depth_in_target`: for each of `target`'s pixels, row-major, the depth
/// along its optical axis of the nearest surface the depth camera saw
/// there, NaN where it saw none. Both images must be rectified.
///
/// Each depth pixel's footprint is projected and fills the target pixels
/// whose centres it covers, so a finer target image has no gaps; where
/// several surfaces land on one pixel the nearest wins. Pixels the depth
/// camera could not see (in the shadow of a nearer object, from the
/// target's viewpoint) stay NaN.
pub fn register_depth(
    depth: &[f32],
    info: &CameraInfo,
    target: &CameraInfo,
    depth_in_target: &na::Isometry3<f64>,
) -> Vec<f32> {
    let (width, height) = (info.width as usize, info.height as usize);
    let (tw, th) = (target.width as usize, target.height as usize);
    let mut registered = vec![f32::NAN; tw * th];
    let (fx, fy, cx, cy) = (info.k[0], info.k[4], info.k[2], info.k[5]);
    let (tfx, tfy, tcx, tcy) = (target.k[0], target.k[4], target.k[2], target.k[5]);
    if !(fx > 0.0 && fy > 0.0 && tfx > 0.0 && tfy > 0.0) || depth.len() < width * height {
        return registered;
    }
    // A point in the depth camera's frame, as target pixel coordinates and
    // depth.
    let project = |u: f64, v: f64, z: f64| {
        let p = depth_in_target * na::Point3::new((u - cx) * z / fx, (v - cy) * z / fy, z);
        (p.z > 0.0).then(|| (tfx * p.x / p.z + tcx, tfy * p.y / p.z + tcy, p.z))
    };
    for v in 0..height {
        for u in 0..width {
            let z = f64::from(depth[v * width + u]);
            if !(z.is_finite() && z > 0.0) {
                continue;
            }
            let (u, v) = (u as f64, v as f64);
            let Some((_, _, along)) = project(u, v, z) else {
                continue;
            };
            let corners = [
                project(u - 0.5, v - 0.5, z),
                project(u + 0.5, v - 0.5, z),
                project(u - 0.5, v + 0.5, z),
                project(u + 0.5, v + 0.5, z),
            ];
            let Some(corners) = corners.into_iter().collect::<Option<Vec<_>>>() else {
                continue;
            };
            let (lo_u, hi_u) = corners
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), c| {
                    (lo.min(c.0), hi.max(c.0))
                });
            let (lo_v, hi_v) = corners
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), c| {
                    (lo.min(c.1), hi.max(c.1))
                });
            // Target pixels whose centres lie in [lo, hi).
            let first_u = lo_u.ceil().max(0.0) as usize;
            let first_v = lo_v.ceil().max(0.0) as usize;
            let end_u = (hi_u.ceil().max(0.0) as usize).min(tw);
            let end_v = (hi_v.ceil().max(0.0) as usize).min(th);
            for tv in first_v..end_v {
                for tu in first_u..end_u {
                    let cell = &mut registered[tv * tw + tu];
                    if cell.is_nan() || (along as f32) < *cell {
                        *cell = along as f32;
                    }
                }
            }
        }
    }
    registered
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

    /// The depth along `info`'s optical axis of a wall at z = 1 and a
    /// 20 cm square board at z = 0.6 centred on the axis, both facing the
    /// camera, for a camera at `x` on the x axis looking along z.
    fn render(info: &CameraInfo, x: f64) -> Vec<f32> {
        let (fx, fy, cx, cy) = (info.k[0], info.k[4], info.k[2], info.k[5]);
        let mut depth = Vec::new();
        for v in 0..info.height as usize {
            for u in 0..info.width as usize {
                let (dx, dy) = ((u as f64 - cx) / fx, (v as f64 - cy) / fy);
                let on_board = (x + dx * 0.6).abs() < 0.1 && (dy * 0.6).abs() < 0.1;
                depth.push(if on_board { 0.6 } else { 1.0 });
            }
        }
        depth
    }

    #[test]
    fn registered_depth_matches_what_the_target_would_see() {
        // The depth camera 5 cm to the right of the colour camera.
        let depth_info = camera(160, 120, 150.0);
        let depth = render(&depth_info, 0.05);
        let depth_in_colour = na::Isometry3::translation(0.05, 0.0, 0.0);
        for colour in [camera(160, 120, 150.0), camera(320, 240, 300.0)] {
            let registered = register_depth(&depth, &depth_info, &colour, &depth_in_colour);
            let truth = render(&colour, 0.0);
            let scale = colour.width as usize / 160;
            let (mut wrong, mut missing) = (0, 0);
            for (k, (&r, &t)) in registered.iter().zip(&truth).enumerate() {
                if r.is_nan() {
                    missing += 1;
                } else if (r - t).abs() > 1e-6 {
                    wrong += 1;
                    // Only along the board's two vertical edges.
                    let u = (k % colour.width as usize) as f64;
                    let edge = colour.k[2] + 0.1 / 0.6 * colour.k[0];
                    let other = colour.k[2] - 0.1 / 0.6 * colour.k[0];
                    assert!(
                        (u - edge).abs().min((u - other).abs()) < 1.5,
                        "pixel {k}: {r} for {t}"
                    );
                }
            }
            // The wall left of the board, which the depth camera on the
            // right cannot see behind it: 0.05 m x (1/0.6 - 1) wide at 1 m,
            // over the board's height, plus the image's right margin that
            // lies outside the depth camera's view.
            let shadow = 0.05 * (1.0 / 0.6 - 1.0) * colour.k[0] * (0.2 / 0.6 * colour.k[4]);
            let margin = 0.05 * colour.k[0] * colour.height as f64;
            eprintln!(
                "scale {scale}: {missing} missing (expected {:.0}), {wrong} wrong of {}",
                shadow + margin,
                registered.len()
            );
            assert!(
                (missing as f64 - shadow - margin).abs() < colour.height as f64,
                "{missing} missing at scale {scale}, expected about {}",
                shadow + margin
            );
            assert!(
                wrong < 2 * colour.height as usize,
                "{wrong} wrong at scale {scale}"
            );
        }
        // Nothing for a camera without a focal length or a short image.
        assert!(
            register_depth(&depth, &camera(10, 10, 0.0), &depth_info, &depth_in_colour)
                .iter()
                .all(|d| d.is_nan())
        );
        assert!(
            register_depth(&depth[..10], &depth_info, &depth_info, &depth_in_colour)
                .iter()
                .all(|d| d.is_nan())
        );
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
