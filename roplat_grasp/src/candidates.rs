//! Grasp candidates for primitive shapes.

use nalgebra as na;
use roplat_description::Geometry;

/// One way to grasp an object: where the tool centre point goes and how
/// wide the object is between the fingers there.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Grasp {
    /// Tool centre point in the world: `z` approaches, `y` closes.
    pub pose: na::Isometry3<f64>,
    /// Object width between the fingers (m): the gripper must open wider.
    pub width: f64,
    /// Larger is preferred; candidates are returned in decreasing order.
    pub score: f64,
}

impl Grasp {
    /// Unit approach direction in the world.
    pub fn approach(&self) -> na::Vector3<f64> {
        self.pose.rotation * na::Vector3::z()
    }

    /// The pose `distance` back along the approach, where the straight
    /// approach starts.
    pub fn pregrasp(&self, distance: f64) -> na::Isometry3<f64> {
        self.pose * na::Translation3::new(0.0, 0.0, -distance)
    }
}

/// What the candidate generator knows of the gripper and the search.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GraspOptions {
    /// Widest opening of the fingers (m); wider parts are not candidates.
    pub max_opening: f64,
    /// How far past the face it approaches the tool centre point goes into
    /// the object (m), at most to the object's middle.
    pub depth: f64,
    /// Directions tried around a cylinder or sphere (at least 1).
    pub samples: usize,
    /// Keep approaches that move upward (from below the object). Off by
    /// default: an object usually rests on something.
    pub from_below: bool,
}

impl Default for GraspOptions {
    fn default() -> Self {
        Self {
            max_opening: 0.08,
            depth: 0.02,
            samples: 8,
            from_below: false,
        }
    }
}

/// Candidates for `geometry` placed at `pose` in the world; none for a mesh.
pub fn grasps(
    geometry: &Geometry,
    pose: &na::Isometry3<f64>,
    options: &GraspOptions,
) -> Vec<Grasp> {
    match *geometry {
        Geometry::Box { size } => box_grasps(pose, size, options),
        Geometry::Cylinder { radius, length } | Geometry::Capsule { radius, length } => {
            cylinder_grasps(pose, radius, length, options)
        }
        Geometry::Sphere { radius } => sphere_grasps(pose, radius, options),
        Geometry::Mesh { .. } => Vec::new(),
    }
}

/// A box of `size` (full edge lengths) centred at `pose`: each face
/// approached along its inward normal, the fingers closing along either of
/// the face's two edges, in both finger orders.
pub fn box_grasps(pose: &na::Isometry3<f64>, size: [f64; 3], options: &GraspOptions) -> Vec<Grasp> {
    let axes = [na::Vector3::x(), na::Vector3::y(), na::Vector3::z()];
    let mut out = Vec::new();
    for i in 0..3 {
        let half = size[i] / 2.0;
        for sign in [1.0, -1.0] {
            let normal = pose.rotation * (axes[i] * sign);
            let tcp = pose.translation.vector + normal * (half - options.depth.min(half));
            for j in (0..3).filter(|&j| j != i) {
                let width = size[j];
                if width > options.max_opening {
                    continue;
                }
                let close = pose.rotation * axes[j];
                push_both(&mut out, tcp, -normal, close, width, options);
            }
        }
    }
    sorted(out)
}

/// A cylinder of `radius` and `length` along the `z` of `pose`: from the
/// side at `samples` directions around it, closing across the diameter
/// across the axis; and from each cap at `samples` closing directions.
pub fn cylinder_grasps(
    pose: &na::Isometry3<f64>,
    radius: f64,
    length: f64,
    options: &GraspOptions,
) -> Vec<Grasp> {
    let mut out = Vec::new();
    let width = 2.0 * radius;
    if width > options.max_opening {
        return out;
    }
    let centre = pose.translation.vector;
    let axis = pose.rotation * na::Vector3::z();
    for k in 0..options.samples.max(1) {
        let angle = std::f64::consts::TAU * k as f64 / options.samples.max(1) as f64;
        let radial = pose.rotation * na::Vector3::new(angle.cos(), angle.sin(), 0.0);
        // Side: approach towards the axis, close across it.
        let tcp = centre + radial * (radius - options.depth.min(radius));
        push_both(&mut out, tcp, -radial, axis.cross(&radial), width, options);
        // Caps: approach along the axis, close across the diameter. Only
        // half the turn is distinct for a parallel gripper; push_both adds
        // the other half.
        if 2 * k < options.samples.max(1) {
            for sign in [1.0, -1.0] {
                let half = length / 2.0;
                let normal = axis * sign;
                let tcp = centre + normal * (half - options.depth.min(half));
                push_both(&mut out, tcp, -normal, radial, width, options);
            }
        }
    }
    sorted(out)
}

/// A sphere of `radius` at `pose`: approached from above, from below (if
/// allowed) and from `samples` directions around its equator.
pub fn sphere_grasps(pose: &na::Isometry3<f64>, radius: f64, options: &GraspOptions) -> Vec<Grasp> {
    let mut out = Vec::new();
    let width = 2.0 * radius;
    if width > options.max_opening {
        return out;
    }
    let centre = pose.translation.vector;
    let reach = radius - options.depth.min(radius);
    let mut normals = vec![na::Vector3::z(), -na::Vector3::z()];
    for k in 0..options.samples.max(1) {
        let angle = std::f64::consts::TAU * k as f64 / options.samples.max(1) as f64;
        normals.push(na::Vector3::new(angle.cos(), angle.sin(), 0.0));
    }
    for normal in normals {
        let close = any_perpendicular(&normal);
        push_both(
            &mut out,
            centre + normal * reach,
            -normal,
            close,
            width,
            options,
        );
    }
    sorted(out)
}

/// The grasp with the tool at `tcp` approaching along `approach`, fingers
/// closing along `close`, and the same grasp turned half a turn about the
/// approach (the fingers swapped), which an arm may reach more easily.
fn push_both(
    out: &mut Vec<Grasp>,
    tcp: na::Vector3<f64>,
    approach: na::Vector3<f64>,
    close: na::Vector3<f64>,
    width: f64,
    options: &GraspOptions,
) {
    let z = approach.normalize();
    // Up is +z in the world: an approach moving up comes from below.
    if !options.from_below && z.z > 1e-9 {
        return;
    }
    let y = (close - z * z.dot(&close)).normalize();
    for y in [y, -y] {
        let x = y.cross(&z);
        let rotation = na::Rotation3::from_matrix_unchecked(na::Matrix3::from_columns(&[x, y, z]));
        out.push(Grasp {
            pose: na::Isometry3::from_parts(
                na::Translation3::from(tcp),
                na::UnitQuaternion::from_rotation_matrix(&rotation),
            ),
            width,
            // From straight above scores 1, horizontal 0.
            score: -z.z,
        });
    }
}

fn any_perpendicular(v: &na::Vector3<f64>) -> na::Vector3<f64> {
    let other = if v.x.abs() < 0.9 {
        na::Vector3::x()
    } else {
        na::Vector3::y()
    };
    v.cross(&other).normalize()
}

fn sorted(mut grasps: Vec<Grasp>) -> Vec<Grasp> {
    // Stable: equal scores keep the generation order.
    grasps.sort_by(|a, b| b.score.total_cmp(&a.score));
    grasps
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    fn on_table() -> na::Isometry3<f64> {
        na::Isometry3::translation(0.5, 0.0, 0.24)
    }

    #[test]
    fn box_candidates_respect_the_opening_and_prefer_from_above() {
        // 0.04 x 0.06 x 0.12: only the two narrow sides fit an 0.08 opening.
        let options = GraspOptions::default();
        let all = box_grasps(&on_table(), [0.04, 0.06, 0.12], &options);
        assert!(!all.is_empty());
        assert!(all.iter().all(|g| g.width <= options.max_opening + EPS));
        // Best first: straight down, closing across 0.04 or 0.06.
        let best = all[0];
        assert!((best.approach() - (-na::Vector3::z())).norm() < EPS);
        assert!((best.score - 1.0).abs() < EPS);
        // The tool centre sits `depth` below the top face (z = 0.30).
        assert!((best.pose.translation.z - (0.30 - 0.02)).abs() < EPS);
        // Nothing approaches from below by default.
        assert!(all.iter().all(|g| g.approach().z <= EPS));
        // Scores never increase down the list.
        assert!(all.windows(2).all(|w| w[0].score >= w[1].score));
    }

    #[test]
    fn every_candidate_is_a_proper_tool_frame() {
        let options = GraspOptions { from_below: true, ..GraspOptions::default() };
        let pose = na::Isometry3::new(
            na::Vector3::new(0.1, 0.2, 0.3),
            na::Vector3::new(0.3, -0.2, 0.7),
        );
        let mut all = box_grasps(&pose, [0.03, 0.05, 0.07], &options);
        all.extend(cylinder_grasps(&pose, 0.03, 0.1, &options));
        all.extend(sphere_grasps(&pose, 0.025, &options));
        for g in &all {
            let r = g.pose.rotation.to_rotation_matrix();
            assert!((r.matrix().determinant() - 1.0).abs() < 1e-9);
            // The tool centre is inside or on the object's bounding sphere.
            assert!((g.pose.translation.vector - pose.translation.vector).norm() < 0.1);
        }
        // With approaches from below, a box offers all six faces.
        let faces: std::collections::BTreeSet<[i64; 3]> =
            box_grasps(&na::Isometry3::identity(), [0.03; 3], &options)
                .iter()
                .map(|g| {
                    let a = g.approach();
                    [a.x.round() as i64, a.y.round() as i64, a.z.round() as i64]
                })
                .collect();
        assert_eq!(faces.len(), 6);
    }

    #[test]
    fn cylinder_side_grasps_close_across_the_axis() {
        let options = GraspOptions { samples: 4, ..GraspOptions::default() };
        let all = cylinder_grasps(&on_table(), 0.03, 0.12, &options);
        let side: Vec<_> = all.iter().filter(|g| g.approach().z.abs() < EPS).collect();
        // Four directions, two finger orders each.
        assert_eq!(side.len(), 8);
        for g in side {
            let close = g.pose.rotation * na::Vector3::y();
            assert!(
                close.z.abs() < EPS,
                "closing direction must cross the vertical axis"
            );
            assert!((g.width - 0.06).abs() < EPS);
        }
        // Too wide for the gripper: nothing.
        assert!(cylinder_grasps(&on_table(), 0.05, 0.12, &options).is_empty());
    }

    #[test]
    fn meshes_give_no_candidates() {
        let mesh = Geometry::Mesh { filename: "part.stl".into(), scale: [1.0; 3] };
        assert!(grasps(&mesh, &on_table(), &GraspOptions::default()).is_empty());
        let sphere = Geometry::Sphere { radius: 0.02 };
        assert!(!grasps(&sphere, &on_table(), &GraspOptions::default()).is_empty());
    }
}
