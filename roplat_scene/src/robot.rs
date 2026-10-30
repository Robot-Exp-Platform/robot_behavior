//! The robot's own geometry: links that are concave, and sensed points
//! that fall on the robot.

use std::path::PathBuf;

use nalgebra as na;
use parry3d_f64::bounding_volume::BoundingVolume;
use roplat_description::Geometry;

use crate::{PlanningScene, Role, SceneError, decompose, mesh, part};

impl<const N: usize> PlanningScene<N> {
    /// Replaces the convex hull of each mesh of `link` by a convex
    /// decomposition (V-HACD), as
    /// [`add_concave_object`](Self::add_concave_object) does for objects,
    /// so a concave link such as an open gripper palm or a hollow tool no
    /// longer fills in. Primitive shapes stay as they are. Returns the
    /// link's number of convex parts. `mesh_path` turns a mesh file name as
    /// written in the description into a path to read, as in
    /// [`new`](Self::new); the mesh's faces are needed.
    ///
    /// Decomposing takes a fraction of a second to seconds per mesh; do it
    /// once, after building the scene. A decomposed link checks more parts,
    /// so do it only for the links whose hollows matter.
    pub fn decompose_link(
        &mut self,
        link: &str,
        mut mesh_path: impl FnMut(&str) -> PathBuf,
    ) -> Result<usize, SceneError> {
        let index = self.link_index(link)?;
        let mut parts = Vec::new();
        let mut count = 0;
        for shape in &self.model.links[index].collisions {
            let origin = shape.origin.quat();
            let Geometry::Mesh { filename, scale } = &shape.geometry else {
                parts.push(part(&shape.geometry, origin, &mut mesh_path)?);
                count += 1;
                continue;
            };
            if !scale.iter().all(|v| v.is_finite() && *v > 0.0) {
                return Err(SceneError::InvalidGeometry(format!(
                    "mesh scale {scale:?} must be positive"
                )));
            }
            let path = mesh_path(filename);
            let failed = |reason: String| SceneError::Mesh { path: path.clone(), reason };
            let (vertices, triangles) = mesh::triangles(&path).map_err(failed)?;
            let vertices: Vec<[f64; 3]> = vertices
                .into_iter()
                .map(|v| [v[0] * scale[0], v[1] * scale[1], v[2] * scale[2]])
                .collect();
            let shape = decompose(&vertices, &triangles).map_err(failed)?;
            count += shape
                .as_compound()
                .map_or(1, |compound| compound.shapes().len());
            parts.push(crate::Part { at: origin, shape });
        }
        self.links[index] = parts;
        Ok(count)
    }

    /// Which of `points` (in the world) lie on the robot at `q`: inside, or
    /// within `margin` (m) of, a link's collision geometry or an object
    /// attached to a link. The role of MoveIt's self filter: remove these
    /// from a depth reading before [`add_points`](Self::add_points), so
    /// the arm in view does not become an obstacle to itself. A margin of a
    /// few centimetres covers the depth noise and the gap between the
    /// collision geometry and the real surface.
    pub fn points_on_robot(&self, points: &[[f64; 3]], q: &[f64; N], margin: f64) -> Vec<bool> {
        let parts: Vec<_> = self
            .bodies(q)
            .into_iter()
            .filter(|body| matches!(body.role, Role::Link(_) | Role::Carried { .. }))
            .flat_map(|body| body.parts.into_iter())
            .map(|(pose, shape)| {
                let bound = shape.compute_aabb(&pose).loosened(margin);
                (pose, shape, bound)
            })
            .collect();
        points
            .iter()
            .map(|p| {
                let point = na::Point3::from(*p);
                parts.iter().any(|(pose, shape, bound)| {
                    bound.contains_local_point(&point)
                        && shape.distance_to_point(pose, &point, true) <= margin
                })
            })
            .collect()
    }

    /// The `points` not on the robot at `q`; see
    /// [`points_on_robot`](Self::points_on_robot).
    pub fn without_robot_points(
        &self,
        points: &[[f64; 3]],
        q: &[f64; N],
        margin: f64,
    ) -> Vec<[f64; 3]> {
        points
            .iter()
            .zip(self.points_on_robot(points, q, margin))
            .filter(|(_, on)| !on)
            .map(|(p, _)| *p)
            .collect()
    }
}
