//! The scene as triangles, for a viewer.

use nalgebra as na;
use parry3d_f64::shape::{Shape, ShapeType};

use crate::{PlanningScene, Role};

/// What a drawn body is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyKind {
    /// A link of the robot.
    Link,
    /// An object in the world.
    Object,
    /// An object attached to a link.
    Attached,
}

/// One body's collision geometry as a triangle mesh in the world.
#[derive(Clone, Debug, PartialEq)]
pub struct BodyMesh {
    /// The link's or object's name.
    pub name: String,
    pub kind: BodyKind,
    pub vertices: Vec<[f64; 3]>,
    /// Indices into `vertices`, counter-clockwise seen from outside.
    pub triangles: Vec<[u32; 3]>,
}

/// Facets around a round shape.
const ROUND: u32 = 24;

impl<const N: usize> PlanningScene<N> {
    /// Every link with geometry and every object at `q`, as triangle meshes
    /// in the world: what the collision checks see, for drawing the scene in
    /// a viewer such as Rerun (the role of RViz's planning-scene display).
    /// Boxes, convex hulls and decomposed parts keep their faces; spheres,
    /// cylinders and capsules get 24 facets around.
    pub fn meshes(&self, q: &[f64; N]) -> Vec<BodyMesh> {
        self.bodies(q)
            .into_iter()
            .map(|body| {
                let kind = match body.role {
                    Role::Link(_) => BodyKind::Link,
                    Role::World => BodyKind::Object,
                    Role::Carried { .. } => BodyKind::Attached,
                };
                let mut mesh = BodyMesh {
                    name: body.name.to_string(),
                    kind,
                    vertices: Vec::new(),
                    triangles: Vec::new(),
                };
                for (pose, shape) in &body.parts {
                    tessellate(shape.as_ref(), pose, &mut mesh);
                }
                mesh
            })
            .collect()
    }
}

/// Appends `shape` placed at `pose` to `mesh`; a compound part by part.
fn tessellate(shape: &dyn Shape, pose: &na::Isometry3<f64>, mesh: &mut BodyMesh) {
    let (vertices, triangles) = match shape.shape_type() {
        ShapeType::Compound => {
            let compound = shape.as_compound().expect("a compound");
            for (at, part) in compound.shapes() {
                tessellate(part.as_ref(), &(pose * at), mesh);
            }
            return;
        }
        ShapeType::Cuboid => shape.as_cuboid().expect("a cuboid").to_trimesh(),
        ShapeType::Ball => shape
            .as_ball()
            .expect("a ball")
            .to_trimesh(ROUND / 2, ROUND),
        ShapeType::Cylinder => shape.as_cylinder().expect("a cylinder").to_trimesh(ROUND),
        ShapeType::Capsule => shape
            .as_capsule()
            .expect("a capsule")
            .to_trimesh(ROUND, ROUND / 2),
        ShapeType::ConvexPolyhedron => shape
            .as_convex_polyhedron()
            .expect("a convex polyhedron")
            .to_trimesh(),
        // The scene builds no other shape; draw its bounding box.
        _ => {
            let aabb = shape.compute_local_aabb();
            let centre = aabb.center();
            let cuboid = parry3d_f64::shape::Cuboid::new(aabb.half_extents());
            let (vertices, triangles) = cuboid.to_trimesh();
            let shifted = vertices.into_iter().map(|v| v + centre.coords).collect();
            (shifted, triangles)
        }
    };
    let offset = mesh.vertices.len() as u32;
    mesh.vertices.extend(
        vertices
            .into_iter()
            .map(|v| -> [f64; 3] { (pose * v).coords.into() }),
    );
    mesh.triangles
        .extend(triangles.into_iter().map(|t| t.map(|i| i + offset)));
}
