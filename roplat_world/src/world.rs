use anyhow::Result;
use nalgebra as na;

use roplat_behavior::RobotDescription;

pub enum Entity<'a> {
    Sphere {
        radius: f64,
    },
    Box {
        half_extents: [f64; 3],
    },
    Capsule {
        radius: f64,
        height: f64,
    },
    Cylinder {
        radius: f64,
        height: f64,
    },
    Plane {
        normal: [f64; 3],
        constant: f64,
    },
    MeshFile {
        file: &'a str,
        scale: [f64; 3],
    },
    ConvexMesh {
        vertices: &'a [[f64; 3]],
        scale: [f64; 3],
    },
    Mesh {
        vertices: &'a [[f64; 3]],
        indices: &'a [i32],
        scale: [f64; 3],
    },
}

pub trait EntityBuilder<'a> {
    type Entity;
    fn name(self, name: String) -> Self;
    fn base(self, base: impl Into<na::Isometry3<f64>>) -> Self;
    fn base_fixed(self, base_fixed: bool) -> Self;
    fn scaling(self, scaling: f64) -> Self;
    fn load(self) -> Result<Self::Entity>;
}

pub type Collision<'a> = Entity<'a>;
pub type Visual<'a> = Entity<'a>;

pub trait AddRobot {
    type PR<R>;
    type RB<'a, R: RobotDescription>: EntityBuilder<'a, Entity = Self::PR<R>>
    where
        Self: 'a;
    fn robot_builder<'a, R: RobotDescription>(&'a mut self, name: impl ToString)
    -> Self::RB<'a, R>;
}

pub trait AddCollision {
    type EntityId: Copy + Eq;
    type CB<'a>: EntityBuilder<'a, Entity = Self::EntityId>
    where
        Self: 'a;
    fn collision<'a>(&'a mut self, collision: Collision<'a>) -> Self::CB<'a>;
}

pub trait AddVisual {
    type EntityId: Copy + Eq;
    type VB<'a>: EntityBuilder<'a, Entity = Self::EntityId>
    where
        Self: 'a;
    fn visual<'a>(&'a mut self, visual: Visual<'a>) -> Self::VB<'a>;
}

/// A URDF shape as an entity. URDF gives a box's full edge lengths and a
/// cylinder's or capsule's length; an entity takes half extents and a height.
impl<'a> From<&'a roplat_description::Geometry> for Entity<'a> {
    fn from(geometry: &'a roplat_description::Geometry) -> Self {
        use roplat_description::Geometry;
        match geometry {
            Geometry::Box { size } => Entity::Box { half_extents: size.map(|s| s / 2.0) },
            Geometry::Cylinder { radius, length } => {
                Entity::Cylinder { radius: *radius, height: *length }
            }
            Geometry::Capsule { radius, length } => {
                Entity::Capsule { radius: *radius, height: *length }
            }
            Geometry::Sphere { radius } => Entity::Sphere { radius: *radius },
            Geometry::Mesh { filename, scale } => {
                Entity::MeshFile { file: filename, scale: *scale }
            }
        }
    }
}
