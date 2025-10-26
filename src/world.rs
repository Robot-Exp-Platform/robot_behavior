use anyhow::Result;
use nalgebra as na;

use crate::RobotFile;

pub trait World {}

pub trait AddRobot {
    type PR<R>;
    type RB<'a, R: RobotFile>: RobotBuilder<'a, R, Self::PR<R>>
    where
        Self: 'a;
    fn robot_builder<'a, R: RobotFile>(&'a mut self, name: impl ToString) -> Self::RB<'a, R>;
}

pub trait RobotBuilder<'a, R, PR> {
    fn name(self, name: String) -> Self;
    fn mesh_path(self, mesh_path: &'static str) -> Self;
    fn base(self, base: impl Into<na::Isometry3<f64>>) -> Self;
    fn base_fixed(self, base_fixed: bool) -> Self;
    fn scaling(self, scaling: f64) -> Self;
    fn load(self) -> Result<PR>;
}

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
    type EntityId: Copy + Eq;
    fn name(self, name: String) -> Self;
    fn base(self, base: impl Into<na::Isometry3<f64>>) -> Self;
    fn base_fixed(self, base_fixed: bool) -> Self;
    fn scaling(self, scaling: f64) -> Self;
    fn load(self) -> Result<Self::EntityId>;
}

pub type Collision<'a> = Entity<'a>;
pub type Visual<'a> = Entity<'a>;

pub trait AddCollision {
    type EntityId: Copy + Eq;
    type CB<'a>: EntityBuilder<'a, EntityId = Self::EntityId>
    where
        Self: 'a;
    fn collision<'a>(&'a mut self, collision: Collision<'a>) -> Self::CB<'a>;
}

pub trait AddVisual {
    type EntityId: Copy + Eq;
    type VB<'a>: EntityBuilder<'a, EntityId = Self::EntityId>
    where
        Self: 'a;
    fn visual<'a>(&'a mut self, visual: Visual<'a>) -> Self::VB<'a>;
}
