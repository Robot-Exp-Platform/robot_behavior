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
    HeightfieldFile {
        file: &'a str,
        mesh_scale: [f64; 3],
        texture_scaling: f64,
    },
    HeightfieldData {
        mesh_scale: [f64; 3],
        texture_scaling: f64,
        samples: &'a [f32],
        rows: i32,
        columns: i32,
        replace_index: Option<i32>,
    },
}

pub trait EntityBuilder<'a, E> {
    fn name(self, name: String) -> Self;
    fn base(self, base: impl Into<na::Isometry3<f64>>) -> Self;
    fn base_fixed(self, base_fixed: bool) -> Self;
    fn scaling(self, scaling: f64) -> Self;
    fn load(self) -> E;
}

pub type Collision<'a> = Entity<'a>;
pub type Render<'a> = Entity<'a>;

pub trait AddCollision {
    type E;
    type CB<'a>: EntityBuilder<'a, Self::E>
    where
        Self: 'a;
    fn collision<'a>(&'a mut self, collision: Collision) -> Self::CB<'a>;
}

pub trait AddRender {
    type E;
    type RB<'a>: EntityBuilder<'a, Self::E>
    where
        Self: 'a;
    fn render<'a>(&'a mut self, render: Render) -> Self::RB<'a>;
}
