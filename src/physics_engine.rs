use crate::{PhysicsEngineResult, RobotFile};
use anyhow::Result;
use nalgebra as na;
use std::{path::Path, time::Duration};

pub trait PhysicsEngine {
    fn reset(&mut self) -> PhysicsEngineResult<()>;
    fn step(&mut self) -> PhysicsEngineResult<()>;
    fn shutdown(self);

    fn set_step_time(&mut self, dt: Duration) -> PhysicsEngineResult<&mut Self>;
    fn set_gravity(&mut self, gravity: impl Into<[f64; 3]>) -> PhysicsEngineResult<&mut Self>;
    fn set_additional_search_path(
        &mut self,
        path: impl AsRef<Path>,
    ) -> PhysicsEngineResult<&mut Self>;
}

pub trait AddRobot {
    type PR<R>;
    type RB<'a, R: RobotFile>: RobotBuilder<'a, R, Self::PR<R>>
    where
        Self: 'a;
    fn robot_builder<'a, R: RobotFile>(&'a mut self, _name: impl ToString) -> Self::RB<'a, R>;
}

pub trait RobotBuilder<'a, R, PR> {
    fn name(self, name: String) -> Self;
    fn mesh_path(self, mesh_path: &'static str) -> Self;
    fn base(self, base: impl Into<na::Isometry3<f64>>) -> Self;
    fn base_fixed(self, base_fixed: bool) -> Self;
    fn scaling(self, scaling: f64) -> Self;
    fn load(self) -> Result<PR>;
}
