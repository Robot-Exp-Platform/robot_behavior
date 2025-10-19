use nalgebra as na;
use std::time::Duration;

use crate::{PhysicsEngineResult, RobotFile};

pub trait PhysicsEngine {
    fn reset(&mut self) -> PhysicsEngineResult<()>;
    fn step(&mut self) -> PhysicsEngineResult<()>;
    fn shutdown(self);

    fn set_step_time(&mut self, dt: Duration) -> PhysicsEngineResult<&mut Self>;
    fn set_gravity(&mut self, gravity: impl Into<[f64; 3]>) -> PhysicsEngineResult<&mut Self>;
}

pub trait PhysicsEngineRobot {
    type PR<R>;
    type RB<'a, R: RobotFile>: RobotBuilder<'a, R, Self::PR<R>>
    where
        Self: 'a;
    fn robot_builder<'a, R: RobotFile>(&'a mut self) -> Self::RB<'a, R>;
}

pub trait RobotBuilder<'a, R, PR> {
    fn base(self, base: na::Isometry3<f64>) -> Self;
    fn base_fixed(self, base_fixed: bool) -> Self;
    fn scaling(self, scaling: f64) -> Self;
    fn load(self) -> PhysicsEngineResult<PR>;
}
