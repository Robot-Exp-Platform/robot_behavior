use std::{path::Path, time::Duration};

pub trait PhysicsEngine {
    type Error;
    fn reset(&mut self) -> Result<(), Self::Error>;
    fn step(&mut self) -> Result<(), Self::Error>;
    fn shutdown(self);

    fn set_step_time(&mut self, dt: Duration) -> Result<&mut Self, Self::Error>;
    fn set_gravity(&mut self, gravity: impl Into<[f64; 3]>) -> Result<&mut Self, Self::Error>;
}

pub trait AddSearchPath {
    type Error;
    fn add_search_path(&mut self, path: impl AsRef<Path>) -> Result<&mut Self, Self::Error>;
}
