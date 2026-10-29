//! The model files a robot type offers to simulators, declared through
//! [`RobotDescription::SIMULATION_MODELS`](crate::RobotDescription). Choosing one
//! for a backend is `roplat_world::select_simulation_model`.

/// On-disk description format understood by a simulator's robot loader.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ModelFormat {
    Urdf,
    Mjcf,
    Sdf,
}

/// One model offered by a robot description.
///
/// Paths follow the selected backend's asset/search-path rules. Relative paths
/// are not resolved by this crate. A backend restriction is an exact identifier
/// such as `"bullet"` or `"mujoco"`, not a file-format or operating-system claim.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SimulationModel {
    pub format: ModelFormat,
    pub path: &'static str,
    /// `None` offers this model to any backend accepting its format.
    pub backend: Option<&'static str>,
}

impl SimulationModel {
    /// Offer a model to any backend accepting its format.
    pub const fn new(format: ModelFormat, path: &'static str) -> Self {
        Self { format, path, backend: None }
    }

    /// Restrict this model to a backend that needs its particular adaptation.
    pub const fn for_backend(mut self, backend: &'static str) -> Self {
        self.backend = Some(backend);
        self
    }
}
