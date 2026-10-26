//! Robot-model selection for a particular simulator loader.
//!
//! Selection intersects a robot's available descriptions with the formats the
//! backend actually accepts. It does not convert models, inspect files, or prove
//! that their joints, actuators, sensors, or dynamics are compatible.

use std::fmt;

use crate::RobotDescription;

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

/// Why a model was selected, for configuration and experiment provenance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModelSource {
    BackendSpecific,
    Portable,
    LegacyUrdf,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SelectedSimulationModel {
    pub model: SimulationModel,
    pub source: ModelSource,
}

/// No declared model intersects the backend's loader formats and identifier.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ModelSelectionError {
    pub robot: &'static str,
    pub backend: &'static str,
    pub supported_formats: &'static [ModelFormat],
}

impl fmt::Display for ModelSelectionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "robot {} has no model for backend {:?} accepting {:?}",
            self.robot, self.backend, self.supported_formats
        )
    }
}

impl std::error::Error for ModelSelectionError {}

/// Metadata for a specific backend's robot loader.
///
/// Implement this on the loader/engine adapter, not the physical robot type.
/// `MODEL_FORMATS` lists only formats accepted by that adapter's robot-loading
/// entry point, in preference order. A lower-level scene API may support more.
/// Physics stepping remains the independent [`crate::PhysicsEngine`] contract;
/// robot operations remain individual motion/control/state capabilities.
pub trait SimulationBackend {
    const BACKEND_ID: &'static str;
    const MODEL_FORMATS: &'static [ModelFormat];

    fn select_robot_model<R: RobotDescription>()
    -> Result<SelectedSimulationModel, ModelSelectionError> {
        select_simulation_model::<R>(Self::BACKEND_ID, Self::MODEL_FORMATS)
    }
}

/// Select a declared model without doing I/O or mutating a simulator.
///
/// Backend-specific declarations win over portable declarations. Within each
/// tier, the backend's format order wins, then the robot's declaration order.
/// Existing [`RobotDescription::URDF`] is the last fallback, and only when URDF
/// is accepted. No compatible declaration returns an error before loading.
/// A selected file's load failure must be reported by the backend; it must not
/// silently fall through to a different model with potentially different physics.
pub fn select_simulation_model<R: RobotDescription>(
    backend: &'static str,
    supported_formats: &'static [ModelFormat],
) -> Result<SelectedSimulationModel, ModelSelectionError> {
    for (restriction, source) in [
        (Some(backend), ModelSource::BackendSpecific),
        (None, ModelSource::Portable),
    ] {
        for format in supported_formats {
            if let Some(model) = R::SIMULATION_MODELS
                .iter()
                .find(|model| model.backend == restriction && model.format == *format)
            {
                return Ok(SelectedSimulationModel { model: *model, source });
            }
        }
    }
    if supported_formats.contains(&ModelFormat::Urdf)
        && let Some(path) = R::URDF
    {
        return Ok(SelectedSimulationModel {
            model: SimulationModel::new(ModelFormat::Urdf, path),
            source: ModelSource::LegacyUrdf,
        });
    }
    Err(ModelSelectionError {
        robot: std::any::type_name::<R>(),
        backend,
        supported_formats,
    })
}
