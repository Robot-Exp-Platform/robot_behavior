//! Robot state types.
//!
//! The sample types come from `roplat_msgs` (one representation, fixed native
//! layout, `Option` accessors over a validity mask). The composite states for
//! mobile, legged and humanoid robots are contract types here.

pub use roplat_msgs::{
    ContactSample, ContactState, JointSample, JointState, SpatialSample, SpatialState, StateView,
};

/// Pose / twist / wrench state of a floating or mobile base.
pub type BaseState = SpatialState;
/// Pose / twist / wrench state of an end effector (flange, TCP, hand, foot).
pub type EndEffectorState = SpatialState;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MobileBaseState {
    /// Base pose / twist / wrench state.
    pub base: BaseState,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct QuadrupedState<const N: usize> {
    /// Whole-body joint state for the quadruped.
    pub joint: JointState<N>,
    /// Floating or mobile base state.
    pub base: BaseState,
    /// Per-foot pose / twist / wrench state.
    pub feet: [EndEffectorState; 4],
    /// Per-foot contact state.
    pub contacts: [ContactState; 4],
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct HumanoidState<const N: usize> {
    /// Whole-body joint state for the humanoid.
    pub joint: JointState<N>,
    /// Floating or mobile base state.
    pub base: BaseState,
    /// Per-hand pose / twist / wrench state.
    pub hands: [EndEffectorState; 2],
    /// Per-foot pose / twist / wrench state.
    pub feet: [EndEffectorState; 2],
    /// Contact states for feet, hands or other contact surfaces.
    pub contacts: Vec<ContactState>,
}
