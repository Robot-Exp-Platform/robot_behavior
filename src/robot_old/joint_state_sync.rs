use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use crate::ArmState;

/// State of a single joint, identified by name.
#[derive(Debug, Clone, Default)]
pub struct JointStateEntry {
    pub position: f64,
    pub velocity: Option<f64>,
    pub acceleration: Option<f64>,
    pub torque: Option<f64>,
}

/// A shared, lock-protected map from joint name to [`JointStateEntry`].
///
/// Producers (e.g. real robot drivers) write into this handle every control
/// cycle; consumers (e.g. simulation mirrors) read from it to stay
/// synchronized.
pub type JointStateMap = Arc<Mutex<HashMap<String, JointStateEntry>>>;

/// Build a [`HashMap<String, JointStateEntry>`] from joint names and an
/// [`ArmState`].
///
/// This is the canonical conversion used by robot drivers when populating a
/// [`JointStateMap`].
pub fn joint_state_map_from_arm_state<const N: usize>(
    names: &[&str; N],
    state: &ArmState<N>,
) -> HashMap<String, JointStateEntry> {
    let positions = state.measured.joint.unwrap_or([0.0; N]);
    let velocities = state.measured.joint_vel;
    let accelerations = state.measured.joint_acc;
    let torques = state.measured.torque;

    names
        .iter()
        .enumerate()
        .map(|(i, name)| {
            (
                name.to_string(),
                JointStateEntry {
                    position: positions[i],
                    velocity: velocities.map(|v| v[i]),
                    acceleration: accelerations.map(|a| a[i]),
                    torque: torques.map(|t| t[i]),
                },
            )
        })
        .collect()
}

/// Update an existing [`JointStateMap`] in-place from joint names and an
/// [`ArmState`], avoiding allocation when the map already contains the
/// expected keys.
pub fn update_joint_state_map<const N: usize>(
    map: &JointStateMap,
    names: &[&str],
    state: &ArmState<N>,
) {
    let positions = state.measured.joint.unwrap_or([0.0; N]);
    let velocities = state.measured.joint_vel;
    let accelerations = state.measured.joint_acc;
    let torques = state.measured.torque;

    if let Ok(mut guard) = map.lock() {
        for (i, name) in names.iter().enumerate() {
            let entry = guard
                .entry(name.to_string())
                .or_insert_with(JointStateEntry::default);
            entry.position = positions[i];
            entry.velocity = velocities.map(|v| v[i]);
            entry.acceleration = accelerations.map(|a| a[i]);
            entry.torque = torques.map(|t| t[i]);
        }
    }
}

/// Trait for robots that can expose a live joint-state map.
///
/// Implement this on a real robot driver so that a simulation mirror can
/// call [`joint_state_handle`](JointStateSync::joint_state_handle) once,
/// then read updated joint states every cycle without any further
/// cooperation from the caller.
pub trait JointStateSync {
    /// Return a shared handle that is kept up-to-date by the implementor.
    fn joint_state_handle(&self) -> JointStateMap;
}
