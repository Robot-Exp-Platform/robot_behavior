//! Reusable realtime controller closure builders.
//!
//! The functions in this module deliberately return plain `FnMut` closures
//! instead of controller structs. A driver can pass the returned closure
//! directly to `control_with::<...>()`, while higher-level code can
//! compose these closures with path generators, model maps or session handles.
//!
//! Joint-space controllers observe [`JointState`](crate::JointState), so the
//! same helper works for a serial arm, a humanoid joint set or a quadruped
//! whole-body joint vector. Controllers that need Cartesian pose, Jacobians or
//! full arm metadata observe [`ArmState`](crate::ArmState) and should be used
//! with [`ArmTorqueControl`](crate::ArmTorqueControl).
pub mod dynamics;
pub mod impedance;
pub mod pid;

pub use dynamics::*;
pub use impedance::*;
pub use pid::*;
