//! roplat adapters for robot_behavior capabilities.
//!
//! This module keeps roplat-specific execution glue out of the core behavior
//! traits. Drivers still implement `Robot`, `MoveTo`, `ControlWith`, `SpaceMap`
//! and related traits; these adapters make those capabilities usable as roplat
//! `Rhythm` domains and `Node`s.

pub mod node;
pub mod rhythm;

pub use node::*;
pub use rhythm::*;
