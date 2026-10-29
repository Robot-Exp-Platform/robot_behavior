//! Python and C++ projections of the `roplat_behavior` contracts, for driver
//! packages used without the roplat runtime.
//!
//! A driver enables feature `py` or `cxx` and calls the macros (`py_robot!`,
//! `py_arm!`, `py_joint_motion!`, …, `cxx_robot_api!`, …) on its robot type.
//! The macros expand to `pyo3` / `cxx` code in the driver, so the driver
//! depends on the same `pyo3` and `cxx` versions as this crate.
//!
//! The bindings follow the contract: when `roplat_behavior` changes its minor
//! version, this crate releases the matching one, and `pyo3` or `cxx` upgrades
//! wait for such a release.

#[cfg(feature = "cxx")]
pub mod cxx_api;
#[cfg(feature = "cxx")]
mod cxx_types;
#[cfg(feature = "py")]
pub mod py;
#[cfg(feature = "py")]
mod py_types;

#[cfg(feature = "cxx")]
pub use cxx_api::*;
#[cfg(feature = "cxx")]
#[allow(unused_imports)]
pub use cxx_types::types::*;
#[cfg(feature = "py")]
pub use py::*;
#[cfg(feature = "py")]
pub use py_types::types::*;

/// A [`RobotException`](roplat_behavior::RobotException) as a Python
/// exception. A `From` impl cannot live in either crate (both types are
/// foreign to one of them), so the macros call this.
#[cfg(feature = "py")]
pub fn py_err(error: roplat_behavior::RobotException) -> pyo3::PyErr {
    pyo3::exceptions::PyException::new_err(error.to_string())
}

/// Paths the macros expand to.
#[doc(hidden)]
pub mod __private {
    pub use roplat_behavior;
}
