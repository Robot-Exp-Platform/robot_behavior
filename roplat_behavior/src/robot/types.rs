use serde::{Deserialize, Serialize};

pub use roplat_msgs::Pose;

#[derive(Debug, Default, Serialize, Deserialize, PartialEq, Clone, Copy)]
/// The reference frame in which a Cartesian command is interpreted.
///
/// Parsed from a string via [`From<&str>`](Coord::from): `"OCS"` → [`Coord::OCS`],
/// `"Shot"` → [`Coord::Relative`], `"Inertial"` → [`Coord::Inertial`], and any
/// other string is decoded as a JSON [`Pose`] into [`Coord::Other`].
pub enum Coord {
    /// Object / base coordinate system (the default world frame).
    #[default]
    OCS,
    /// Relative to the robot's current pose (incremental motion).
    Relative,
    /// A fixed inertial frame.
    Inertial,
    /// An arbitrary user-supplied frame given as a [`Pose`].
    Other(Pose),
}

impl From<&str> for Coord {
    fn from(s: &str) -> Self {
        match s {
            "OCS" => Coord::OCS,
            "Shot" => Coord::Relative,
            "Inertial" => Coord::Inertial,
            _ => Coord::Other(serde_json::from_str(s).unwrap()),
        }
    }
}
