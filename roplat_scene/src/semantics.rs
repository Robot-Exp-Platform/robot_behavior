//! What the description does not say: named configurations and end
//! effectors, the part of MoveIt's SRDF that planning code asks for.
//!
//! Both are built in code, like the scene's planning group and
//! [`AllowedCollisions`](crate::AllowedCollisions); nothing reads SRDF.
//!
//! ```
//! use nalgebra::{Isometry3, Vector3};
//! use roplat_scene::{EndEffector, NamedPoses};
//!
//! let mut poses = NamedPoses::<2>::new();
//! poses.insert("home", [0.0, 0.0]);
//! poses.insert("ready", [0.3, -1.2]);
//! assert_eq!(poses.get("ready"), Some([0.3, -1.2]));
//! assert_eq!(poses.nearest(&[0.25, -1.0]).map(|(name, _)| name), Some("ready"));
//!
//! // A parallel gripper whose fingertip centre is 0.1 m out of the flange.
//! let hand = EndEffector::new("hand", "flange", Isometry3::translation(0.0, 0.0, 0.1))
//!     .with_links(["left_finger", "right_finger"]);
//! assert_eq!(hand.links(), ["left_finger", "right_finger"]);
//! ```

use std::collections::BTreeMap;

use nalgebra as na;

use crate::{CollisionCheck, PlanningScene, SceneError};

/// Joint configurations of one planning group, by name (`home`, `ready`,
/// `stow`, ...), in the group's joint order.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NamedPoses<const N: usize> {
    poses: BTreeMap<String, [f64; N]>,
}

impl<const N: usize> NamedPoses<N> {
    pub fn new() -> Self {
        Self { poses: BTreeMap::new() }
    }

    /// Adds or replaces `name`; returns the configuration it replaced.
    pub fn insert(&mut self, name: &str, q: [f64; N]) -> Option<[f64; N]> {
        self.poses.insert(name.to_owned(), q)
    }

    pub fn remove(&mut self, name: &str) -> Option<[f64; N]> {
        self.poses.remove(name)
    }

    pub fn get(&self, name: &str) -> Option<[f64; N]> {
        self.poses.get(name).copied()
    }

    /// The names in sorted order.
    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.poses.keys().map(String::as_str)
    }

    /// The named configuration closest to `q` by the largest per-joint
    /// difference, the metric the planners in `roplat_trajectory` use.
    pub fn nearest(&self, q: &[f64; N]) -> Option<(&str, [f64; N])> {
        self.poses
            .iter()
            .map(|(name, p)| (name.as_str(), *p, max_difference(p, q)))
            .min_by(|a, b| a.2.total_cmp(&b.2))
            .map(|(name, p, _)| (name, p))
    }

    /// The names whose configuration is collision-free in `scene`; a pose
    /// saved before objects were added may no longer be.
    pub fn free_in<'a>(&'a self, scene: &'a PlanningScene<N>) -> impl Iterator<Item = &'a str> {
        self.poses
            .iter()
            .filter(|(_, q)| scene.is_free(q))
            .map(|(name, _)| name.as_str())
    }
}

/// A tool mounted on a link: where its working point (tool centre point) is
/// and which links belong to it.
///
/// The tool frame follows the convention of `roplat_grasp`: `z` points out
/// of the tool, the direction it approaches along; for a parallel gripper
/// the fingers close along `y`.
#[derive(Clone, Debug, PartialEq)]
pub struct EndEffector {
    name: String,
    link: String,
    tcp: na::Isometry3<f64>,
    links: Vec<String>,
}

impl EndEffector {
    /// `tcp` is the tool centre point in `link`'s frame.
    pub fn new(name: &str, link: &str, tcp: na::Isometry3<f64>) -> Self {
        Self {
            name: name.to_owned(),
            link: link.to_owned(),
            tcp,
            links: Vec::new(),
        }
    }

    /// The tool's own links (fingers, palm), which may touch a grasped or
    /// attached object.
    pub fn with_links<'a>(mut self, links: impl IntoIterator<Item = &'a str>) -> Self {
        self.links = links.into_iter().map(str::to_owned).collect();
        self
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    /// The link the tool is mounted on.
    pub fn link(&self) -> &str {
        &self.link
    }

    /// The tool centre point in [`link`](Self::link)'s frame.
    pub fn tcp(&self) -> na::Isometry3<f64> {
        self.tcp
    }

    pub fn links(&self) -> Vec<&str> {
        self.links.iter().map(String::as_str).collect()
    }

    /// The tool centre point in the world at `q`.
    pub fn pose<const N: usize>(
        &self,
        scene: &PlanningScene<N>,
        q: &[f64; N],
    ) -> Result<na::Isometry3<f64>, SceneError> {
        Ok(scene.link_pose(&self.link, q)? * self.tcp)
    }

    /// The pose [`link`](Self::link) must have in the world for the tool
    /// centre point to be at `tcp_in_world`: the target an inverse
    /// kinematics solver for the mounting link is given.
    pub fn link_target(&self, tcp_in_world: &na::Isometry3<f64>) -> na::Isometry3<f64> {
        tcp_in_world * self.tcp.inverse()
    }
}

fn max_difference<const N: usize>(a: &[f64; N], b: &[f64; N]) -> f64 {
    (0..N).map(|i| (a[i] - b[i]).abs()).fold(0.0, f64::max)
}
