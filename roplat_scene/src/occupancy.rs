//! An occupancy map that sensed scans keep current: the role of the OctoMap
//! in MoveIt's planning scene.
//!
//! [`PlanningScene::add_points`] adds what a depth reading hits and nothing
//! else, so an object that has since moved stays in the scene until the
//! reading is replaced. An [`OccupancyMap`] also learns from the free space
//! each ray passes through: every voxel a ray crosses before its return is
//! seen empty, the voxel of the return is seen occupied, and each voxel keeps
//! the log-odds of being occupied, clamped so it can change its mind (the
//! update OctoMap uses, with its default probabilities). Over a few scans
//! a removed object's voxels clear and a new one's fill, while voxels never
//! seen stay unknown. [`PlanningScene::set_occupancy`] puts the occupied
//! voxels into the scene as one object.
//!
//! ```
//! use roplat_scene::occupancy::OccupancyMap;
//!
//! let mut map = OccupancyMap::new(0.05);
//! // A sensor at the origin sees a point 1 m ahead.
//! map.insert_scan([0.0, 0.0, 0.0], &[[1.0, 0.0, 0.0]], 5.0);
//! assert_eq!(map.is_occupied([1.0, 0.0, 0.0]), Some(true));
//! assert_eq!(map.is_occupied([0.5, 0.0, 0.0]), Some(false));
//! assert_eq!(map.is_occupied([0.0, 1.0, 0.0]), None);
//! ```

use std::collections::HashMap;

use nalgebra as na;
use parry3d_f64::shape::SharedShape;

use crate::{Mount, Object, Part, PlanningScene, SceneError};

/// How a voxel's belief moves with each observation, as log-odds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OccupancyModel {
    /// Added for a return in the voxel (OctoMap: probability 0.7).
    pub hit: f32,
    /// Added for a ray passing through it (OctoMap: probability 0.4).
    pub miss: f32,
    /// Bounds of the belief, so it can change its mind (OctoMap: 0.12 and
    /// 0.97).
    pub min: f32,
    pub max: f32,
}

impl Default for OccupancyModel {
    fn default() -> Self {
        let logit = |p: f32| (p / (1.0 - p)).ln();
        Self {
            hit: logit(0.7),
            miss: logit(0.4),
            min: logit(0.12),
            max: logit(0.97),
        }
    }
}

/// Voxels of a fixed edge with a belief each: occupied above even odds,
/// free below, unknown where no ray has been.
#[derive(Clone, Debug)]
pub struct OccupancyMap {
    voxel: f64,
    model: OccupancyModel,
    cells: HashMap<[i64; 3], f32>,
}

impl OccupancyMap {
    /// An empty map of voxels of edge `voxel` (m).
    ///
    /// # Panics
    ///
    /// If `voxel` is not positive.
    pub fn new(voxel: f64) -> Self {
        assert!(
            voxel > 0.0 && voxel.is_finite(),
            "voxel edge {voxel} must be positive"
        );
        Self {
            voxel,
            model: OccupancyModel::default(),
            cells: HashMap::new(),
        }
    }

    pub fn with_model(mut self, model: OccupancyModel) -> Self {
        self.model = model;
        self
    }

    pub fn voxel(&self) -> f64 {
        self.voxel
    }

    /// Adds one scan taken from `origin`: each of `points` is a return, so
    /// the voxels its ray crosses are seen free and its own voxel occupied.
    /// A return farther than `max_range` (m) only clears the ray up to that
    /// range, as a sensor's far returns are too noisy to mark. All in one
    /// frame, usually the world. Non-finite points are skipped.
    pub fn insert_scan(&mut self, origin: [f64; 3], points: &[[f64; 3]], max_range: f64) {
        let origin = na::Point3::from(origin);
        // A voxel is updated once per scan: occupied wins over free.
        let mut hits: HashMap<[i64; 3], ()> = HashMap::new();
        let mut misses: HashMap<[i64; 3], ()> = HashMap::new();
        for p in points.iter().filter(|p| p.iter().all(|c| c.is_finite())) {
            let point = na::Point3::from(*p);
            let along = point - origin;
            let length = along.norm();
            if length == 0.0 {
                continue;
            }
            let (end, hit) = if length > max_range {
                (origin + along * (max_range / length), false)
            } else {
                (point, true)
            };
            let end_key = self.key(&end);
            for key in self.traverse(&origin, &end) {
                if key != end_key {
                    misses.insert(key, ());
                }
            }
            if hit {
                hits.insert(end_key, ());
            } else {
                misses.insert(end_key, ());
            }
        }
        let model = self.model;
        for key in misses.keys().filter(|key| !hits.contains_key(*key)) {
            let belief = self.cells.entry(*key).or_insert(0.0);
            *belief = (*belief + model.miss).clamp(model.min, model.max);
        }
        for key in hits.keys() {
            let belief = self.cells.entry(*key).or_insert(0.0);
            *belief = (*belief + model.hit).clamp(model.min, model.max);
        }
    }

    /// Whether the voxel holding `p` is occupied; `None` where no ray has
    /// reached.
    pub fn is_occupied(&self, p: [f64; 3]) -> Option<bool> {
        self.cells
            .get(&self.key(&na::Point3::from(p)))
            .map(|belief| *belief > 0.0)
    }

    /// Centres of the occupied voxels.
    pub fn occupied(&self) -> impl Iterator<Item = [f64; 3]> + '_ {
        self.cells
            .iter()
            .filter(|(_, belief)| **belief > 0.0)
            .map(|(key, _)| key.map(|k| (k as f64 + 0.5) * self.voxel))
    }

    /// Voxels seen at least once, occupied or free.
    pub fn known(&self) -> usize {
        self.cells.len()
    }

    fn key(&self, p: &na::Point3<f64>) -> [i64; 3] {
        [p.x, p.y, p.z].map(|c| (c / self.voxel).floor() as i64)
    }

    /// The voxels the segment from `from` to `to` passes through, in order
    /// (Amanatides and Woo's traversal).
    fn traverse(&self, from: &na::Point3<f64>, to: &na::Point3<f64>) -> Vec<[i64; 3]> {
        let mut key = self.key(from);
        let end = self.key(to);
        let direction = to - from;
        let mut step = [0i64; 3];
        let mut t_max = [f64::INFINITY; 3];
        let mut t_delta = [f64::INFINITY; 3];
        for k in 0..3 {
            if direction[k] > 0.0 {
                step[k] = 1;
                let boundary = (key[k] + 1) as f64 * self.voxel;
                t_max[k] = (boundary - from[k]) / direction[k];
                t_delta[k] = self.voxel / direction[k];
            } else if direction[k] < 0.0 {
                step[k] = -1;
                let boundary = key[k] as f64 * self.voxel;
                t_max[k] = (boundary - from[k]) / direction[k];
                t_delta[k] = -self.voxel / direction[k];
            }
        }
        let mut keys = vec![key];
        // Each step crosses one face; the segment crosses at most this many.
        let limit = (0..3)
            .map(|k| (end[k] - key[k]).unsigned_abs())
            .sum::<u64>()
            + 1;
        for _ in 0..limit {
            if key == end {
                break;
            }
            let axis = (0..3)
                .min_by(|&a, &b| t_max[a].total_cmp(&t_max[b]))
                .expect("three axes");
            if t_max[axis] > 1.0 {
                break;
            }
            key[axis] += step[axis];
            t_max[axis] += t_delta[axis];
            keys.push(key);
        }
        keys
    }
}

impl<const N: usize> PlanningScene<N> {
    /// Puts the occupied voxels of `map` into the scene as the world object
    /// `name`, each a cube; replaces an object of that name placed this way
    /// or by [`add_points`](Self::add_points). Returns the number of
    /// voxels. Remove the robot's own returns before inserting scans
    /// ([`without_robot_points`](Self::without_robot_points)), or the arm
    /// becomes an obstacle to itself.
    pub fn set_occupancy(&mut self, name: &str, map: &OccupancyMap) -> Result<usize, SceneError> {
        if self.model.link(name).is_some() {
            return Err(SceneError::NameTaken(name.into()));
        }
        if let Ok(k) = self.object(name) {
            if !matches!(self.objects[k].mount, Mount::World(_)) {
                return Err(SceneError::Attached(name.into()));
            }
            self.objects.remove(k);
        }
        let half = map.voxel() / 2.0;
        let cubes: Vec<_> = map
            .occupied()
            .map(|c| {
                (
                    na::Isometry3::translation(c[0], c[1], c[2]),
                    SharedShape::cuboid(half, half, half),
                )
            })
            .collect();
        let count = cubes.len();
        if count > 0 {
            let part = Part {
                at: na::Isometry3::identity(),
                shape: SharedShape::compound(cubes),
            };
            self.objects.push(Object {
                name: name.into(),
                part,
                mount: Mount::World(na::Isometry3::identity()),
            });
        }
        Ok(count)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_ray_crosses_every_voxel_on_its_way() {
        let map = OccupancyMap::new(0.1);
        let keys = map.traverse(
            &na::Point3::new(0.05, 0.05, 0.05),
            &na::Point3::new(0.95, 0.35, 0.05),
        );
        assert_eq!(keys.first(), Some(&[0, 0, 0]));
        assert_eq!(keys.last(), Some(&[9, 3, 0]));
        // Neighbours share a face.
        for pair in keys.windows(2) {
            let d: i64 = (0..3).map(|k| (pair[1][k] - pair[0][k]).abs()).sum();
            assert_eq!(d, 1, "{pair:?}");
        }
        assert_eq!(keys.len(), 9 + 3 + 1);
    }

    #[test]
    fn beliefs_saturate_and_can_change() {
        let mut map = OccupancyMap::new(0.1);
        for _ in 0..20 {
            map.insert_scan([0.0; 3], &[[1.0, 0.05, 0.05]], 5.0);
        }
        assert_eq!(map.is_occupied([1.0, 0.05, 0.05]), Some(true));
        // The object is gone: rays now reach 2 m through its voxel. A
        // saturated belief clears within a handful of scans.
        let mut scans = 0;
        while map.is_occupied([1.0, 0.05, 0.05]) == Some(true) {
            map.insert_scan([0.0; 3], &[[2.0, 0.05, 0.05]], 5.0);
            scans += 1;
            assert!(scans < 10);
        }
        assert!(scans >= 2, "{scans}");
        // Beyond the maximum range a return only clears.
        map.insert_scan([0.0; 3], &[[9.0, 0.05, 0.05]], 3.0);
        assert_eq!(map.is_occupied([2.95, 0.05, 0.05]), Some(false));
        assert_eq!(map.is_occupied([3.15, 0.05, 0.05]), None);
        assert_eq!(map.is_occupied([9.0, 0.05, 0.05]), None);
    }
}
