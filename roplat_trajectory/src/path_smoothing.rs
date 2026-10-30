//! Shorter and smoother joint paths, after a planner.
//!
//! A sampling planner's path zigzags even after
//! [`plan_joint_path`](crate::plan_joint_path)'s random shortcuts, and its
//! corners are sharp, which time scaling turns into slow-downs. These are the
//! two steps of OMPL's path simplifier, which MoveIt runs after planning:
//!
//! - [`simplify_path`] keeps only the waypoints needed: from each kept
//!   waypoint it jumps to the farthest later one it can reach in a straight
//!   valid segment. Deterministic.
//! - [`smooth_path`] rounds the corners by Chaikin's corner cutting: each pass
//!   replaces every interior corner by the two points a quarter of the way
//!   along its segments, which converges to a quadratic B-spline. Where that
//!   cut would leave the valid region a smaller one is tried, an eighth and
//!   then a sixteenth of the way, and failing those the corner stays, so
//!   near an obstacle the path keeps hugging it.
//!
//! Both keep the endpoints, take collision checking from the caller like the
//! planner (`is_valid` for one configuration, segments sampled every
//! `resolution` per joint), and return a path whose new segments are valid.
//! [`simplify_motion`] and [`smooth_motion`] take a check of a whole straight
//! motion instead, as [`plan_joint_motion`](crate::plan_joint_motion) does.
//! Time the result with [`plan_waypoints_traj_via_copp`](crate::plan_waypoints_traj_via_copp).
//!
//! ```
//! use roplat_trajectory::{path_length, simplify_path, smooth_path};
//!
//! // A zigzag in free space straightens out; an L gets a rounded corner.
//! let zigzag = [[0.0, 0.0], [0.2, 0.1], [0.4, -0.1], [0.6, 0.1], [1.0, 0.0]];
//! assert_eq!(simplify_path(&zigzag, 0.01, |_| true), vec![[0.0, 0.0], [1.0, 0.0]]);
//! let corner = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]];
//! let round = smooth_path(&corner, 3, 0.01, |_| true);
//! assert_eq!((round[0], *round.last().unwrap()), ([0.0, 0.0], [1.0, 1.0]));
//! assert!(path_length(&round) < path_length(&corner));
//! ```

use crate::path_planning::segment_is_valid;

/// Waypoints of `path` that a straight valid segment cannot skip, with both
/// endpoints; see the [module](self) docs. A segment of `path` itself is
/// kept even if it is invalid.
pub fn simplify_path<const N: usize>(
    path: &[[f64; N]],
    resolution: f64,
    mut is_valid: impl FnMut(&[f64; N]) -> bool,
) -> Vec<[f64; N]> {
    simplify_motion(path, |a, b| {
        segment_is_valid(a, b, resolution, &mut is_valid)
    })
}

/// As [`simplify_path`], with each straight segment checked whole by
/// `motion_is_valid(from, to)`.
pub fn simplify_motion<const N: usize>(
    path: &[[f64; N]],
    mut motion_is_valid: impl FnMut(&[f64; N], &[f64; N]) -> bool,
) -> Vec<[f64; N]> {
    let Some(&first) = path.first() else {
        return Vec::new();
    };
    let mut kept = vec![first];
    let mut at = 0;
    while at + 1 < path.len() {
        // The farthest waypoint in sight; the next one at worst.
        let next = (at + 2..path.len())
            .rev()
            .find(|&j| motion_is_valid(&path[at], &path[j]))
            .unwrap_or(at + 1);
        kept.push(path[next]);
        at = next;
    }
    kept
}

/// `path` with its corners rounded by `passes` rounds of corner cutting;
/// see the [module](self) docs. Each pass nearly doubles the waypoints.
///
/// The two points that replace a corner lie on its two segments, so only
/// the chord between them is new; it is checked, and a smaller cut is tried
/// when the chord is invalid. The rest of each segment is part of the input
/// segment it lies on.
pub fn smooth_path<const N: usize>(
    path: &[[f64; N]],
    passes: usize,
    resolution: f64,
    mut is_valid: impl FnMut(&[f64; N]) -> bool,
) -> Vec<[f64; N]> {
    smooth_motion(path, passes, |a, b| {
        segment_is_valid(a, b, resolution, &mut is_valid)
    })
}

/// As [`smooth_path`], with each new chord checked whole by
/// `motion_is_valid(from, to)`.
pub fn smooth_motion<const N: usize>(
    path: &[[f64; N]],
    passes: usize,
    mut motion_is_valid: impl FnMut(&[f64; N], &[f64; N]) -> bool,
) -> Vec<[f64; N]> {
    let mut path = path.to_vec();
    for _ in 0..passes {
        if path.len() < 3 {
            break;
        }
        let mut cut = Vec::with_capacity(2 * path.len());
        cut.push(path[0]);
        for k in 1..path.len() - 1 {
            let (before, corner, after) = (&path[k - 1], &path[k], &path[k + 1]);
            let chord = [0.25, 0.125, 0.0625].into_iter().find_map(|f| {
                let a: [f64; N] = std::array::from_fn(|i| corner[i] + f * (before[i] - corner[i]));
                let b: [f64; N] = std::array::from_fn(|i| corner[i] + f * (after[i] - corner[i]));
                motion_is_valid(&a, &b).then_some([a, b])
            });
            match chord {
                Some(chord) => cut.extend(chord),
                None => cut.push(*corner),
            }
        }
        cut.push(path[path.len() - 1]);
        // A kept corner leaves its neighbours' cut points in line with it;
        // repeated points from degenerate corners are dropped.
        cut.dedup();
        path = cut;
    }
    path
}

/// Euclidean length of `path` in joint space.
pub fn path_length<const N: usize>(path: &[[f64; N]]) -> f64 {
    path.windows(2)
        .map(|w| {
            (0..N)
                .map(|i| (w[1][i] - w[0][i]).powi(2))
                .sum::<f64>()
                .sqrt()
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{PlannerOptions, plan_joint_path};

    const RESOLUTION: f64 = 0.005;

    /// Free outside a disk of radius 0.2 at (0.5, 0.5).
    fn outside_disk(q: &[f64; 2]) -> bool {
        (q[0] - 0.5).powi(2) + (q[1] - 0.5).powi(2) > 0.04
    }

    fn all_valid(path: &[[f64; 2]], is_valid: impl Fn(&[f64; 2]) -> bool) -> bool {
        path.windows(2)
            .all(|w| segment_is_valid(&w[0], &w[1], RESOLUTION, &is_valid))
    }

    /// Largest change of direction (rad) at any interior waypoint.
    fn sharpest_turn(path: &[[f64; 2]]) -> f64 {
        path.windows(3)
            .map(|w| {
                let (u, v) = (
                    [w[1][0] - w[0][0], w[1][1] - w[0][1]],
                    [w[2][0] - w[1][0], w[2][1] - w[1][1]],
                );
                let cos = (u[0] * v[0] + u[1] * v[1]) / (u[0].hypot(u[1]) * v[0].hypot(v[1]));
                cos.clamp(-1.0, 1.0).acos()
            })
            .fold(0.0, f64::max)
    }

    #[test]
    fn simplifying_keeps_only_the_waypoints_an_obstacle_needs() {
        // Small steps around the disk's lower right, then up.
        let mut around: Vec<[f64; 2]> = (0..=20).map(|k| [0.05 * k as f64, 0.1]).collect();
        around.extend((1..=16).map(|k| [1.0, 0.1 + 0.05 * k as f64]));
        let simple = simplify_path(&around, RESOLUTION, outside_disk);
        assert_eq!(
            (simple[0], *simple.last().unwrap()),
            (around[0], *around.last().unwrap())
        );
        assert!(simple.len() < 6, "{simple:?}");
        assert!(all_valid(&simple, outside_disk));
        assert!(path_length(&simple) < path_length(&around));
        // A path through the disk: no waypoint is skipped across it.
        let blocked = [[0.0, 0.5], [0.5, 0.25], [1.0, 0.5]];
        assert_eq!(
            simplify_path(&blocked, RESOLUTION, outside_disk),
            blocked.to_vec()
        );
        assert!(simplify_path::<2>(&[], RESOLUTION, outside_disk).is_empty());
        assert_eq!(
            simplify_path(&[[0.3, 0.0]], RESOLUTION, outside_disk),
            vec![[0.3, 0.0]]
        );
    }

    #[test]
    fn smoothing_rounds_corners_and_stays_clear() {
        let corner = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]];
        let round = smooth_path(&corner, 4, RESOLUTION, |_| true);
        assert_eq!(round.len(), 18);
        assert!(sharpest_turn(&round) < 0.3, "{}", sharpest_turn(&round));
        assert!(
            round
                .iter()
                .all(|q| q[0] >= 0.0 && q[0] <= 1.0 && q[1] >= 0.0 && q[1] <= 1.0)
        );

        // An obstacle inside the corner: a disk of radius 0.15 at (0.8, 0.2)
        // blocks the quarter cut, not the eighth, so the rounding is tighter.
        let near = |q: &[f64; 2]| (q[0] - 0.8).powi(2) + (q[1] - 0.2).powi(2) > 0.0225;
        let hugging = smooth_path(&corner, 4, RESOLUTION, near);
        assert!(all_valid(&hugging, near));
        assert!(sharpest_turn(&hugging) < 0.6, "{}", sharpest_turn(&hugging));
        // Closer to the corner than the free rounding comes.
        let from_corner = |path: &[[f64; 2]]| {
            path.iter()
                .map(|q| (q[0] - 1.0).hypot(q[1]))
                .fold(f64::INFINITY, f64::min)
        };
        assert!(from_corner(&hugging) < from_corner(&round));
        // Blocked even at a sixteenth (a box over the corner's inside): the
        // corner stays.
        let wedged = |q: &[f64; 2]| !(q[0] > 0.85 && q[1] > 0.0 && q[0] < 1.0 && q[1] < 0.15);
        assert_eq!(smooth_path(&corner, 2, RESOLUTION, wedged), corner.to_vec());
        // Nothing to round.
        assert_eq!(
            smooth_path(&[[0.0, 0.0], [1.0, 1.0]], 3, RESOLUTION, near),
            vec![[0.0, 0.0], [1.0, 1.0]]
        );
    }

    #[test]
    fn a_planned_path_comes_out_shorter_smoother_and_still_valid() {
        let options =
            PlannerOptions { resolution: RESOLUTION, seed: 7, ..PlannerOptions::default() };
        let planned = plan_joint_path(
            [0.1, 0.5],
            [0.9, 0.5],
            [0.0; 2],
            [1.0; 2],
            outside_disk,
            options,
        )
        .unwrap();
        let simple = simplify_path(&planned, RESOLUTION, outside_disk);
        let smooth = smooth_path(&simple, 4, RESOLUTION, outside_disk);
        assert!(all_valid(&simple, outside_disk) && all_valid(&smooth, outside_disk));
        assert!(path_length(&simple) <= path_length(&planned) + 1e-12);
        assert!(path_length(&smooth) <= path_length(&simple) + 1e-12);
        assert!(sharpest_turn(&smooth) < sharpest_turn(&simple) || simple.len() < 3);
        assert_eq!(
            (smooth[0], *smooth.last().unwrap()),
            ([0.1, 0.5], [0.9, 0.5])
        );
    }
}
