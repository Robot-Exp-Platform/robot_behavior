//! Collision-free joint-space paths.
//!
//! [`plan_joint_path`] runs RRT-Connect between two configurations inside
//! joint bounds, then shortcuts the result. Collision checking is the
//! caller's: `is_valid` answers for one configuration (a simulator's
//! distance query, primitive geometry, a workspace limit), and edges are
//! checked by sampling it every `resolution` radians. The sampler is a seeded
//! xorshift, so a seed always gives the same path. The waypoints can be timed
//! with [`plan_waypoints_traj_via_copp`](crate::plan_waypoints_traj_via_copp).

/// Search settings for [`plan_joint_path`].
#[derive(Clone, Copy, Debug)]
pub struct PlannerOptions {
    /// Longest tree edge, as the largest per-joint step (rad).
    pub max_step: f64,
    /// Largest per-joint gap between configurations checked along an edge.
    pub resolution: f64,
    /// Tree extensions before giving up.
    pub max_iterations: usize,
    /// Random shortcut attempts on the found path.
    pub shortcut_attempts: usize,
    pub seed: u64,
}

impl Default for PlannerOptions {
    fn default() -> Self {
        Self {
            max_step: 0.2,
            resolution: 0.02,
            max_iterations: 5000,
            shortcut_attempts: 200,
            seed: 1,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PlanError {
    /// The start is invalid or outside the bounds.
    InvalidStart,
    /// The goal is invalid or outside the bounds.
    InvalidGoal,
    /// No path within the iteration budget.
    NotFound { iterations: usize },
}

impl std::fmt::Display for PlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidStart => write!(f, "the start configuration is invalid"),
            Self::InvalidGoal => write!(f, "the goal configuration is invalid"),
            Self::NotFound { iterations } => write!(f, "no path found in {iterations} iterations"),
        }
    }
}

impl std::error::Error for PlanError {}

/// Waypoints from `start` to `goal`, both included, with every straight
/// segment between consecutive waypoints valid at `options.resolution`.
pub fn plan_joint_path<const N: usize>(
    start: [f64; N],
    goal: [f64; N],
    lower: [f64; N],
    upper: [f64; N],
    mut is_valid: impl FnMut(&[f64; N]) -> bool,
    options: PlannerOptions,
) -> Result<Vec<[f64; N]>, PlanError> {
    let inside = |q: &[f64; N]| (0..N).all(|i| (lower[i]..=upper[i]).contains(&q[i]));
    if !inside(&start) || !is_valid(&start) {
        return Err(PlanError::InvalidStart);
    }
    if !inside(&goal) || !is_valid(&goal) {
        return Err(PlanError::InvalidGoal);
    }
    if segment_is_valid(&start, &goal, options.resolution, &mut is_valid) {
        return Ok(vec![start, goal]);
    }

    let mut rng = XorShift(options.seed.max(1));
    // Each tree is a list of (configuration, parent index).
    let mut trees = [vec![(start, usize::MAX)], vec![(goal, usize::MAX)]];
    for iteration in 0..options.max_iterations {
        let (a, b) = if iteration % 2 == 0 { (0, 1) } else { (1, 0) };
        let sample: [f64; N] =
            std::array::from_fn(|i| lower[i] + rng.unit() * (upper[i] - lower[i]));
        let Some(new) = extend(&mut trees[a], &sample, &options, &mut is_valid) else {
            continue;
        };
        // Greedily grow the other tree towards the new node.
        let target = trees[a][new].0;
        let mut reached = None;
        while let Some(node) = extend(&mut trees[b], &target, &options, &mut is_valid) {
            if distance(&trees[b][node].0, &target) == 0.0 {
                reached = Some(node);
                break;
            }
        }
        if let Some(meet) = reached {
            // Root of `a` to the new node, then on from the meeting node
            // (the same configuration) to the root of `b`.
            let mut path = branch(&trees[a], new);
            path.reverse();
            path.extend(branch(&trees[b], meet).into_iter().skip(1));
            if a == 1 {
                path.reverse();
            }
            shortcut(&mut path, &options, &mut rng, &mut is_valid);
            return Ok(path);
        }
    }
    Err(PlanError::NotFound { iterations: options.max_iterations })
}

/// Whether the straight segment from `a` to `b` stays valid, checked at
/// most `resolution` apart per joint.
pub fn segment_is_valid<const N: usize>(
    a: &[f64; N],
    b: &[f64; N],
    resolution: f64,
    mut is_valid: impl FnMut(&[f64; N]) -> bool,
) -> bool {
    let steps = (distance(a, b) / resolution).ceil().max(1.0) as usize;
    (1..=steps).all(|k| {
        let t = k as f64 / steps as f64;
        is_valid(&std::array::from_fn(|i| a[i] + t * (b[i] - a[i])))
    })
}

/// Largest per-joint difference.
fn distance<const N: usize>(a: &[f64; N], b: &[f64; N]) -> f64 {
    (0..N).map(|i| (a[i] - b[i]).abs()).fold(0.0, f64::max)
}

/// Adds a node one step from the nearest node towards `target`, if that
/// edge is valid; returns its index.
fn extend<const N: usize>(
    tree: &mut Vec<([f64; N], usize)>,
    target: &[f64; N],
    options: &PlannerOptions,
    is_valid: &mut impl FnMut(&[f64; N]) -> bool,
) -> Option<usize> {
    let (nearest, gap) = tree
        .iter()
        .enumerate()
        .map(|(i, (q, _))| (i, distance(q, target)))
        .min_by(|x, y| x.1.total_cmp(&y.1))?;
    if gap == 0.0 {
        return None;
    }
    let from = tree[nearest].0;
    let t = (options.max_step / gap).min(1.0);
    let to: [f64; N] = std::array::from_fn(|i| from[i] + t * (target[i] - from[i]));
    if !segment_is_valid(&from, &to, options.resolution, &mut *is_valid) {
        return None;
    }
    tree.push((to, nearest));
    Some(tree.len() - 1)
}

/// Configurations from `node` back to the tree's root.
fn branch<const N: usize>(tree: &[([f64; N], usize)], mut node: usize) -> Vec<[f64; N]> {
    let mut path = Vec::new();
    while node != usize::MAX {
        path.push(tree[node].0);
        node = tree[node].1;
    }
    path
}

fn shortcut<const N: usize>(
    path: &mut Vec<[f64; N]>,
    options: &PlannerOptions,
    rng: &mut XorShift,
    is_valid: &mut impl FnMut(&[f64; N]) -> bool,
) {
    for _ in 0..options.shortcut_attempts {
        if path.len() < 3 {
            return;
        }
        let i = (rng.unit() * (path.len() - 2) as f64) as usize;
        let j = i + 2 + (rng.unit() * (path.len() - i - 2) as f64) as usize;
        let j = j.min(path.len() - 1);
        if segment_is_valid(&path[i], &path[j], options.resolution, &mut *is_valid) {
            path.drain(i + 1..j);
        }
    }
}

/// xorshift64*: small, seedable, the same everywhere.
struct XorShift(u64);

impl XorShift {
    /// Uniform in `[0, 1)`.
    fn unit(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1u64 << 53) as f64
    }
}
