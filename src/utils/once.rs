use crate::Coord;

/// A value with a stable `default` plus an optional one-shot `override`.
///
/// Calling [`once`](OverrideOnce::once) stages a value that is consumed by the
/// **next** [`get`](OverrideOnce::get); afterwards `get` falls back to the
/// `default` again. This is how the [`Arm`](crate::Arm) `with_*` builders apply
/// a tightened limit to a single motion without permanently changing the
/// configured bound.
///
/// # Example
/// ```
/// use robot_behavior::OverrideOnce;
///
/// let mut v = OverrideOnce::new(10);
/// v.once(3);
/// assert_eq!(v.get(), 3);  // one-shot override used
/// assert_eq!(v.get(), 10); // back to default
/// ```
#[derive(Default)]
pub struct OverrideOnce<T> {
    once: Option<T>,
    default: T,
}

impl<T: Clone> OverrideOnce<T> {
    /// Create an `OverrideOnce` with the given persistent default.
    pub fn new(default: T) -> Self {
        Self { once: None, default }
    }

    /// Replace the persistent default value.
    pub fn set(&mut self, default: T) {
        self.default = default;
    }

    /// Stage a one-shot value to be returned by the next [`get`](Self::get).
    pub fn once(&mut self, once: T) {
        self.once = Some(once);
    }

    /// Stage a one-shot value computed lazily by `f`.
    pub fn once_with(&mut self, f: impl FnOnce() -> T) {
        self.once = Some(f());
    }

    /// Return the staged one-shot value if present (consuming it), otherwise a
    /// clone of the default.
    pub fn get(&mut self) -> T {
        self.once.take().unwrap_or(self.default.clone())
    }
}

impl<T> From<T> for OverrideOnce<T> {
    fn from(default: T) -> Self {
        Self { once: None, default }
    }
}

#[derive(Default)]
/// Per-motion override bundle for a `Vec`-sized (dynamic-DoF) robot: a frame
/// [`Coord`] plus joint velocity / acceleration / jerk limits, each an
/// [`OverrideOnce`] so callers can tweak one motion at a time.
pub struct RobotBound {
    pub coord: OverrideOnce<Coord>,
    pub joint_max_vel: OverrideOnce<Vec<f64>>,
    pub joint_max_acc: OverrideOnce<Vec<f64>>,
    pub joint_max_jerk: OverrideOnce<Vec<f64>>,
}

#[derive(Default)]
/// Per-motion override bundle for an `N`-DoF arm: frame [`Coord`], joint
/// limits (`[f64; N]`) and Cartesian/rotational limits, each an
/// [`OverrideOnce`]. The array-typed counterpart of [`RobotBound`].
pub struct ArmBound<const N: usize>
where
    [f64; N]: Default,
{
    pub coord: OverrideOnce<Coord>,
    pub max_vel: OverrideOnce<[f64; N]>,
    pub max_acc: OverrideOnce<[f64; N]>,
    pub max_jerk: OverrideOnce<[f64; N]>,
    pub max_cartesian_vel: OverrideOnce<f64>,
    pub max_cartesian_acc: OverrideOnce<f64>,
    pub max_cartesian_jerk: OverrideOnce<f64>,
    pub max_rotation_vel: OverrideOnce<f64>,
    pub max_rotation_acc: OverrideOnce<f64>,
    pub max_rotation_jerk: OverrideOnce<f64>,
}
