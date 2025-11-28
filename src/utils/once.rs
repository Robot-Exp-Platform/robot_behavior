use crate::Coord;

#[derive(Default)]
pub struct OverrideOnce<T> {
    once: Option<T>,
    default: T,
}

impl<T: Clone> OverrideOnce<T> {
    pub fn new(default: T) -> Self {
        Self { once: None, default }
    }

    pub fn set(&mut self, default: T) {
        self.default = default;
    }

    pub fn once(&mut self, once: T) {
        self.once = Some(once);
    }

    pub fn once_with(&mut self, f: impl FnOnce() -> T) {
        self.once = Some(f());
    }

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
pub struct RobotBound {
    pub coord: OverrideOnce<Coord>,
    pub joint_max_vel: OverrideOnce<Vec<f64>>,
    pub joint_max_acc: OverrideOnce<Vec<f64>>,
    pub joint_max_jerk: OverrideOnce<Vec<f64>>,
}

#[derive(Default)]
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
