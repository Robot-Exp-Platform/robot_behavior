use nalgebra as na;
use std::{sync::Arc, time::Duration};

type FN<T> = Arc<dyn Fn(Duration) -> T + Send + Sync>;

/// Generate a linear path for joint space
pub fn joint_linear<const N: usize>(
    start: &[f64; N],
    end: &[f64; N],
    v_max: &[f64; N],
) -> (FN<[f64; N]>, Duration) {
    let start = na::SVector::<f64, N>::from_row_slice(start);
    let end = na::SVector::<f64, N>::from_row_slice(end);
    let v_max = na::SVector::<f64, N>::from_row_slice(v_max);

    let delta = end - start;
    let t_max = delta
        .iter()
        .zip(v_max.iter())
        .map(|(d, v)| d.abs() / v)
        .fold(0.0, f64::max);

    let f = move |t: Duration| {
        let t = t.as_secs_f64().min(t_max);
        let t = t / t_max;
        if t >= 1. { end } else { start + delta * t }.into()
    };

    (Arc::new(f), Duration::from_secs_f64(t_max))
}

/// Generate a linear path for cartesian space
pub fn cartesian_quat_linear(
    start: na::Isometry3<f64>,
    end: na::Isometry3<f64>,
    v_max: f64,
) -> (FN<na::Isometry3<f64>>, Duration) {
    let start_trans = start.translation.vector;
    let end_trans = end.translation.vector;
    let delta = end_trans - start_trans;
    let t_max = delta.norm() / v_max;

    let f = move |t: Duration| {
        let t = t.as_secs_f64().min(t_max);
        let t = t / t_max;
        if t >= 1. {
            end
        } else {
            start.lerp_slerp(&end, t)
        }
    };

    (Arc::new(f), Duration::from_secs_f64(t_max))
}

pub fn joint_trapezoid<const N: usize>(
    start: &[f64; N],
    end: &[f64; N],
    v_max: &[f64; N],
    a_max: &[f64; N],
) -> (FN<[f64; N]>, Duration) {
    let start = na::SVector::<f64, N>::from_column_slice(start);
    let end = na::SVector::<f64, N>::from_column_slice(end);
    let delta = end - start;

    let k1 = delta
        .iter()
        .zip(a_max)
        .map(|(d, a)| d / a)
        .fold(0.0, f64::max);
    let k2 = delta
        .iter()
        .zip(v_max)
        .map(|(d, v)| d / v)
        .fold(0.0, f64::max);
    let mut t_max = delta
        .iter()
        .zip(v_max)
        .zip(a_max)
        .map(trapezoid_min_time)
        .fold(0.0, f64::max);

    let t1_max = 1. - (1. - 4. * k1 / t_max.powi(2)).sqrt();
    let t2_min = 2. - 2. * k2 / t_max;

    let (t_max, t2) = if t1_max <= t2_min {
        (t_max, t2_min / 2.)
    } else {
        t_max = k2 + k1 / k2;
        (t_max, 1. - 1. * k2 / t_max)
    };

    let f = move |t: Duration| {
        let t = t.as_secs_f64().min(t_max);
        let t = t / t_max;
        let b = 1. / 2. / t2 / (1. - t2);
        if t < 1. - t2 {
            start + delta * b * if t < t2 { t * t } else { 2. * t - t2.powi(2) }
        } else {
            end - delta * b * if t < 1. { (1. - t).powi(2) } else { 0. }
        }
        .into()
    };

    (Arc::new(f), Duration::from_secs_f64(t_max))
}

fn trapezoid_min_time(para: ((&f64, &f64), &f64)) -> f64 {
    let ((d, v_max), a_max) = para;
    if *d > v_max.powi(2) / *a_max {
        d / v_max + v_max / a_max
    } else {
        2. * (d / a_max).sqrt()
    }
}

pub fn t_curve(
    delta: f64,
    v_max: f64,
    a_max: f64,
) -> (f64, Arc<dyn Fn(Duration) -> f64 + Send + Sync>) {
    if delta.abs() < 1e-6 {
        return (0., Arc::new(|_| 0.));
    }
    let delta = delta.abs();
    if !a_max.is_finite() {
        // Unbounded acceleration: constant speed, or no time at all when the
        // speed is unbounded too.
        let t_total = if v_max.is_finite() { delta / v_max } else { 0. };
        let f = move |t: Duration| {
            let t = t.as_secs_f64();
            if t < t_total { v_max * t } else { delta }
        };
        return (t_total, Arc::new(f));
    }
    // A short move never reaches v_max: accelerating to sqrt(delta * a_max)
    // and braking again covers exactly delta.
    let v_max = v_max.min((delta * a_max).sqrt());
    let t_acc = v_max / a_max;
    let s_acc = 0.5 * a_max * t_acc * t_acc;
    let s_cruise = delta - 2. * s_acc;
    let t_cruise = if s_cruise > 0. { s_cruise / v_max } else { 0. };
    let t_total = 2. * t_acc + t_cruise;
    let f = move |t: Duration| {
        let t = t.as_secs_f64().min(t_total);
        if t < t_acc {
            0.5 * a_max * t * t
        } else if t < t_acc + t_cruise {
            s_acc + v_max * (t - t_acc)
        } else {
            delta - 0.5 * a_max * (t_total - t).powi(2)
        }
    };
    (t_total, Arc::new(f))
}

#[derive(Clone, Copy, Debug)]
enum ScalarSCurveProfile {
    Hold,
    /// Velocity and acceleration unbounded: the move takes no time.
    Instant {
        delta: f64,
    },
    /// Acceleration and jerk unbounded: constant speed from start to end.
    Constant {
        delta: f64,
        v_max: f64,
        t_total: f64,
    },
    Trapezoid {
        delta: f64,
        v_max: f64,
        a_max: f64,
        t_acc: f64,
        s_acc: f64,
        t_cruise: f64,
        t_total: f64,
    },
    Short {
        delta: f64,
        j_max: f64,
        t_min: f64,
        t1: f64,
    },
    Cruise {
        delta: f64,
        v_max: f64,
        a_max: f64,
        j_max: f64,
        t_min: f64,
        t1: f64,
        t2: f64,
        t34: f64,
    },
    NoCruise {
        delta: f64,
        a_max: f64,
        j_max: f64,
        t_min: f64,
        t1: f64,
        t2: f64,
    },
}

impl ScalarSCurveProfile {
    fn plan(delta: f64, v_max: f64, a_max: f64, j_max: f64) -> Self {
        if delta.abs() < 1e-6 {
            return Self::Hold;
        }

        let delta = delta.abs();
        if !a_max.is_finite() && !j_max.is_finite() {
            return if v_max.is_finite() {
                Self::Constant { delta, v_max, t_total: delta / v_max }
            } else {
                Self::Instant { delta }
            };
        }
        // An unbounded jerk plans a trapezoid. Compare with infinity, not
        // f64::MAX: a huge jerk bound divided by a short distance overflows.
        if !j_max.is_finite() {
            // A short move peaks at sqrt(delta * a_max): half the distance
            // accelerating, half braking.
            let v_max = v_max.min((delta * a_max).sqrt());
            let t_acc = v_max / a_max;
            let s_acc = 0.5 * a_max * t_acc * t_acc;
            let s_cruise = delta - 2. * s_acc;
            let t_cruise = if s_cruise > 0. { s_cruise / v_max } else { 0. };
            let t_total = 2. * t_acc + t_cruise;
            return Self::Trapezoid { delta, v_max, a_max, t_acc, s_acc, t_cruise, t_total };
        }

        let d2 = 2. * a_max.powi(3) / j_max.powi(2);
        let d1 = v_max * (a_max / j_max + v_max / a_max);

        if delta < d2 {
            let t_min = (32. * delta / j_max).powf(1. / 3.);
            Self::Short { delta, j_max, t_min, t1: t_min / 4. }
        } else if delta > d1 {
            Self::Cruise {
                delta,
                v_max,
                a_max,
                j_max,
                t_min: delta / v_max + v_max / a_max + a_max / j_max,
                t1: a_max / j_max,
                t2: v_max / a_max,
                t34: (delta - d1) / v_max,
            }
        } else {
            let t_min = (a_max.powi(2)
                + (a_max.powi(4) + 4. * a_max * delta * j_max.powi(2)).sqrt())
                / (a_max * j_max);
            Self::NoCruise {
                delta,
                a_max,
                j_max,
                t_min,
                t1: a_max / j_max,
                t2: t_min / 2. - a_max / j_max,
            }
        }
    }

    fn total_time(self) -> f64 {
        match self {
            Self::Hold | Self::Instant { .. } => 0.,
            Self::Constant { t_total, .. } => t_total,
            Self::Trapezoid { t_total, .. } => t_total,
            Self::Short { t_min, .. } => t_min,
            Self::Cruise { t_min, .. } => t_min,
            Self::NoCruise { t_min, .. } => t_min,
        }
    }

    fn sample(self, t: f64) -> f64 {
        match self {
            Self::Hold => 0.,
            Self::Instant { delta } => delta,
            Self::Constant { delta, v_max, t_total } => {
                if t < t_total {
                    v_max * t
                } else {
                    delta
                }
            }
            Self::Trapezoid { delta, v_max, a_max, t_acc, s_acc, t_cruise, t_total } => {
                let t = t.min(t_total);
                if t < t_acc {
                    0.5 * a_max * t * t
                } else if t < t_acc + t_cruise {
                    s_acc + v_max * (t - t_acc)
                } else {
                    delta - 0.5 * a_max * (t_total - t).powi(2)
                }
            }
            Self::Short { delta, j_max, t_min, t1 } => {
                let t = t.min(t_min);
                if t < t1 {
                    Self::path_1(j_max, t)
                } else if t < t1 * 3. {
                    Self::path_1(j_max, t1)
                        + Self::path_3(j_max, t - t1, j_max * t1.powi(2) / 2., j_max * t1)
                } else if t < t_min {
                    delta - Self::path_1(j_max, t_min - t)
                } else {
                    delta
                }
            }
            Self::Cruise { delta, v_max, a_max, j_max, t_min, t1, t2, t34 } => {
                let t = t.min(t_min);
                if t < t1 {
                    Self::path_1(j_max, t)
                } else if t < t2 {
                    Self::path_1(j_max, t1) + Self::path_2(t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t1 + t2 {
                    Self::path_1(j_max, t1)
                        + Self::path_2(t2 - t1, a_max * t1 / 2., j_max * t1)
                        + Self::path_3(j_max, t - t2, v_max - a_max * t1 / 2., a_max)
                } else if t < t1 + t2 + t34 {
                    delta / 2. + Self::path_4(t - t_min / 2., v_max)
                } else if t < t_min - t2 {
                    delta / 2.
                        + Self::path_4(t34 / 2., v_max)
                        + Self::path_5(j_max, t - (t1 + t2 + t34), v_max)
                } else if t < t_min - t1 {
                    delta
                        - Self::path_1(j_max, t1)
                        - Self::path_2(t_min - t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t_min {
                    delta - Self::path_1(j_max, t_min - t)
                } else {
                    delta
                }
            }
            Self::NoCruise { delta, a_max, j_max, t_min, t1, t2 } => {
                let t = t.min(t_min);
                if t < t1 {
                    Self::path_1(j_max, t)
                } else if t < t2 {
                    Self::path_1(j_max, t1) + Self::path_2(t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t_min - t2 {
                    Self::path_1(j_max, t1)
                        + Self::path_2(t2 - t1, a_max * t1 / 2., j_max * t1)
                        + Self::path_3(j_max, t - t2, a_max * (t2 - t1 / 2.), a_max)
                } else if t < t_min - t1 {
                    delta
                        - Self::path_1(j_max, t1)
                        - Self::path_2(t_min - t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t_min {
                    delta - Self::path_1(j_max, t_min - t)
                } else {
                    delta
                }
            }
        }
    }

    fn path_1(j_max: f64, t: f64) -> f64 {
        j_max * t.powi(3) / 6.
    }

    fn path_2(t: f64, v_s: f64, a: f64) -> f64 {
        a * t.powi(2) / 2. + v_s * t
    }

    fn path_3(j_max: f64, t: f64, v_s: f64, a_s: f64) -> f64 {
        -j_max * t.powi(3) / 6. + a_s * t.powi(2) / 2. + v_s * t
    }

    fn path_4(t: f64, v_s: f64) -> f64 {
        v_s * t
    }

    fn path_5(j_max: f64, t: f64, v_s: f64) -> f64 {
        v_s * t - j_max * t.powi(3) / 6.
    }
}

/// A synchronized jerk-limited move of `N` joints from a start to a target.
///
/// A bound may be `f64::INFINITY`. An unbounded jerk gives a trapezoid, an
/// unbounded acceleration and jerk a constant speed, and an unbounded
/// velocity and acceleration a move that takes no time. A caller that drives
/// hardware passes finite velocity and acceleration bounds.
#[derive(Clone, Debug)]
pub struct JointSCurve<const N: usize> {
    v_max: [f64; N],
    a_max: [f64; N],
    j_max: [f64; N],
    start: [f64; N],
    target: [f64; N],
    delta: [f64; N],
    profile: ScalarSCurveProfile,
    total: Duration,
    planned: bool,
}

impl<const N: usize> JointSCurve<N> {
    pub fn new(v_max: [f64; N], a_max: [f64; N], j_max: [f64; N]) -> Self {
        Self {
            v_max,
            a_max,
            j_max,
            start: [0.; N],
            target: [0.; N],
            delta: [0.; N],
            profile: ScalarSCurveProfile::Hold,
            total: Duration::ZERO,
            planned: false,
        }
    }

    pub fn plan(&mut self, start: [f64; N], target: [f64; N]) -> Duration {
        self.start = start;
        self.target = target;
        let mut v_path = f64::INFINITY;
        let mut a_path = f64::INFINITY;
        let mut j_path = f64::INFINITY;
        let mut has_motion = false;

        for i in 0..N {
            self.delta[i] = target[i] - start[i];
            let delta = self.delta[i].abs();
            if delta > 1e-6 {
                has_motion = true;
                v_path = v_path.min(self.v_max[i] / delta);
                a_path = a_path.min(self.a_max[i] / delta);
                j_path = j_path.min(self.j_max[i] / delta);
            }
        }

        if has_motion {
            self.profile = ScalarSCurveProfile::plan(1., v_path, a_path, j_path);
            self.total = Duration::from_secs_f64(self.profile.total_time());
        } else {
            self.profile = ScalarSCurveProfile::Hold;
            self.total = Duration::ZERO;
        }
        self.planned = true;
        self.total
    }

    pub fn apply(&self, time: Duration) -> ([f64; N], bool) {
        if !self.planned || self.total.is_zero() {
            return (self.target, true);
        }

        let progress = self.profile.sample(time.as_secs_f64()).min(1.);
        let mut result = self.start;
        for (value, delta) in result.iter_mut().zip(&self.delta) {
            *value += delta * progress;
        }

        if time >= self.total {
            (self.target, true)
        } else {
            (result, false)
        }
    }
}

pub fn joint_s_curve<const N: usize>(
    start: &[f64; N],
    end: &[f64; N],
    v_max: &[f64; N],
    a_max: &[f64; N],
    j_max: &[f64; N],
) -> (Arc<dyn Fn(Duration) -> [f64; N] + Send + Sync>, Duration) {
    let mut curve = JointSCurve::new(*v_max, *a_max, *j_max);
    let total = curve.plan(*start, *end);
    let f = move |t: Duration| curve.apply(t).0;

    (Arc::new(f), total)
}

#[cfg(all(test, any(target_os = "linux", target_os = "windows")))]
fn s_curve(
    delta: f64,
    v_max: f64,
    a_max: f64,
    j_max: f64,
) -> (f64, Arc<dyn Fn(Duration) -> f64 + Send + Sync>) {
    if !j_max.is_finite() {
        return t_curve(delta, v_max, a_max);
    }
    let d2 = 2. * a_max.powi(3) / j_max.powi(2);
    let d1 = v_max * (a_max / j_max + v_max / a_max);

    let path_1 = move |t: f64| j_max * t.powi(3) / 6.;
    let path_2 = move |t: f64, v_s: f64, a: f64| a * t.powi(2) / 2. + v_s * t;
    let path_3 =
        move |t: f64, v_s: f64, a_s: f64| -j_max * t.powi(3) / 6. + a_s * t.powi(2) / 2. + v_s * t;
    let path_4 = move |t: f64, v_s: f64| v_s * t;
    let path_5 = move |t: f64, v_s: f64| v_s * t - j_max * t.powi(3) / 6.;

    if delta < d2 {
        let t_min = (32. * delta / j_max).powf(1. / 3.);
        let t1 = t_min / 4.;
        (
            t_min,
            Arc::new(move |t: Duration| {
                let t = t.as_secs_f64();
                if t < t1 {
                    path_1(t)
                } else if t < t1 * 3. {
                    path_1(t1) + path_3(t - t1, j_max * t1.powi(2) / 2., j_max * t1)
                } else if t < t_min {
                    delta - path_1(t_min - t)
                } else {
                    delta
                }
            }),
        )
    } else if delta > d1 {
        let t_min = delta / v_max + v_max / a_max + a_max / j_max;
        let t1 = a_max / j_max;
        let t2 = v_max / a_max;
        let t34 = (delta - d1) / v_max;
        (
            t_min,
            Arc::new(move |t: Duration| {
                let t = t.as_secs_f64();
                if t < t1 {
                    path_1(t)
                } else if t < t2 {
                    path_1(t1) + path_2(t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t1 + t2 {
                    path_1(t1)
                        + path_2(t2 - t1, a_max * t1 / 2., j_max * t1)
                        + path_3(t - t2, v_max - a_max * t1 / 2., a_max)
                } else if t < t1 + t2 + t34 {
                    delta / 2. + path_4(t - t_min / 2., v_max)
                } else if t < t_min - t2 {
                    delta / 2. + path_4(t34 / 2., v_max) + path_5(t - (t1 + t2 + t34), v_max)
                } else if t < t_min - t1 {
                    delta - path_1(t1) - path_2(t_min - t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t_min {
                    delta - path_1(t_min - t)
                } else {
                    delta
                }
            }),
        )
    } else {
        let t_min = (a_max.powi(2) + (a_max.powi(4) + 4. * a_max * delta * j_max.powi(2)).sqrt())
            / (a_max * j_max);
        let t1 = a_max / j_max;
        let t2 = t_min / 2. - a_max / j_max;
        (
            t_min,
            Arc::new(move |t: Duration| {
                let t = t.as_secs_f64();
                if t < t1 {
                    path_1(t)
                } else if t < t2 {
                    path_1(t1) + path_2(t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t_min - t2 {
                    path_1(t1)
                        + path_2(t2 - t1, a_max * t1 / 2., j_max * t1)
                        + path_3(t - t2, a_max * (t2 - t1 / 2.), a_max)
                } else if t < t_min - t1 {
                    delta - path_1(t1) - path_2(t_min - t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t_min {
                    delta - path_1(t_min - t)
                } else {
                    delta
                }
            }),
        )
    }
}

pub fn joint_simple_4th_curve<const N: usize>(
    start: &[f64; N],
    end: &[f64; N],
    v_max: &[f64; N],
    a_max: &[f64; N],
) -> (FN<[f64; N]>, Duration) {
    let start = na::SVector::<f64, N>::from_column_slice(start);
    let end = na::SVector::<f64, N>::from_column_slice(end);
    let delta = end - start;

    let mut t_path = Vec::with_capacity(N);
    let mut f_path = Vec::with_capacity(N);
    for i in 0..N {
        let (t, f) = if v_max[i].is_finite() || a_max[i].is_finite() {
            simple_4th_curve(delta[i].abs(), v_max[i], a_max[i])
        } else {
            // Velocity and acceleration unbounded: this joint takes no time.
            let d = delta[i].abs();
            (
                0.,
                Arc::new(move |_: Duration| d) as Arc<dyn Fn(Duration) -> f64 + Send + Sync>,
            )
        };
        t_path.push(t);
        f_path.push(f);
    }

    let t_max = t_path.iter().cloned().fold(0.0, f64::max);

    let f = move |t: Duration| {
        let t = t.as_secs_f64();
        let t = if t_max > 0.0 { t / t_max } else { 0.0 };
        let mut result = start;
        for i in 0..N {
            result[i] += delta[i].signum() * f_path[i](Duration::from_secs_f64(t * t_path[i]));
        }
        result.into()
    };

    (Arc::new(f), Duration::from_secs_f64(t_max))
}

pub fn cartesian_quat_simple_4th_curve(
    start: na::Isometry3<f64>,
    end: na::Isometry3<f64>,
    v_max: f64,
    a_max: f64,
) -> (FN<na::Isometry3<f64>>, Duration) {
    let delta = (end.translation.vector - start.translation.vector).norm();
    let (t_min, f) = simple_4th_curve(1., v_max / delta, a_max / delta);
    let f = move |t: Duration| {
        let t = if t_min > 0.0 { f(t) } else { 0.0 };
        if t >= 1. {
            end
        } else {
            start.lerp_slerp(&end, t)
        }
    };
    (Arc::new(f), Duration::from_secs_f64(t_min))
}

fn simple_4th_curve(
    delta: f64,
    v_max: f64,
    a_max: f64,
) -> (f64, Arc<dyn Fn(Duration) -> f64 + Send + Sync>) {
    if delta < 1e-6 {
        return (0., Arc::new(|_| 0.));
    }
    let mut v_max = v_max;
    if delta < 1.5 * v_max.powi(2) / a_max {
        v_max = (2. / 3. * delta * a_max).sqrt();
    }

    let t1 = 1.5 * v_max / a_max;
    let t_min = t1 + delta / v_max;

    let f = move |t: Duration| {
        let t = t.as_secs_f64();
        if t < t1 {
            (t / t1).powi(3) * (t1 - 0.5 * t) * v_max
        } else if t < t_min - t1 {
            t1 * v_max / 2. + (t - t1) * v_max
        } else if t < t_min {
            delta - ((t_min - t) / t1).powi(3) * (t1 - 0.5 * (t_min - t)) * v_max
        } else {
            delta
        }
    };

    (t_min, Arc::new(f))
}

#[cfg(any(target_os = "linux", target_os = "windows"))]
#[cfg(test)]
mod test {
    use std::f64::consts::PI;

    use super::*;

    /// Review finding F2: with an unbounded jerk and a move shorter than 1 rad
    /// the curve used to step to the target.
    #[test]
    fn unbounded_jerk_moves_smoothly_over_a_short_distance() {
        for jerk in [f64::INFINITY, f64::MAX] {
            let mut curve = JointSCurve::new([2.0], [10.0], [jerk]);
            let total = curve.plan([0.0], [0.5]);
            assert!(total > Duration::ZERO);
            let mut previous = 0.0;
            for k in 0..=(total.as_micros() / 1000) as u64 {
                let (q, _) = curve.apply(Duration::from_millis(k));
                assert!(q[0].is_finite());
                assert!((q[0] - previous).abs() <= 2.0 * 0.001 + 1e-9, "t = {k} ms");
                previous = q[0];
            }
            assert_eq!(curve.apply(total).0, [0.5]);
        }
    }

    #[test]
    fn unbounded_acceleration_moves_at_constant_speed_or_at_once() {
        let mut curve = JointSCurve::new([2.0], [f64::INFINITY], [f64::INFINITY]);
        assert_eq!(curve.plan([0.0], [0.5]), Duration::from_millis(250));
        assert!((curve.apply(Duration::from_millis(100)).0[0] - 0.2).abs() < 1e-12);
        let mut instant = JointSCurve::new([f64::INFINITY], [f64::INFINITY], [f64::INFINITY]);
        assert_eq!(instant.plan([0.0], [0.5]), Duration::ZERO);
        assert_eq!(instant.apply(Duration::ZERO), ([0.5], true));
        let (total, f) = t_curve(0.5, 2.0, f64::INFINITY);
        assert_eq!((total, f(Duration::from_secs(1))), (0.25, 0.5));
    }

    /// A move too short to reach v_max peaks at sqrt(delta * a_max) and stays
    /// continuous where acceleration turns into braking.
    #[test]
    fn a_short_trapezoid_is_continuous() {
        let (total, f) = t_curve(1.0, 3.5, 7.0);
        assert!((total - 2.0 * (1.0f64 / 7.0).sqrt()).abs() < 1e-12);
        let mid = Duration::from_secs_f64(total / 2.0);
        // Within the nanosecond resolution of `Duration`.
        assert!((f(mid) - 0.5).abs() < 1e-6);
        let mut curve = JointSCurve::new([3.5], [7.0], [f64::INFINITY]);
        let total = curve.plan([0.0], [1.0]);
        let mut previous = 0.0;
        for k in 0..=total.as_millis() as u64 {
            let q = curve.apply(Duration::from_millis(k)).0[0];
            assert!((q - previous).abs() < 0.01, "t = {k} ms");
            previous = q;
        }
    }

    #[test]
    fn test_joint_linear() {
        let start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let end = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let v_max = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let (f, _) = joint_linear(&start, &end, &v_max);

        let t = Duration::from_secs_f64(0.5);
        let result = f(t);
        assert_eq!(result, [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]);
    }

    #[test]
    fn test_cartesian_quat_linear() {
        let start = na::Isometry3::identity();
        let end = na::Isometry3::from_parts(
            na::Translation3::new(1.0, 1.0, 1.0),
            na::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3),
        );
        let v_max = 1.0;
        let (f, _) = cartesian_quat_linear(start, end, v_max);

        for i in 0..210 {
            let t = Duration::from_secs_f64(i as f64 / 100.);
            let result = f(t);
            println!("time: {} | {:?}", i as f64 / 100., result);
        }
    }

    #[test]
    fn test_joint_trapezoid() {
        let start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let end = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let v_max = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let a_max = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let (f, _) = joint_trapezoid(&start, &end, &v_max, &a_max);

        for i in 0..210 {
            let t = Duration::from_secs_f64(i as f64 / 100.);
            let result = f(t);
            println!("time: {} | {:?}", i as f64 / 100., result);
        }
    }

    #[test]
    fn test_joint_s_curve_struct_endpoints() {
        let start = [-0.2, -0.77, -0.36, -2.42, 0.00, 2.57, 0.77];
        let end = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
        let v_max = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100];
        let a_max = [15., 7.5, 10., 12.5, 15., 20., 20.];
        let j_max = [100., 100., 100., 100., 100., 100., 100.];

        let mut curve = JointSCurve::new(v_max, a_max, j_max);
        let total = curve.plan(start, end);

        let (first, first_done) = curve.apply(Duration::ZERO);
        assert_eq!(first, start);
        assert!(!first_done);

        let (last, last_done) = curve.apply(total);
        assert_eq!(last, end);
        assert!(last_done);
    }

    #[test]
    fn test_joint_s_curve_struct_keeps_joint_line() {
        let start = [0.0, -0.3, 0.7];
        let end = [1.0, 0.1, -0.1];
        let v_max = [1.0, 0.8, 1.2];
        let a_max = [2.0, 1.5, 2.5];
        let j_max = [8.0, 7.0, 9.0];

        let mut curve = JointSCurve::new(v_max, a_max, j_max);
        let total = curve.plan(start, end);
        let (mid, done) = curve.apply(total / 2);

        assert!(!done);
        let progress = (mid[0] - start[0]) / (end[0] - start[0]);
        for i in 1..3 {
            let joint_progress = (mid[i] - start[i]) / (end[i] - start[i]);
            assert!((joint_progress - progress).abs() < 1e-12);
        }
    }

    use plotters::prelude::*;

    #[test]
    fn test_joint_s_curve() {
        let start = [-0.2, -0.77, -0.36, -2.42, 0.00, 2.57, 0.77];
        let end = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
        let v_max = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100];
        let a_max = [15., 7.5, 10., 12.5, 15., 20., 20.];
        let j_max = [100., 100., 100., 100., 100., 100., 100.];
        let (f, _) = joint_s_curve(&start, &end, &v_max, &a_max, &j_max);
        let f_dot = |t: Duration| {
            let dt: Duration = Duration::from_secs_f64(0.001);
            let t_dot = t + dt;
            f(t_dot)
                .iter()
                .zip(f(t).iter())
                .map(|(a, b)| (a - b) / dt.as_secs_f64())
                .collect::<Vec<f64>>()
        };
        let f_ddot = |t: Duration| {
            let dt = Duration::from_secs_f64(0.001);
            let t_dot = t + dt;
            let t_ddot = t_dot + dt;
            f_dot(t_ddot)
                .iter()
                .zip(f_dot(t_dot).iter())
                .map(|(a, b)| (a - b) / dt.as_secs_f64())
                .collect::<Vec<f64>>()
        };
        let f_dddot = |t: Duration| {
            let dt = Duration::from_secs_f64(0.001);
            let t_dot = t + dt;
            let t_ddot = t_dot + dt;
            let t_dddot = t_ddot + dt;
            f_ddot(t_dddot)
                .iter()
                .zip(f_ddot(t_ddot).iter())
                .map(|(a, b)| (a - b) / dt.as_secs_f64())
                .collect::<Vec<f64>>()
        };

        let root = plotters::prelude::SVGBackend::new("plot.svg", (640, 480)).into_drawing_area();
        root.fill(&plotters::prelude::WHITE).unwrap();

        let mut chart = plotters::prelude::ChartBuilder::on(&root)
            .caption("S-Curve", ("sans-serif", 50).into_font())
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(30)
            .build_cartesian_2d(0.0..1.0, -5.0..5.0)
            .unwrap();

        chart.configure_mesh().draw().unwrap();

        for j in 0..7 {
            let mut data = Vec::new();
            for i in 0..1000 {
                let t = i as f64 / 1000.0;
                let y = f(Duration::from_secs_f64(t));
                data.push((t, y[j]));
            }

            let mut data_dot = Vec::new();
            for i in 0..1000 {
                let t = i as f64 / 1000.0;
                let y = f_dot(Duration::from_secs_f64(t));
                data_dot.push((t, y[j]));
            }

            let mut data_ddot = Vec::new();
            for i in 0..1000 {
                let t = i as f64 / 1000.0;
                let y = f_ddot(Duration::from_secs_f64(t));
                data_ddot.push((t, y[j]));
            }

            let mut data_dddot = Vec::new();
            for i in 0..1000 {
                let t = i as f64 / 1000.0;
                let y = f_dddot(Duration::from_secs_f64(t));
                data_dddot.push((t, y[j]));
            }

            for i in 0..1000 {
                println!(
                    "time: {} | {}, {}, {}",
                    i as f64 / 1000.,
                    data[i].1,
                    data_dot[i].1,
                    data_ddot[i].1
                );
            }

            chart
                .draw_series(plotters::prelude::LineSeries::new(
                    data,
                    &plotters::prelude::RED,
                ))
                .unwrap();

            // chart
            //     .draw_series(plotters::prelude::LineSeries::new(
            //         data_dot,
            //         &plotters::prelude::GREEN,
            //     ))
            //     .unwrap();

            // chart
            //     .draw_series(plotters::prelude::LineSeries::new(
            //         data_ddot,
            //         &plotters::prelude::BLUE,
            //     ))
            //     .unwrap();
        }
    }

    #[test]
    fn test_s_curve() {
        let delta = 2.57 - PI / 2.;
        let v_max = 2.6100;
        let a_max = 20.;
        let j_max = 10000.;
        let (t, f) = s_curve(delta, v_max, a_max, j_max);
        println!("t: {}", t);

        let root = SVGBackend::new("plot.svg", (640, 480)).into_drawing_area();
        root.fill(&WHITE).unwrap();

        let mut chart = ChartBuilder::on(&root)
            .caption("S-Curve", ("sans-serif", 50).into_font())
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(30)
            .build_cartesian_2d(0.0..1.0, 0.0..2.)
            .unwrap();

        chart.configure_mesh().draw().unwrap();

        let mut data = Vec::new();
        for i in 0..1000 {
            let t = i as f64 / 100.0;
            let y = f(Duration::from_secs_f64(t));
            data.push((t, y));
        }

        chart.draw_series(LineSeries::new(data, &RED)).unwrap();
    }

    #[test]
    fn test_joint_simple_4th_curve() {
        let start = [-0.2, -0.77, -0.36, -2.42, 0.00, 2.57, 0.77];
        let end = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
        let v_max = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100];
        let a_max = [15., 7.5, 10., 12.5, 15., 20., 20.];
        let (f, _) = joint_simple_4th_curve(&start, &end, &v_max, &a_max);
        let f_dot = |t: Duration| {
            let dt: Duration = Duration::from_secs_f64(0.001);
            let t_dot = t + dt;
            f(t_dot)
                .iter()
                .zip(f(t).iter())
                .map(|(a, b)| (a - b) / dt.as_secs_f64())
                .collect::<Vec<f64>>()
        };
        let f_ddot = |t: Duration| {
            let dt = Duration::from_secs_f64(0.001);
            let t_dot = t + dt;
            let t_ddot = t_dot + dt;
            f_dot(t_ddot)
                .iter()
                .zip(f_dot(t_dot).iter())
                .map(|(a, b)| (a - b) / dt.as_secs_f64())
                .collect::<Vec<f64>>()
        };

        for i in 0..210 {
            let t = Duration::from_secs_f64(i as f64 / 100.);
            let result = f(t);
            println!("time: {} | {:?}", i as f64 / 100., result);
        }

        let root = plotters::prelude::SVGBackend::new("plot.svg", (640, 480)).into_drawing_area();
        root.fill(&plotters::prelude::WHITE).unwrap();

        let mut chart = plotters::prelude::ChartBuilder::on(&root)
            .caption("Simple 4th Curve", ("sans-serif", 50).into_font())
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(30)
            .build_cartesian_2d(0.0..1.0, -10.0..10.0)
            .unwrap();

        chart.configure_mesh().draw().unwrap();

        for j in 0..7 {
            let mut data = Vec::new();
            for i in 0..2000 {
                let t = i as f64 / 1000.0;
                let y = f(Duration::from_secs_f64(t));
                data.push((t, y[j]));
            }

            let mut data_dot = Vec::new();
            for i in 0..2000 {
                let t = i as f64 / 1000.0;
                let y = f_dot(Duration::from_secs_f64(t));
                data_dot.push((t, y[j]));
            }

            let mut data_ddot = Vec::new();
            for i in 0..2000 {
                let t = i as f64 / 1000.0;
                let y = f_ddot(Duration::from_secs_f64(t));
                data_ddot.push((t, y[j]));
            }

            // for i in 0..1000 {
            //     println!(
            //         "time: {} | {}, {}",
            //         i as f64 / 1000.,
            //         data[i].1,
            //         data_dot[i].1
            //     );
            // }

            // chart
            //     .draw_series(plotters::prelude::LineSeries::new(
            //         data,
            //         &plotters::prelude::RED,
            //     ))
            //     .unwrap();

            // chart
            //     .draw_series(plotters::prelude::LineSeries::new(
            //         data_dot,
            //         &plotters::prelude::GREEN,
            //     ))
            //     .unwrap();

            chart
                .draw_series(plotters::prelude::LineSeries::new(
                    data_ddot,
                    &plotters::prelude::BLUE,
                ))
                .unwrap();
        }
    }
}
