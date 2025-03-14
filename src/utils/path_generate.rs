use nalgebra as na;
use std::time::Duration;

/// Generate a linear path for joint space
pub fn joint_linear<const N: usize>(
    start: &[f64; N],
    end: &[f64; N],
    v_max: &[f64; N],
) -> Box<dyn Fn(Duration) -> [f64; N]> {
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

    Box::new(f)
}

/// Generate a linear path for cartesian space
pub fn cartesian_quat_linear(
    start: na::Isometry3<f64>,
    end: na::Isometry3<f64>,
    v_max: f64,
) -> Box<dyn Fn(Duration) -> na::Isometry3<f64>> {
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
        .into()
    };

    Box::new(f)
}

pub fn joint_trapezoid<const N: usize>(
    start: &[f64; N],
    end: &[f64; N],
    v_max: &[f64; N],
    a_max: &[f64; N],
) -> Box<dyn Fn(Duration) -> [f64; N]> {
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

    Box::new(f)
}

fn trapezoid_min_time(para: ((&f64, &f64), &f64)) -> f64 {
    let ((d, v_max), a_max) = para;
    if *d > v_max.powi(2) / *a_max {
        d / v_max + v_max / a_max
    } else {
        2. * (d / a_max).sqrt()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_joint_linear() {
        let start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let end = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let v_max = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let f = joint_linear(&start, &end, &v_max);

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
        let f = cartesian_quat_linear(start, end, v_max);

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
        let f = joint_trapezoid(&start, &end, &v_max, &a_max);

        for i in 0..210 {
            let t = Duration::from_secs_f64(i as f64 / 100.);
            let result = f(t);
            println!("time: {} | {:?}", i as f64 / 100., result);
        }
    }
}
