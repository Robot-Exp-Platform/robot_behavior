use nalgebra as na;
use std::{time::Duration, usize};

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

pub fn joint_s_curve<const N: usize>(
    start: &[f64; N],
    end: &[f64; N],
    v_max: &[f64; N],
    a_max: &[f64; N],
    j_max: &[f64; N],
) -> Box<dyn Fn(Duration) -> [f64; N]> {
    let start = na::SVector::<f64, N>::from_column_slice(start);
    let end = na::SVector::<f64, N>::from_column_slice(end);
    let delta = end - start;

    let mut t_path = Vec::with_capacity(N);
    let mut f_path = Vec::with_capacity(N);
    for i in 0..N {
        let (t, f) = s_curve(delta[i].abs(), v_max[i], a_max[i], j_max[i]);
        t_path.push(t);
        f_path.push(f);
    }

    let t_max = t_path.iter().cloned().fold(0.0, f64::max);

    let f = move |t: Duration| {
        let t = t.as_secs_f64();
        let t = t / t_max;
        let mut result = start.clone();
        for i in 0..N {
            result[i] += delta[i].signum() * f_path[i](Duration::from_secs_f64(t * t_path[i]));
        }
        result.into()
    };

    Box::new(f)
}

fn s_curve(delta: f64, v_max: f64, a_max: f64, j_max: f64) -> (f64, Box<dyn Fn(Duration) -> f64>) {
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
            Box::new(move |t: Duration| {
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
            Box::new(move |t: Duration| {
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
            Box::new(move |t: Duration| {
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
) -> Box<dyn Fn(Duration) -> [f64; N]> {
    let start = na::SVector::<f64, N>::from_column_slice(start);
    let end = na::SVector::<f64, N>::from_column_slice(end);
    let delta = end - start;

    let mut t_path = Vec::with_capacity(N);
    let mut f_path = Vec::with_capacity(N);
    for i in 0..N {
        let (t, f) = simple_4th_curve(delta[i].abs(), v_max[i], a_max[i]);
        t_path.push(t);
        f_path.push(f);
    }

    let t_max = t_path.iter().cloned().fold(0.0, f64::max);

    let f = move |t: Duration| {
        let t = t.as_secs_f64();
        let t = t / t_max;
        let mut result = start.clone();
        for i in 0..N {
            result[i] += delta[i].signum() * f_path[i](Duration::from_secs_f64(t * t_path[i]));
        }
        result.into()
    };

    Box::new(f)
}

fn simple_4th_curve(delta: f64, v_max: f64, a_max: f64) -> (f64, Box<dyn Fn(Duration) -> f64>) {
    if delta < 1e-6 {
        return (0., Box::new(|_| 0.));
    }
    let mut v_max = v_max;
    if delta < 1.5 * v_max.powi(2) / a_max {
        v_max = (2. / 3. * delta * a_max).sqrt();
    }

    let t1 = 1.5 * v_max / a_max;
    let t_min =  t1 + delta / v_max;

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

    (t_min, Box::new(f))
}

#[cfg(test)]
mod test {
    use std::f64::consts::PI;

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
        let f: Box<
            dyn Fn(
                Duration,
            )
                -> nalgebra::Isometry<f64, nalgebra::Unit<nalgebra::Quaternion<f64>>, 3>,
        > = cartesian_quat_linear(start, end, v_max);

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

    use plotters::prelude::*;

    #[test]
    fn test_joint_s_curve() {
        let start = [-0.2, -0.77, -0.36, -2.42, 0.00, 2.57, 0.77];
        let end = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
        let v_max = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100];
        let a_max = [15., 7.5, 10., 12.5, 15., 20., 20.];
        let j_max = [100., 100., 100., 100., 100., 100., 100.];
        let f = joint_s_curve(&start, &end, &v_max, &a_max, &j_max);
        let f_dot = |t: Duration| {
            let dt: Duration = Duration::from_secs_f64(0.001);
            let t_dot = t + dt;
            let result = f(t_dot)
                .iter()
                .zip(f(t).iter())
                .map(|(a, b)| (a - b) / dt.as_secs_f64())
                .collect::<Vec<f64>>();
            result
        };
        let f_ddot = |t: Duration| {
            let dt = Duration::from_secs_f64(0.001);
            let t_dot = t + dt;
            let t_ddot = t_dot + dt;
            let result = f_dot(t_ddot)
                .iter()
                .zip(f_dot(t_dot).iter())
                .map(|(a, b)| (a - b) / dt.as_secs_f64())
                .collect::<Vec<f64>>();
            result
        };
        let f_dddot = |t: Duration| {
            let dt = Duration::from_secs_f64(0.001);
            let t_dot = t + dt;
            let t_ddot = t_dot + dt;
            let t_dddot = t_ddot + dt;
            let result = f_ddot(t_dddot)
                .iter()
                .zip(f_ddot(t_ddot).iter())
                .map(|(a, b)| (a - b) / dt.as_secs_f64())
                .collect::<Vec<f64>>();
            result
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
        let f = joint_simple_4th_curve(&start, &end, &v_max, &a_max);
        let f_dot = |t: Duration| {
            let dt: Duration = Duration::from_secs_f64(0.001);
            let t_dot = t + dt;
            let result = f(t_dot)
                .iter()
                .zip(f(t).iter())
                .map(|(a, b)| (a - b) / dt.as_secs_f64())
                .collect::<Vec<f64>>();
            result
        };
        let f_ddot = |t: Duration| {
            let dt = Duration::from_secs_f64(0.001);
            let t_dot = t + dt;
            let t_ddot = t_dot + dt;
            let result = f_dot(t_ddot)
                .iter()
                .zip(f_dot(t_dot).iter())
                .map(|(a, b)| (a - b) / dt.as_secs_f64())
                .collect::<Vec<f64>>();
            result
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
