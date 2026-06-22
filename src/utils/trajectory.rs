use nalgebra as na;
use std::fmt::Display;

use std::time::Duration;

use crate::{JointState, Joints, Robot, RobotException, RobotResult};

/// Number of `s` samples used to discretise the geometric path for copp.
pub const COPP_PATH_SAMPLES: usize = 1001;

/// Run copp's TOPP2-RA → TOPP3-SOCP (2-iteration SCP) pipeline on a discretised
/// joint-space path and return the time-uniform `s(t)` grid.
pub fn plan_s_t_via_copp<R, const N: usize>(
    q_grid: &na::DMatrix<f64>,
    dq_grid: &na::DMatrix<f64>,
    ddq_grid: &na::DMatrix<f64>,
    dddq_grid: Option<&na::DMatrix<f64>>,
    scale: f64,
) -> RobotResult<Vec<f64>>
where
    R: Joints<N> + Robot + ?Sized,
{
    use copp::InterpolationMode;
    use copp::robot::Robot;
    use copp::solver::topp2_ra::{ReachSet2OptionsBuilder, Topp2ProblemBuilder, topp2_ra};
    use copp::solver::topp3_socp::{
        ClarabelOptionsBuilder, Topp3ProblemBuilder, s_to_t_topp3, t_to_s_topp3, topp3_socp,
    };

    let dt = R::CONTROL_PERIOD;
    if !(dt.is_finite() && dt > 0.0) {
        return Err(RobotException::UnprocessableInstructionError(format!(
            "Robot::CONTROL_PERIOD must be a positive finite number, got {dt}"
        )));
    }
    if !(scale.is_finite() && scale > 0.0) {
        return Err(RobotException::UnprocessableInstructionError(format!(
            "Arm::get_scale must return a positive finite number, got {scale}"
        )));
    }

    let joint_vel_bound = R::JOINT_VEL_BOUND.map(|v| v * scale);
    let joint_acc_bound = R::JOINT_ACC_BOUND.map(|v| v * scale);
    let joint_jerk_bound = R::JOINT_JERK_BOUND.map(|v| v * scale);

    fn map_err<E: Display>(e: E) -> RobotException {
        RobotException::UnprocessableInstructionError(format!("copp planning failed: {e}"))
    }

    let n = COPP_PATH_SAMPLES;
    let s: Vec<f64> = (0..n).map(|j| j as f64 / (n - 1) as f64).collect();

    let mut robot = Robot::with_capacity(N, n);
    robot.with_s(s.as_slice()).map_err(map_err)?;
    robot
        .with_q(
            &q_grid.as_view(),
            &dq_grid.as_view(),
            &ddq_grid.as_view(),
            dddq_grid.map(|m| m.as_view()).as_ref(),
            0,
        )
        .map_err(map_err)?;
    robot
        .with_axial_velocity(
            (joint_vel_bound.as_slice(), n),
            (joint_vel_bound.map(|v| -v).as_slice(), n),
            0,
        )
        .map_err(map_err)?;
    robot
        .with_axial_acceleration(
            (joint_acc_bound.as_slice(), n),
            (joint_acc_bound.map(|v| -v).as_slice(), n),
            0,
        )
        .map_err(map_err)?;
    robot
        .with_axial_jerk(
            (joint_jerk_bound.as_slice(), n),
            (joint_jerk_bound.map(|v| -v).as_slice(), n),
            0,
        )
        .map_err(map_err)?;

    let idx_s_interval = (0, n - 1);
    let a_boundary = (0.0, 0.0);
    let a_ra0 = {
        let prob = Topp2ProblemBuilder::new(&robot, idx_s_interval, a_boundary)
            .build()
            .map_err(map_err)?;
        let opts = ReachSet2OptionsBuilder::new().build().map_err(map_err)?;
        topp2_ra(&prob, &opts).map_err(map_err)?
    };
    let a_ra0: Vec<f64> = a_ra0.into_iter().map(|a| a.max(0.0)).collect();
    robot
        .constraints
        .amax_substitute(&a_ra0, 0)
        .map_err(map_err)?;

    let opts_socp = ClarabelOptionsBuilder::new()
        .allow_almost_solved(true)
        .allow_insufficient_progress(true)
        .allow_max_iterations(true)
        .build()
        .map_err(map_err)?;

    let profile_qp1 = {
        let prob =
            Topp3ProblemBuilder::new(&mut robot, idx_s_interval.0, &a_ra0, (0.0, 0.0), (0.0, 0.0))
                .build_with_linearization()
                .map_err(map_err)?;
        topp3_socp(&prob, &opts_socp).map_err(map_err)?
    };
    let a_qp1: Vec<f64> = profile_qp1.a.iter().map(|&a| a.max(0.0)).collect();
    let profile_qp2 = {
        let prob =
            Topp3ProblemBuilder::new(&mut robot, idx_s_interval.0, &a_qp1, (0.0, 0.0), (0.0, 0.0))
                .build_with_linearization()
                .map_err(map_err)?;
        match topp3_socp(&prob, &opts_socp) {
            Ok(res) => res,
            Err(_) => {
                let prob = Topp3ProblemBuilder::new(
                    &mut robot,
                    idx_s_interval.0,
                    &a_ra0,
                    (0.0, 0.0),
                    (0.0, 0.0),
                )
                .build_with_linearization()
                .map_err(map_err)?;
                topp3_socp(&prob, &opts_socp).map_err(map_err)?
            }
        }
    };

    let (_t_final, t_s) = s_to_t_topp3(&s, profile_qp2.as_parts(), 0.0).map_err(map_err)?;
    let s_t = t_to_s_topp3(
        &s,
        profile_qp2.as_parts(),
        &t_s,
        InterpolationMode::UniformTimeGrid(0.0, dt, true),
    )
    .map_err(map_err)?;
    if s_t.is_empty() {
        return Err(RobotException::UnprocessableInstructionError(
            "copp t_to_s_topp3 produced an empty time grid".to_string(),
        ));
    }
    Ok(s_t)
}

/// Plan a time-uniform joint trajectory through discrete `waypoints` via copp.
pub fn plan_waypoints_traj_via_copp<R, const N: usize>(
    waypoints: &[[f64; N]],
    scale: f64,
) -> RobotResult<Vec<[f64; N]>>
where
    R: Joints<N> + Robot + ?Sized,
{
    use copp::path::{Path, SplineConfig};

    if waypoints.len() < 2 {
        return Err(RobotException::UnprocessableInstructionError(
            "joint waypoint planning requires at least 2 waypoints".to_string(),
        ));
    }

    let n_pts = waypoints.len();
    let wp_mat = na::DMatrix::<f64>::from_fn(N, n_pts, |i, j| waypoints[j][i]);
    let path = Path::from_waypoints(&wp_mat, SplineConfig::default()).map_err(|e| {
        RobotException::UnprocessableInstructionError(format!(
            "copp Path::from_waypoints failed: {e}"
        ))
    })?;

    let n = COPP_PATH_SAMPLES;
    let s: Vec<f64> = (0..n).map(|j| j as f64 / (n - 1) as f64).collect();
    let derivs = path.evaluate_up_to_3rd(&s).map_err(|e| {
        RobotException::UnprocessableInstructionError(format!("path.evaluate_up_to_3rd: {e}"))
    })?;

    let s_t = plan_s_t_via_copp::<R, N>(
        &derivs.q,
        derivs.dq.as_ref().unwrap(),
        derivs.ddq.as_ref().unwrap(),
        derivs.dddq.as_ref(),
        scale,
    )?;

    let q_t = path.evaluate_q(&s_t).map_err(|e| {
        RobotException::UnprocessableInstructionError(format!("path.evaluate_q: {e}"))
    })?;
    Ok(q_grid_to_traj::<N>(&q_t.q))
}

/// Plan a time-uniform joint trajectory along a continuous user-supplied path.
pub fn plan_path_traj_via_copp<P, F, const N: usize>(
    path_fn: &F,
    scale: f64,
) -> RobotResult<Vec<[f64; N]>>
where
    P: Joints<N> + Robot + ?Sized,
    F: Fn(f64) -> Option<[f64; N]>,
{
    let n = COPP_PATH_SAMPLES;
    let s_grid: Vec<f64> = (0..n).map(|j| j as f64 / (n - 1) as f64).collect();
    let (q_grid, dq_grid, ddq_grid, dddq_grid) =
        finite_difference_path_derivatives(path_fn, &s_grid)?;

    let s_t = plan_s_t_via_copp::<P, N>(&q_grid, &dq_grid, &ddq_grid, Some(&dddq_grid), scale)?;

    let mut traj = Vec::with_capacity(s_t.len());
    for &s in &s_t {
        match path_fn(s) {
            Some(q) => traj.push(q),
            None => {
                return Err(RobotException::UnprocessableInstructionError(format!(
                    "move_path closure returned None at s={s} during final resampling"
                )));
            }
        }
    }
    Ok(traj)
}

/// Wrap an already sampled joint trajectory as a `JointPositionControl` closure.
///
/// The closure emits one trajectory sample per non-zero control tick and reports
/// `done = true` after the final sample. It is intentionally open-loop: it only
/// generates position commands, while any feedback behavior should be layered
/// through PID / impedance / dynamics controller helpers.
pub fn joint_traj_position_control<const N: usize>(
    traj: Vec<[f64; N]>,
) -> impl FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static {
    let mut step = 0usize;

    move |state, duration| {
        if traj.is_empty() {
            return (
                state
                    .cmd
                    .q
                    .or(state.des.q)
                    .or(state.meas.q)
                    .unwrap_or([0.0; N]),
                true,
            );
        }
        if duration == Duration::ZERO && step == 0 {
            return (traj[0], false);
        }

        let index = step.min(traj.len() - 1);
        let target = traj[index];
        if duration > Duration::ZERO {
            step += 1;
        }
        (target, step >= traj.len())
    }
}

/// Plan waypoints with copp and return a closure usable by
/// `control_with::<JointPositionControl<N>, _>(...)`.
pub fn copp_waypoints_joint_position_control<R, const N: usize>(
    waypoints: &[[f64; N]],
    scale: f64,
) -> RobotResult<impl FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static>
where
    R: Joints<N> + Robot + ?Sized,
{
    let traj = plan_waypoints_traj_via_copp::<R, N>(waypoints, scale)?;
    Ok(joint_traj_position_control(traj))
}

/// Plan a continuous path with copp and return a closure usable by
/// `control_with::<JointPositionControl<N>, _>(...)`.
pub fn copp_path_joint_position_control<R, F, const N: usize>(
    path_fn: &F,
    scale: f64,
) -> RobotResult<impl FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static>
where
    R: Joints<N> + Robot + ?Sized,
    F: Fn(f64) -> Option<[f64; N]>,
{
    let traj = plan_path_traj_via_copp::<R, F, N>(path_fn, scale)?;
    Ok(joint_traj_position_control(traj))
}

fn finite_difference_path_derivatives<F, const N: usize>(
    path_fn: &F,
    s_grid: &[f64],
) -> RobotResult<(
    na::DMatrix<f64>,
    na::DMatrix<f64>,
    na::DMatrix<f64>,
    na::DMatrix<f64>,
)>
where
    F: Fn(f64) -> Option<[f64; N]>,
{
    if s_grid.len() < 5 {
        return Err(RobotException::UnprocessableInstructionError(
            "move_path finite-difference planning requires at least 5 path samples".to_string(),
        ));
    }

    let mut q_samples = Vec::with_capacity(s_grid.len());
    for &s in s_grid {
        match path_fn(s) {
            Some(q) if q.iter().all(|v| v.is_finite()) => q_samples.push(q),
            Some(_) => {
                return Err(RobotException::UnprocessableInstructionError(format!(
                    "move_path closure returned non-finite joint values at s={s}"
                )));
            }
            None => {
                return Err(RobotException::UnprocessableInstructionError(format!(
                    "move_path closure returned None at s={s}; must be defined on [0, 1]"
                )));
            }
        }
    }

    let n = s_grid.len();
    let q = na::DMatrix::<f64>::from_fn(N, n, |i, j| q_samples[j][i]);
    let mut dq = na::DMatrix::<f64>::zeros(N, n);
    let mut ddq = na::DMatrix::<f64>::zeros(N, n);
    let mut dddq = na::DMatrix::<f64>::zeros(N, n);

    for j in 0..n {
        let start = j.saturating_sub(2).min(n - 5);
        let indices = [start, start + 1, start + 2, start + 3, start + 4];
        let offsets = indices.map(|idx| s_grid[idx] - s_grid[j]);
        let weights_d1 = finite_difference_weights(&offsets, 1);
        let weights_d2 = finite_difference_weights(&offsets, 2);
        let weights_d3 = finite_difference_weights(&offsets, 3);

        for axis in 0..N {
            for (k, &idx) in indices.iter().enumerate() {
                let q_value = q_samples[idx][axis];
                dq[(axis, j)] += weights_d1[k] * q_value;
                ddq[(axis, j)] += weights_d2[k] * q_value;
                dddq[(axis, j)] += weights_d3[k] * q_value;
            }
        }
    }

    Ok((q, dq, ddq, dddq))
}

fn finite_difference_weights(offsets: &[f64; 5], derivative_order: usize) -> [f64; 5] {
    let mut matrix = [[0.0f64; 6]; 5];
    for row in 0..5 {
        for (col, &offset) in offsets.iter().enumerate() {
            matrix[row][col] = offset.powi(row as i32);
        }
        matrix[row][5] = if row == derivative_order {
            factorial(derivative_order) as f64
        } else {
            0.0
        };
    }

    for pivot in 0..5 {
        let pivot_row = (pivot..5)
            .max_by(|&a, &b| matrix[a][pivot].abs().total_cmp(&matrix[b][pivot].abs()))
            .unwrap();
        matrix.swap(pivot, pivot_row);

        let pivot_value = matrix[pivot][pivot];
        for col in pivot..6 {
            matrix[pivot][col] /= pivot_value;
        }

        for row in 0..5 {
            if row == pivot {
                continue;
            }
            let factor = matrix[row][pivot];
            for col in pivot..6 {
                matrix[row][col] -= factor * matrix[pivot][col];
            }
        }
    }

    matrix.map(|row| row[5])
}

fn factorial(n: usize) -> usize {
    (1..=n).product()
}

/// Convert a column-major `(N, M)` joint matrix to `Vec<[f64; N]>`.
pub fn q_grid_to_traj<const N: usize>(q: &na::DMatrix<f64>) -> Vec<[f64; N]> {
    let mut traj = Vec::with_capacity(q.ncols());
    for j in 0..q.ncols() {
        let mut joint = [0.0f64; N];
        for (i, slot) in joint.iter_mut().enumerate().take(N) {
            *slot = q[(i, j)];
        }
        traj.push(joint);
    }
    traj
}
