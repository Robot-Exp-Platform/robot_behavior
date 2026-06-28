use nalgebra as na;
use std::fmt::Display;

use std::time::Duration;

use crate::{JointState, Joints, Pose, Robot, RobotException, RobotResult};

use super::path_generate::joint_s_curve;

/// Number of `s` samples used to discretise the geometric path for copp.
pub const COPP_PATH_SAMPLES: usize = 1001;

/// Default number of IK samples used to discretise a Cartesian geometric path.
pub const CARTESIAN_TIME_SCALE_PATH_SAMPLES: usize = 201;

/// Options for non-COPP Cartesian path time scaling.
///
/// The planner first samples IK along a geometric Cartesian path `x(s)`, then
/// generates a scalar S-curve `s(t)`. The final dense joint trajectory is
/// validated against joint velocity and acceleration limits. Jerk validation is
/// optional because the sampled IK path is interpolated and therefore is not
/// guaranteed to be globally C3-smooth.
#[derive(Debug, Clone, Copy)]
pub struct CartesianTimeScalingOptions {
    /// Number of IK samples over `s in [0, 1]`. Must be at least 5.
    pub path_samples: usize,
    /// Extra conservative factor applied to the initial scalar S-curve bounds.
    pub safety_scale: f64,
    /// Bound shrink factor used when validation fails.
    pub retry_scale: f64,
    /// Maximum validation / shrink retries.
    pub max_retries: usize,
    /// Whether to validate jerk with finite differences on the final trajectory.
    pub validate_jerk: bool,
}

impl Default for CartesianTimeScalingOptions {
    fn default() -> Self {
        Self {
            path_samples: CARTESIAN_TIME_SCALE_PATH_SAMPLES,
            safety_scale: 0.8,
            retry_scale: 0.8,
            max_retries: 16,
            validate_jerk: false,
        }
    }
}

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

/// Plan a Cartesian geometric path without copp by scalar time scaling.
///
/// The path is a purely geometric curve `x(s), s in [0, 1]`. The caller supplies
/// an IK function that receives `(pose, seed)` and returns a joint solution. IK
/// is sampled once along the path with the previous solution as the next seed,
/// then a scalar S-curve `s(t)` is generated and shrunk until the dense joint
/// trajectory satisfies the robot's joint velocity / acceleration limits.
///
/// This is intentionally not time-optimal. It is a compact, deterministic
/// planner for drivers that want a Cartesian straight-line style command while
/// still executing through joint-space constraints.
pub fn plan_cartesian_path_traj_via_time_scaling<R, F, IK, const N: usize>(
    path_fn: F,
    q_start: [f64; N],
    scale: f64,
    ik: IK,
) -> RobotResult<Vec<[f64; N]>>
where
    R: Joints<N> + Robot + ?Sized,
    F: Fn(f64) -> Pose,
    IK: FnMut(Pose, [f64; N]) -> RobotResult<[f64; N]>,
{
    plan_cartesian_path_traj_via_time_scaling_with_options::<R, F, IK, N>(
        path_fn,
        q_start,
        scale,
        ik,
        CartesianTimeScalingOptions::default(),
    )
}

/// Same as [`plan_cartesian_path_traj_via_time_scaling`] with explicit options.
pub fn plan_cartesian_path_traj_via_time_scaling_with_options<R, F, IK, const N: usize>(
    path_fn: F,
    q_start: [f64; N],
    scale: f64,
    mut ik: IK,
    options: CartesianTimeScalingOptions,
) -> RobotResult<Vec<[f64; N]>>
where
    R: Joints<N> + Robot + ?Sized,
    F: Fn(f64) -> Pose,
    IK: FnMut(Pose, [f64; N]) -> RobotResult<[f64; N]>,
{
    validate_time_scaling_inputs::<R, N>(scale, options)?;

    let s_grid = unit_grid(options.path_samples);
    let mut q_samples = Vec::with_capacity(s_grid.len());
    let mut seed = q_start;
    for &s in &s_grid {
        let q = ik(path_fn(s), seed)?;
        if !q.iter().all(|v| v.is_finite()) {
            return Err(RobotException::UnprocessableInstructionError(format!(
                "cartesian time scaling IK returned non-finite joint values at s={s}"
            )));
        }
        q_samples.push(q);
        seed = q;
    }

    if joint_path_is_stationary(&q_samples) {
        return Ok(vec![q_start]);
    }

    let (_q_grid, dq_ds, ddq_ds, dddq_ds) =
        finite_difference_sample_derivatives::<N>(&q_samples, &s_grid)?;
    let (mut s_vel, mut s_acc, mut s_jerk) =
        scalar_s_curve_bounds::<R, N>(&dq_ds, &ddq_ds, &dddq_ds, scale, options.safety_scale)?;
    let dt = R::CONTROL_PERIOD;
    let mut last_error = String::new();

    for _attempt in 0..=options.max_retries {
        let (s_curve, total) = joint_s_curve::<1>(&[0.0], &[1.0], &[s_vel], &[s_acc], &[s_jerk]);
        let traj = sample_time_scaled_joint_path(&q_samples, &dq_ds, s_curve.as_ref(), total, dt);

        match joint_traj_limit_violation::<R, N>(&traj, dt, scale, options.validate_jerk) {
            None => return Ok(traj),
            Some(err) => last_error = err,
        }

        s_vel *= options.retry_scale;
        s_acc *= options.retry_scale * options.retry_scale;
        s_jerk *= options.retry_scale * options.retry_scale * options.retry_scale;
    }

    Err(RobotException::UnprocessableInstructionError(format!(
        "cartesian time scaling failed to satisfy joint limits after {} retries: {last_error}",
        options.max_retries
    )))
}

/// Plan a Cartesian straight line / slerp orientation path without copp.
pub fn plan_cartesian_line_traj_via_time_scaling<R, IK, const N: usize>(
    start: Pose,
    target: Pose,
    q_start: [f64; N],
    scale: f64,
    ik: IK,
) -> RobotResult<Vec<[f64; N]>>
where
    R: Joints<N> + Robot + ?Sized,
    IK: FnMut(Pose, [f64; N]) -> RobotResult<[f64; N]>,
{
    plan_cartesian_line_traj_via_time_scaling_with_options::<R, IK, N>(
        start,
        target,
        q_start,
        scale,
        ik,
        CartesianTimeScalingOptions::default(),
    )
}

/// Same as [`plan_cartesian_line_traj_via_time_scaling`] with explicit options.
pub fn plan_cartesian_line_traj_via_time_scaling_with_options<R, IK, const N: usize>(
    start: Pose,
    target: Pose,
    q_start: [f64; N],
    scale: f64,
    ik: IK,
    options: CartesianTimeScalingOptions,
) -> RobotResult<Vec<[f64; N]>>
where
    R: Joints<N> + Robot + ?Sized,
    IK: FnMut(Pose, [f64; N]) -> RobotResult<[f64; N]>,
{
    let start = start.quat();
    let target = target.quat();
    plan_cartesian_path_traj_via_time_scaling_with_options::<R, _, IK, N>(
        move |s| Pose::Quat(start.lerp_slerp(&target, s.clamp(0.0, 1.0))),
        q_start,
        scale,
        ik,
        options,
    )
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

fn validate_time_scaling_inputs<R, const N: usize>(
    scale: f64,
    options: CartesianTimeScalingOptions,
) -> RobotResult<()>
where
    R: Joints<N> + Robot + ?Sized,
{
    let dt = R::CONTROL_PERIOD;
    if !(dt.is_finite() && dt > 0.0) {
        return Err(RobotException::UnprocessableInstructionError(format!(
            "Robot::CONTROL_PERIOD must be a positive finite number, got {dt}"
        )));
    }
    if !(scale.is_finite() && scale > 0.0) {
        return Err(RobotException::UnprocessableInstructionError(format!(
            "scale must be a positive finite number, got {scale}"
        )));
    }
    if options.path_samples < 5 {
        return Err(RobotException::UnprocessableInstructionError(format!(
            "cartesian time scaling requires at least 5 path samples, got {}",
            options.path_samples
        )));
    }
    if !(options.safety_scale.is_finite() && options.safety_scale > 0.0) {
        return Err(RobotException::UnprocessableInstructionError(format!(
            "CartesianTimeScalingOptions::safety_scale must be positive finite, got {}",
            options.safety_scale
        )));
    }
    if !(options.retry_scale.is_finite() && options.retry_scale > 0.0 && options.retry_scale < 1.0)
    {
        return Err(RobotException::UnprocessableInstructionError(format!(
            "CartesianTimeScalingOptions::retry_scale must be in (0, 1), got {}",
            options.retry_scale
        )));
    }
    Ok(())
}

fn unit_grid(n: usize) -> Vec<f64> {
    (0..n).map(|i| i as f64 / (n - 1) as f64).collect()
}

fn joint_path_is_stationary<const N: usize>(q_samples: &[[f64; N]]) -> bool {
    let Some(first) = q_samples.first() else {
        return true;
    };
    q_samples
        .iter()
        .all(|q| q.iter().zip(first).all(|(a, b)| (*a - *b).abs() < 1e-10))
}

fn finite_difference_sample_derivatives<const N: usize>(
    q_samples: &[[f64; N]],
    s_grid: &[f64],
) -> RobotResult<(
    na::DMatrix<f64>,
    na::DMatrix<f64>,
    na::DMatrix<f64>,
    na::DMatrix<f64>,
)> {
    if q_samples.len() != s_grid.len() || q_samples.len() < 5 {
        return Err(RobotException::UnprocessableInstructionError(
            "joint sample derivative estimation requires matching grids with at least 5 samples"
                .to_string(),
        ));
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

fn scalar_s_curve_bounds<R, const N: usize>(
    dq_ds: &na::DMatrix<f64>,
    ddq_ds: &na::DMatrix<f64>,
    dddq_ds: &na::DMatrix<f64>,
    scale: f64,
    safety_scale: f64,
) -> RobotResult<(f64, f64, f64)>
where
    R: Joints<N> + Robot + ?Sized,
{
    let vel = R::JOINT_VEL_BOUND.map(|v| v * scale);
    let acc = R::JOINT_ACC_BOUND.map(|v| v * scale);
    let jerk = R::JOINT_JERK_BOUND.map(|v| v * scale);
    let eps = 1e-9;
    let mut s_vel = f64::INFINITY;
    let mut s_acc = f64::INFINITY;
    let mut s_jerk = f64::INFINITY;

    for axis in 0..N {
        let mut q1 = 0.0f64;
        let mut q2 = 0.0f64;
        let mut q3 = 0.0f64;
        for sample in 0..dq_ds.ncols() {
            q1 = q1.max(dq_ds[(axis, sample)].abs());
            q2 = q2.max(ddq_ds[(axis, sample)].abs());
            q3 = q3.max(dddq_ds[(axis, sample)].abs());
        }

        if q1 > eps {
            s_vel = s_vel.min(vel[axis] / q1);
            s_acc = s_acc.min(acc[axis] / q1);
            s_jerk = s_jerk.min(jerk[axis] / q1);
        }
        if q2 > eps {
            s_vel = s_vel.min((acc[axis] / q2).sqrt());
        }
        if q3 > eps {
            s_vel = s_vel.min((jerk[axis] / q3).cbrt());
        }
    }

    if !s_vel.is_finite() {
        s_vel = 1.0;
    }
    if !s_acc.is_finite() {
        s_acc = s_vel.max(1e-3);
    }
    if !s_jerk.is_finite() {
        s_jerk = s_acc.max(1e-3);
    }

    s_vel *= safety_scale;
    s_acc *= safety_scale;
    s_jerk *= safety_scale;

    if s_vel <= 0.0 || s_acc <= 0.0 || s_jerk <= 0.0 {
        return Err(RobotException::UnprocessableInstructionError(
            "cartesian time scaling produced non-positive scalar S-curve bounds".to_string(),
        ));
    }
    Ok((s_vel, s_acc, s_jerk))
}

fn sample_time_scaled_joint_path<const N: usize>(
    q_samples: &[[f64; N]],
    dq_ds: &na::DMatrix<f64>,
    s_curve: &dyn Fn(Duration) -> [f64; 1],
    total: Duration,
    dt: f64,
) -> Vec<[f64; N]> {
    let steps = (total.as_secs_f64() / dt).ceil() as usize;
    let mut traj = Vec::with_capacity(steps + 2);
    for step in 0..=steps {
        let t = (step as f64 * dt).min(total.as_secs_f64());
        let s = s_curve(Duration::from_secs_f64(t))[0].clamp(0.0, 1.0);
        traj.push(interpolate_joint_path(q_samples, dq_ds, s));
    }
    let end = *q_samples.last().unwrap();
    if traj.last().is_none_or(|q| {
        q.iter()
            .zip(end.iter())
            .any(|(a, b)| (*a - *b).abs() > 1e-10)
    }) {
        traj.push(end);
    }
    traj
}

fn interpolate_joint_path<const N: usize>(
    q_samples: &[[f64; N]],
    dq_ds: &na::DMatrix<f64>,
    s: f64,
) -> [f64; N] {
    if s <= 0.0 {
        return q_samples[0];
    }
    if s >= 1.0 {
        return *q_samples.last().unwrap();
    }

    let n = q_samples.len();
    let pos = s * (n - 1) as f64;
    let i0 = pos.floor() as usize;
    let i1 = (i0 + 1).min(n - 1);
    let u = pos - i0 as f64;
    let ds = 1.0 / (n - 1) as f64;

    let h00 = 2.0 * u.powi(3) - 3.0 * u.powi(2) + 1.0;
    let h10 = u.powi(3) - 2.0 * u.powi(2) + u;
    let h01 = -2.0 * u.powi(3) + 3.0 * u.powi(2);
    let h11 = u.powi(3) - u.powi(2);

    let mut q = [0.0; N];
    for axis in 0..N {
        q[axis] = h00 * q_samples[i0][axis]
            + h10 * ds * dq_ds[(axis, i0)]
            + h01 * q_samples[i1][axis]
            + h11 * ds * dq_ds[(axis, i1)];
    }
    q
}

fn joint_traj_limit_violation<R, const N: usize>(
    traj: &[[f64; N]],
    dt: f64,
    scale: f64,
    validate_jerk: bool,
) -> Option<String>
where
    R: Joints<N> + Robot + ?Sized,
{
    let vel = R::JOINT_VEL_BOUND.map(|v| v * scale);
    let acc = R::JOINT_ACC_BOUND.map(|v| v * scale);
    let jerk = R::JOINT_JERK_BOUND.map(|v| v * scale);

    for step in 1..traj.len() {
        for axis in 0..N {
            let value = ((traj[step][axis] - traj[step - 1][axis]) / dt).abs();
            if value > vel[axis] {
                return Some(format!(
                    "velocity axis {axis} at sample {step}: {value} > {}",
                    vel[axis]
                ));
            }
        }
    }

    for step in 2..traj.len() {
        for axis in 0..N {
            let value = ((traj[step][axis] - 2.0 * traj[step - 1][axis] + traj[step - 2][axis])
                / (dt * dt))
                .abs();
            if value > acc[axis] {
                return Some(format!(
                    "acceleration axis {axis} at sample {step}: {value} > {}",
                    acc[axis]
                ));
            }
        }
    }

    if validate_jerk {
        for step in 3..traj.len() {
            for axis in 0..N {
                let value = ((traj[step][axis] - 3.0 * traj[step - 1][axis]
                    + 3.0 * traj[step - 2][axis]
                    - traj[step - 3][axis])
                    / (dt * dt * dt))
                    .abs();
                if value > jerk[axis] {
                    return Some(format!(
                        "jerk axis {axis} at sample {step}: {value} > {}",
                        jerk[axis]
                    ));
                }
            }
        }
    }

    None
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

#[cfg(test)]
mod tests {
    use super::*;

    struct OneJointRobot;

    impl Robot for OneJointRobot {
        type State = ();
        const CONTROL_PERIOD: f64 = 0.001;

        fn version() -> String {
            "test".to_string()
        }

        fn read_state(&mut self) -> RobotResult<Self::State> {
            Ok(())
        }
    }

    impl Joints<1> for OneJointRobot {
        const JOINT_MIN: [f64; 1] = [-10.0];
        const JOINT_MAX: [f64; 1] = [10.0];
        const JOINT_VEL_BOUND: [f64; 1] = [1.0];
        const JOINT_ACC_BOUND: [f64; 1] = [2.0];
        const JOINT_JERK_BOUND: [f64; 1] = [20.0];
    }

    #[test]
    fn cartesian_line_time_scaling_generates_limited_joint_traj() {
        let options = CartesianTimeScalingOptions {
            path_samples: 21,
            safety_scale: 0.7,
            retry_scale: 0.8,
            max_retries: 8,
            validate_jerk: false,
        };
        let traj = plan_cartesian_line_traj_via_time_scaling_with_options::<OneJointRobot, _, 1>(
            Pose::Position([0.0, 0.0, 0.0]),
            Pose::Position([0.2, 0.0, 0.0]),
            [0.0],
            1.0,
            |pose, _seed| Ok([pose.position()[0]]),
            options,
        )
        .unwrap();

        assert!(traj.len() > 2);
        assert!((traj[0][0] - 0.0).abs() < 1e-9);
        assert!((traj.last().unwrap()[0] - 0.2).abs() < 1e-9);
        assert!(
            joint_traj_limit_violation::<OneJointRobot, 1>(
                &traj,
                OneJointRobot::CONTROL_PERIOD,
                1.0,
                false,
            )
            .is_none()
        );
    }
}
