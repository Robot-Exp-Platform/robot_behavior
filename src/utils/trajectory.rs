use nalgebra as na;
use std::fmt::Display;

use crate::{ArmParam, MotionType, RobotException, RobotResult};

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
    R: ArmParam<N> + ?Sized,
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
            "ArmParam::CONTROL_PERIOD must be a positive finite number, got {dt}"
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

    let (a_qp1, _b_qp1, _ns1) = {
        let prob =
            Topp3ProblemBuilder::new(&mut robot, idx_s_interval.0, &a_ra0, (0.0, 0.0), (0.0, 0.0))
                .build_with_linearization()
                .map_err(map_err)?;
        topp3_socp(&prob, &opts_socp).map_err(map_err)?
    };
    let a_qp1: Vec<f64> = a_qp1.into_iter().map(|a| a.max(0.0)).collect();
    let (a_qp2, b_qp2, ns2) = {
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

    let (_t_final, t_s) = s_to_t_topp3(&s, &a_qp2, &b_qp2, ns2, 0.0);
    let s_t = t_to_s_topp3(
        &s,
        &a_qp2,
        &b_qp2,
        ns2,
        &t_s,
        InterpolationMode::UniformTimeGrid(0.0, dt, true),
    );
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
) -> RobotResult<Vec<MotionType<N>>>
where
    R: ArmParam<N> + ?Sized,
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
) -> RobotResult<Vec<MotionType<N>>>
where
    P: ArmParam<N> + ?Sized,
    F: Fn(f64) -> Option<MotionType<N>>,
{
    use copp::path::{Path, SplineConfig};

    let n = COPP_PATH_SAMPLES;
    let mut q_samples: Vec<[f64; N]> = Vec::with_capacity(n);
    for i in 0..n {
        let s = i as f64 / (n - 1) as f64;
        match path_fn(s) {
            Some(MotionType::Joint(q)) => q_samples.push(q),
            Some(other) => {
                return Err(RobotException::ConflictingInstruction(format!(
                    "default move_path only supports MotionType::Joint, got {other:?}"
                )));
            }
            None => {
                return Err(RobotException::UnprocessableInstructionError(format!(
                    "move_path closure returned None at s={}; must be defined on [0, 1]",
                    s
                )));
            }
        }
    }

    let wp_mat = na::DMatrix::<f64>::from_fn(N, n, |i, j| q_samples[j][i]);
    let spline = Path::from_waypoints(&wp_mat, SplineConfig::default()).map_err(|e| {
        RobotException::UnprocessableInstructionError(format!(
            "copp Path::from_waypoints (closure derivative estimate) failed: {e}"
        ))
    })?;
    let s_grid: Vec<f64> = (0..n).map(|j| j as f64 / (n - 1) as f64).collect();
    let derivs = spline.evaluate_up_to_3rd(&s_grid).map_err(|e| {
        RobotException::UnprocessableInstructionError(format!("path.evaluate_up_to_3rd: {e}"))
    })?;

    let s_t = plan_s_t_via_copp::<P, N>(
        &derivs.q,
        derivs.dq.as_ref().unwrap(),
        derivs.ddq.as_ref().unwrap(),
        derivs.dddq.as_ref(),
        scale,
    )?;

    let mut traj = Vec::with_capacity(s_t.len());
    for &s in &s_t {
        match path_fn(s) {
            Some(MotionType::Joint(q)) => traj.push(MotionType::Joint(q)),
            Some(other) => {
                return Err(RobotException::ConflictingInstruction(format!(
                    "default move_path only supports MotionType::Joint, got {other:?}"
                )));
            }
            None => {
                return Err(RobotException::UnprocessableInstructionError(format!(
                    "move_path closure returned None at s={s} during final resampling"
                )));
            }
        }
    }
    Ok(traj)
}

/// Convert a column-major `(N, M)` joint matrix to `Vec<MotionType::Joint>`.
pub fn q_grid_to_traj<const N: usize>(q: &na::DMatrix<f64>) -> Vec<MotionType<N>> {
    let mut traj = Vec::with_capacity(q.ncols());
    for j in 0..q.ncols() {
        let mut joint = [0.0f64; N];
        for (i, slot) in joint.iter_mut().enumerate().take(N) {
            *slot = q[(i, j)];
        }
        traj.push(MotionType::Joint(joint));
    }
    traj
}
