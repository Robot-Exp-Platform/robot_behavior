use futures::future::BoxFuture;
use nalgebra as na;
use pipe_trait::*;
use serde_json::from_reader;
use std::fmt::Display;
use std::io::BufReader;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::{fs::File, time::Duration};

use crate::utils::limit::*;
use crate::{
    ControlType, Coord, JointType, LoadState, MotionType, Pose, RobotException, RobotResult,
};

#[derive(Debug, Clone)]
pub struct ArmStateSample<const N: usize> {
    pub joint: Option<[f64; N]>,
    pub joint_vel: Option<[f64; N]>,
    pub joint_acc: Option<[f64; N]>,
    pub torque: Option<[f64; N]>,
    pub pose_o_to_ee: Option<Pose>,
    pub pose_ee_to_k: Option<Pose>,
    pub cartesian_vel: Option<[f64; 6]>,
}

#[derive(Debug, Clone)]
pub struct ArmState<const N: usize> {
    pub measured: ArmStateSample<N>,
    pub commanded: ArmStateSample<N>,
    pub desired: ArmStateSample<N>,
    pub load: Option<LoadState>,
}

impl<const N: usize> ArmState<N> {
    pub fn from_measured(measured: ArmStateSample<N>, load: Option<LoadState>) -> Self {
        Self { measured, load, ..Default::default() }
    }
}

pub trait ArmDOF {
    const N: usize;
}

pub trait Arm<const N: usize> {
    fn state(&mut self) -> RobotResult<ArmState<N>>;
    fn set_load(&mut self, load: LoadState) -> RobotResult<()>;
    /// Set the coordinate system for arm
    fn set_coord(&mut self, coord: Coord) -> RobotResult<()>;
    /// Set the coordinate system for next motion command.
    fn with_coord(&mut self, coord: Coord) -> &mut Self;

    /// Set the speed ratio (0.0 ~ 1.0)
    fn set_scale(&mut self, scale: f64) -> RobotResult<()>;
    /// Get the speed ratio used by default trajectory planning.
    fn get_scale(&self) -> f64 {
        1.0
    }
    /// Set the speed ratio (0.0 ~ 1.0) for next motion command.
    fn with_scale(&mut self, scale: f64) -> &mut Self;

    /// Set the velocity for next motion command. [rad/s]()
    fn with_velocity(&mut self, joint_vel: &[f64; N]) -> &mut Self;
    /// Set the acceleration for next motion command. [rad/s^2]()
    fn with_acceleration(&mut self, joint_acc: &[f64; N]) -> &mut Self;
    /// Set the jerk for next motion command. [rad/s^3]()
    fn with_jerk(&mut self, joint_jerk: &[f64; N]) -> &mut Self;
    /// Set the cartesian velocity for next motion command. [m/s, rad/s]()
    fn with_cartesian_velocity(&mut self, cartesian_vel: f64) -> &mut Self;
    /// Set the cartesian acceleration for next motion command. [m/s^2, rad/s^2]()
    fn with_cartesian_acceleration(&mut self, cartesian_acc: f64) -> &mut Self;
    /// Set the cartesian jerk for next motion command. [m/s^3, rad/s^3]()
    fn with_cartesian_jerk(&mut self, cartesian_jerk: f64) -> &mut Self;
    /// Set the rotation velocity for next motion command. [rad/s]()
    fn with_rotation_velocity(&mut self, rotation_vel: f64) -> &mut Self;
    /// Set the rotation acceleration for next motion command. [rad/s^2]()
    fn with_rotation_acceleration(&mut self, rotation_acc: f64) -> &mut Self;
    /// Set the rotation jerk for next motion command. [rad/s^3]()
    fn with_rotation_jerk(&mut self, rotation_jerk: f64) -> &mut Self;
}

pub trait ArmParam<const N: usize> {
    const JOINT_TYPES: [JointType; N] = [JointType::Revolute; N];

    const JOINT_DEFAULT: [f64; N] = [0.; N];
    const JOINT_PACKED: [f64; N] = [0.; N];
    const JOINT_MIN: [f64; N];
    const JOINT_MAX: [f64; N];
    const JOINT_VEL_BOUND: [f64; N] = [f64::MAX; N];
    const JOINT_ACC_BOUND: [f64; N] = [f64::MAX; N];
    const JOINT_JERK_BOUND: [f64; N] = [f64::MAX; N];
    const CARTESIAN_VEL_BOUND: f64 = f64::MAX;
    const CARTESIAN_ACC_BOUND: f64 = f64::MAX;
    const CARTESIAN_JERK_BOUND: f64 = f64::MAX;
    const ROTATION_VEL_BOUND: f64 = f64::MAX;
    const ROTATION_ACC_BOUND: f64 = f64::MAX;
    const ROTATION_JERK_BOUND: f64 = f64::MAX;
    const TORQUE_BOUND: [f64; N] = [f64::MAX; N];
    const TORQUE_DOT_BOUND: [f64; N] = [f64::MAX; N];

    /// Control loop period in seconds. Used by trajectory-planning default
    /// implementations (e.g. `ArmPreplannedPath::move_waypoints`) to lay down
    /// a uniform time grid. Override this to match the actual realtime rate
    /// of the underlying driver.
    const CONTROL_PERIOD: f64;

    #[inline(always)]
    fn limit_joint_jerk(q_dddot: &mut [f64; N]) -> &mut [f64; N] {
        limit(q_dddot, &Self::JOINT_JERK_BOUND)
    }

    #[inline]
    fn limit_joint_acc<'a>(
        q_ddot: &'a mut [f64; N],
        q_ddot_last: &[f64; N],
        time: f64,
    ) -> &'a mut [f64; N] {
        q_ddot
            .pipe_mut(|q| limit_dot(q, q_ddot_last, time, &Self::JOINT_JERK_BOUND))
            .pipe_mut(|q| limit(q, &Self::JOINT_ACC_BOUND))
    }

    #[inline]
    fn limit_joint_vel<'a>(
        q_dot: &'a mut [f64; N],
        q_dot_last: &[f64; N],
        q_ddot_last: &[f64; N],
        time: f64,
    ) -> &'a mut [f64; N] {
        let mut q_ddot = difference(q_dot, q_dot_last, time);
        q_ddot.pipe_mut(|q| Self::limit_joint_acc(q, q_ddot_last, time));

        q_dot
            .pipe_mut(|q| update(q, &q_ddot, time))
            .pipe_mut(|q| limit(q, &Self::JOINT_VEL_BOUND))
    }

    #[inline]
    fn limit_joint<'a>(
        q: &'a mut [f64; N],
        q_last: &[f64; N],
        q_dot_last: &[f64; N],
        q_ddot_last: &[f64; N],
        time: f64,
    ) -> &'a mut [f64; N] {
        let mut q_dot = difference(q, q_last, time);
        q_dot.pipe_mut(|q| Self::limit_joint_vel(q, q_dot_last, q_ddot_last, time));

        q.pipe_mut(|q| update(q, &q_dot, time))
            .pipe_mut(|q| clamp(q, &Self::JOINT_MIN, &Self::JOINT_MAX))
    }

    #[inline]
    fn limit_torque<'a>(tau: &'a mut [f64; N], tau_last: &[f64; N], time: f64) -> &'a mut [f64; N] {
        tau.pipe_mut(|t| limit_dot(t, tau_last, time, &Self::TORQUE_BOUND))
            .pipe_mut(|t| limit(t, &Self::TORQUE_BOUND))
    }
}
pub trait ArmPreplannedMotion<const N: usize>: Arm<N> {
    fn move_joint(&mut self, target: &[f64; N]) -> RobotResult<()>;
    fn move_joint_async(&mut self, target: &[f64; N]) -> RobotResult<()>;

    fn move_cartesian(&mut self, target: &Pose) -> RobotResult<()>;
    fn move_cartesian_async(&mut self, target: &Pose) -> RobotResult<()>;

    fn move_to(&mut self, target: MotionType<N>) -> RobotResult<()> {
        match target {
            MotionType::Joint(target) => self.move_joint(&target),
            MotionType::Cartesian(target) => self.move_cartesian(&target),
            _ => Err(RobotException::ConflictingInstruction(
                "ArmPreplannedMotion does not support this motion type".to_string(),
            )),
        }
    }
    fn move_to_async(&mut self, target: MotionType<N>) -> RobotResult<()> {
        match target {
            MotionType::Joint(target) => self.move_joint_async(&target),
            MotionType::Cartesian(target) => self.move_cartesian_async(&target),
            _ => Err(RobotException::ConflictingInstruction(
                "ArmPreplannedMotion does not support this motion type".to_string(),
            )),
        }
    }
    fn move_rel(&mut self, target: MotionType<N>) -> RobotResult<()> {
        self.with_coord(Coord::Relative).move_to(target)
    }
    fn move_rel_async(&mut self, target: MotionType<N>) -> RobotResult<()> {
        self.with_coord(Coord::Relative).move_to_async(target)
    }
    fn move_int(&mut self, target: MotionType<N>) -> RobotResult<()> {
        self.with_coord(Coord::Inertial).move_to(target)
    }
    fn move_int_async(&mut self, target: MotionType<N>) -> RobotResult<()> {
        self.with_coord(Coord::Inertial).move_to_async(target)
    }
}

pub trait ArmPreplannedPath<const N: usize>: ArmParam<N> + Arm<N> {
    fn move_traj(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()> {
        unimplemented!("move_traj is not implemented yet {path:?}");
    }
    fn move_traj_async(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()> {
        unimplemented!("move_traj_async is not implemented yet {path:?}");
    }

    fn move_path<F>(&mut self, path: F) -> RobotResult<()>
    where
        F: Fn(f64) -> Option<MotionType<N>>,
    {
        let traj = plan_path_traj_via_copp::<Self, F, N>(&path, self.get_scale())?;
        self.move_traj(traj)
    }

    fn move_path_async<F>(&mut self, path: F) -> RobotResult<()>
    where
        F: Fn(f64) -> Option<MotionType<N>>,
    {
        let traj = plan_path_traj_via_copp::<Self, F, N>(&path, self.get_scale())?;
        self.move_traj_async(traj)
    }

    fn move_waypoints(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()> {
        match path.first() {
            Some(MotionType::Joint(_)) => {
                let waypoints: Vec<[f64; N]> = path
                    .iter()
                    .map(|m| match m {
                        MotionType::Joint(q) => *q,
                        _ => unreachable!(),
                    })
                    .collect();
                let traj = plan_waypoints_traj_via_copp::<Self, N>(&waypoints, self.get_scale())?;
                self.move_traj(traj)
            }
            _ => unimplemented!(
                "default move_waypoints only supports MotionType::Joint waypoints {path:?}"
            ),
        }
    }
    fn move_waypoints_async(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()> {
        match path.first() {
            Some(MotionType::Joint(_)) => {
                let waypoints: Vec<[f64; N]> = path
                    .iter()
                    .map(|m| match m {
                        MotionType::Joint(q) => *q,
                        _ => unreachable!(),
                    })
                    .collect();
                let traj = plan_waypoints_traj_via_copp::<Self, N>(&waypoints, self.get_scale())?;
                self.move_traj_async(traj)
            }
            _ => unimplemented!(
                "default move_waypoints_async only supports MotionType::Joint waypoints {path:?}"
            ),
        }
    }
    fn move_waypoints_prepare(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()> {
        unimplemented!("move_waypoints_prepare is not implemented yet {path:?}");
    }
    fn move_waypoints_start(&mut self, start: MotionType<N>) -> RobotResult<()> {
        unimplemented!("move_waypoints_start is not implemented yet {start:?}");
    }
}

/// Number of `s` samples used to discretise the geometric path for copp.
const COPP_PATH_SAMPLES: usize = 1001;

/// Run copp's TOPP2-RA → TOPP3-SOCP (2-iteration SCP) pipeline on a discretised
/// joint-space path and return the time-uniform `s(t)` grid (sample period is
/// `P::CONTROL_PERIOD`).
///
/// The caller supplies the path samples on a uniform `s ∈ [0, 1]` grid:
///   - `q_grid`    : positions, shape `(N, COPP_PATH_SAMPLES)`
///   - `dq_grid`   : 1st derivative w.r.t. `s`
///   - `ddq_grid`  : 2nd derivative w.r.t. `s`
///   - `dddq_grid` : optional 3rd derivative (jerk constraints inactive when `None`)
fn plan_s_t_via_copp<R, const N: usize>(
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
    // The reach-set can return tiny negative values from numerical noise; the
    // downstream linearization rejects any negative entry.
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

    // SCP: linearise once at TOPP2-RA seed, then refine at the QP solution.
    let (a_qp1, _b_qp1, _ns1) = {
        let prob =
            Topp3ProblemBuilder::new(&mut robot, idx_s_interval.0, &a_ra0, (0.0, 0.0), (0.0, 0.0))
                .build_with_linearization()
                .map_err(map_err)?;
        topp3_socp(&prob, &opts_socp).map_err(map_err)?
    };
    // Clarabel under `allow_almost_solved` may return tiny negative `a` values
    // (~1e-12) at near-stationary samples; the next linearization rejects any
    // negative entry, so clamp the seed before feeding it back.
    let a_qp1: Vec<f64> = a_qp1.into_iter().map(|a| a.max(0.0)).collect();
    // Second SCP refinement is best-effort: on hard paths Clarabel may report
    // InsufficientProgress, in which case `(a_qp1, b_qp1)` is already a
    // feasible TOPP3 profile and we use it as-is.
    let (a_qp2, b_qp2, ns2) = {
        let prob =
            Topp3ProblemBuilder::new(&mut robot, idx_s_interval.0, &a_qp1, (0.0, 0.0), (0.0, 0.0))
                .build_with_linearization()
                .map_err(map_err)?;
        match topp3_socp(&prob, &opts_socp) {
            Ok(res) => res,
            Err(_) => {
                // Re-run iteration 1 to recover (a, b, num_stationary); the
                // first solve only kept `a_qp1`. Cheaper than restructuring.
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
///
/// The geometry is reconstructed by `Path::from_waypoints` (Hermite spline);
/// both the constraint discretisation **and** the final `q(t)` come from the
/// same spline — there is no second analytic ground truth to fall back on.
fn plan_waypoints_traj_via_copp<R, const N: usize>(
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

    // Final q(t) comes from the same spline used for planning — no extra
    // approximation source available.
    let q_t = path.evaluate_q(&s_t).map_err(|e| {
        RobotException::UnprocessableInstructionError(format!("path.evaluate_q: {e}"))
    })?;
    Ok(q_grid_to_traj::<N>(&q_t.q))
}

/// Plan a time-uniform joint trajectory along a continuous user-supplied path
/// `s -> Option<MotionType::Joint>` via copp.
///
/// Why two helpers? The geometric path here is *already* given as an analytic
/// closure — only its discrete derivatives are missing. So we:
///   1. sample `q(s)` from the user closure on a fine `s` grid;
///   2. fit a Hermite spline ONLY to estimate `dq/ddq/dddq` for copp's
///      constraint discretisation (this is the unavoidable approximation);
///   3. solve TOPP3-SOCP to get the time-uniform `s(t)` grid;
///   4. evaluate the **user closure** (not the spline) at each `s(t)` to
///      produce the final `q(t)` — preserving full tracking fidelity for any
///      shape the user can express analytically.
fn plan_path_traj_via_copp<P, F, const N: usize>(
    path_fn: &F,
    scale: f64,
) -> RobotResult<Vec<MotionType<N>>>
where
    P: ArmParam<N> + ?Sized,
    F: Fn(f64) -> Option<MotionType<N>>,
{
    use copp::path::{Path, SplineConfig};

    // 1) Sample the user closure on the fine s grid that copp will discretise on.
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

    // 2) Fit a spline only to recover dq/ddq/dddq for copp constraints.
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

    // 3) Solve TOPP3-SOCP → time-uniform s(t).
    let s_t = plan_s_t_via_copp::<P, N>(
        &derivs.q,
        derivs.dq.as_ref().unwrap(),
        derivs.ddq.as_ref().unwrap(),
        derivs.dddq.as_ref(),
        scale,
    )?;

    // 4) Final q(t) comes from the user closure directly — no spline loss.
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
fn q_grid_to_traj<const N: usize>(q: &na::DMatrix<f64>) -> Vec<MotionType<N>> {
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

pub trait ArmPreplannedMotionExt<const N: usize>:
    ArmPreplannedMotion<N> + ArmPreplannedPath<N>
{
    fn move_joint_rel(&mut self, target: &[f64; N]) -> RobotResult<()> {
        self.with_coord(Coord::Relative).move_joint(target)
    }
    fn move_joint_rel_async(&mut self, target: &[f64; N]) -> RobotResult<()> {
        self.with_coord(Coord::Relative).move_joint_async(target)
    }
    fn move_joint_traj(&mut self, path: Vec<[f64; N]>) -> RobotResult<()> {
        self.with_coord(Coord::Relative)
            .move_traj(path.into_iter().map(MotionType::Joint).collect())
    }

    fn move_cartesian_rel(&mut self, target: &Pose) -> RobotResult<()> {
        self.with_coord(Coord::Relative).move_cartesian(target)
    }
    fn move_cartesian_rel_async(&mut self, target: &Pose) -> RobotResult<()> {
        self.with_coord(Coord::Relative)
            .move_cartesian_async(target)
    }
    fn move_cartesian_int(&mut self, target: &Pose) -> RobotResult<()> {
        self.with_coord(Coord::Inertial).move_cartesian(target)
    }
    fn move_cartesian_int_async(&mut self, target: &Pose) -> RobotResult<()> {
        self.with_coord(Coord::Inertial)
            .move_cartesian_async(target)
    }
    fn move_cartesian_traj(&mut self, path: Vec<Pose>) -> RobotResult<()> {
        self.with_coord(Coord::Relative)
            .move_traj(path.into_iter().map(MotionType::Cartesian).collect())
    }

    fn move_linear_with_euler(&mut self, pose: [f64; 6]) -> RobotResult<()> {
        self.move_cartesian(&pose.into())
    }
    fn move_linear_with_euler_async(&mut self, pose: [f64; 6]) -> RobotResult<()> {
        self.move_cartesian_async(&pose.into())
    }
    fn move_linear_with_euler_rel(&mut self, pose: [f64; 6]) -> RobotResult<()> {
        self.with_coord(Coord::Relative)
            .move_cartesian(&pose.into())
    }
    fn move_linear_with_euler_rel_async(&mut self, pose: [f64; 6]) -> RobotResult<()> {
        self.with_coord(Coord::Relative)
            .move_cartesian_async(&pose.into())
    }
    fn move_linear_with_euler_int(&mut self, pose: [f64; 6]) -> RobotResult<()> {
        self.with_coord(Coord::Inertial)
            .move_cartesian(&pose.into())
    }
    fn move_linear_with_euler_int_async(&mut self, pose: [f64; 6]) -> RobotResult<()> {
        self.with_coord(Coord::Inertial)
            .move_cartesian_async(&pose.into())
    }

    fn move_linear_with_quat(&mut self, target: &na::Isometry3<f64>) -> RobotResult<()> {
        self.move_cartesian(&Pose::Quat(*target))
    }
    fn move_linear_with_quat_async(&mut self, target: &na::Isometry3<f64>) -> RobotResult<()> {
        self.move_cartesian_async(&Pose::Quat(*target))
    }
    fn move_linear_with_quat_rel(&mut self, target: &na::Isometry3<f64>) -> RobotResult<()> {
        self.with_coord(Coord::Relative)
            .move_cartesian(&Pose::Quat(*target))
    }
    fn move_linear_with_quat_rel_async(&mut self, target: &na::Isometry3<f64>) -> RobotResult<()> {
        self.with_coord(Coord::Relative)
            .move_cartesian_async(&Pose::Quat(*target))
    }
    fn move_linear_with_quat_int(&mut self, target: &na::Isometry3<f64>) -> RobotResult<()> {
        self.with_coord(Coord::Inertial)
            .move_cartesian(&Pose::Quat(*target))
    }
    fn move_linear_with_quat_int_async(&mut self, target: &na::Isometry3<f64>) -> RobotResult<()> {
        self.with_coord(Coord::Inertial)
            .move_cartesian_async(&Pose::Quat(*target))
    }

    fn move_linear_with_homo(&mut self, target: [f64; 16]) -> RobotResult<()> {
        self.move_cartesian(&target.into())
    }
    fn move_linear_with_homo_async(&mut self, target: [f64; 16]) -> RobotResult<()> {
        self.move_cartesian_async(&target.into())
    }
    fn move_linear_with_homo_rel(&mut self, target: [f64; 16]) -> RobotResult<()> {
        self.with_coord(Coord::Relative)
            .move_cartesian(&target.into())
    }
    fn move_linear_with_homo_rel_async(&mut self, target: [f64; 16]) -> RobotResult<()> {
        self.with_coord(Coord::Relative)
            .move_cartesian_async(&target.into())
    }
    fn move_linear_with_homo_int(&mut self, target: [f64; 16]) -> RobotResult<()> {
        self.with_coord(Coord::Inertial)
            .move_cartesian(&target.into())
    }
    fn move_linear_with_homo_int_async(&mut self, target: [f64; 16]) -> RobotResult<()> {
        self.with_coord(Coord::Inertial)
            .move_cartesian_async(&target.into())
    }

    fn move_waypoints_prepare_from_file(&mut self, path: &str) -> RobotResult<()> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);
        let path: Vec<MotionType<N>> = from_reader(reader).unwrap();
        self.move_waypoints_prepare(path)
    }
    fn move_traj_from_file(&mut self, path: &str) -> RobotResult<()> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);
        let path: Vec<MotionType<N>> = from_reader(reader).unwrap();
        self.move_traj(path)
    }
    fn move_waypoints_from_file(&mut self, path: &str) -> RobotResult<()> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);
        let path: Vec<MotionType<N>> = from_reader(reader).unwrap();
        self.move_waypoints(path)
    }
}

impl<const N: usize, T> ArmPreplannedMotionExt<N> for T where
    T: ArmPreplannedMotion<N> + ArmPreplannedPath<N>
{
}

#[derive(Clone, Debug)]
pub struct JointImpedanceHandle<const N: usize> {
    pub stiffness: Arc<Mutex<[f64; N]>>,
    pub damping: Arc<Mutex<[f64; N]>>,
    pub target: Arc<Mutex<Option<[f64; N]>>>,
    pub is_finished: Arc<AtomicBool>,
}

pub trait JointImpedance<const N: usize> {
    fn set_stiffness(&self, stiffness: [f64; N]);
    fn set_damping(&self, damping: [f64; N]);
    fn set_target(&self, target: Option<[f64; N]>);
    fn finish(&self);
}

impl<const N: usize> JointImpedance<N> for JointImpedanceHandle<N> {
    fn set_stiffness(&self, stiffness: [f64; N]) {
        let mut s = self.stiffness.lock().unwrap();
        *s = stiffness;
    }
    fn set_damping(&self, damping: [f64; N]) {
        let mut d = self.damping.lock().unwrap();
        *d = damping;
    }
    fn set_target(&self, target: Option<[f64; N]>) {
        let mut t = self.target.lock().unwrap();
        *t = target;
    }
    fn finish(&self) {
        self.is_finished
            .store(true, std::sync::atomic::Ordering::SeqCst);
    }
}

#[derive(Clone, Debug)]
pub struct CartesianImpedanceHandle {
    pub stiffness: Arc<Mutex<(f64, f64)>>,
    pub damping: Arc<Mutex<(f64, f64)>>,
    pub target: Arc<Mutex<Option<Pose>>>,
    pub is_finished: Arc<AtomicBool>,
}

pub trait CartesianImpedance {
    fn set_stiffness(&self, stiffness: (f64, f64));
    fn set_damping(&self, damping: (f64, f64));
    fn set_target(&self, target: Option<Pose>);
    fn finish(&self);
}

impl CartesianImpedance for CartesianImpedanceHandle {
    fn set_stiffness(&self, stiffness: (f64, f64)) {
        let mut s = self.stiffness.lock().unwrap();
        *s = stiffness;
    }
    fn set_damping(&self, damping: (f64, f64)) {
        let mut d = self.damping.lock().unwrap();
        *d = damping;
    }
    fn set_target(&self, target: Option<Pose>) {
        let mut t = self.target.lock().unwrap();
        *t = target;
    }
    fn finish(&self) {
        self.is_finished
            .store(true, std::sync::atomic::Ordering::SeqCst);
    }
}

pub trait ArmImpedance<const N: usize> {
    fn joint_impedance_async(
        &mut self,
        stiffness: &[f64; N],
        damping: &[f64; N],
    ) -> RobotResult<JointImpedanceHandle<N>>;
    fn cartesian_impedance_async(
        &mut self,
        stiffness: (f64, f64),
        damping: (f64, f64),
    ) -> RobotResult<CartesianImpedanceHandle>;

    fn joint_impedance_control(
        &mut self,
        stiffness: &[f64; N],
        damping: &[f64; N],
    ) -> RobotResult<(
        JointImpedanceHandle<N>,
        impl FnMut() -> BoxFuture<'static, RobotResult<()>> + Send + 'static,
    )>;

    fn cartesian_impedance_control(
        &mut self,
        stiffness: (f64, f64),
        damping: (f64, f64),
    ) -> RobotResult<(
        CartesianImpedanceHandle,
        impl FnMut() -> BoxFuture<'static, RobotResult<()>> + Send + 'static,
    )>;
}

pub trait ArmStreamingHandle<const N: usize> {
    fn last_motion(&self) -> Option<MotionType<N>>;
    fn move_to(&mut self, target: MotionType<N>) -> RobotResult<()>;

    fn last_control(&self) -> Option<ControlType<N>>;
    fn control_with(&mut self, control: ControlType<N>) -> RobotResult<()>;
}

pub trait ArmStreamingMotion<const N: usize>: Arm<N> {
    type Handle: ArmStreamingHandle<N>;
    fn start_streaming(&mut self) -> RobotResult<Self::Handle>;
    fn end_streaming(&mut self) -> RobotResult<()>;

    fn move_to_target(&mut self) -> Arc<Mutex<Option<MotionType<N>>>>;
    fn control_with_target(&mut self) -> Arc<Mutex<Option<ControlType<N>>>>;
}

pub trait ArmStreamingMotionExt<const N: usize>: ArmStreamingMotion<N> {
    fn move_joint_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
    fn move_joint_vel_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
    fn move_joint_acc_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
    fn move_cartesian_target(&mut self) -> Arc<Mutex<Option<Pose>>>;
    fn move_cartesian_vel_target(&mut self) -> Arc<Mutex<Option<[f64; 6]>>>;
    fn move_cartesian_euler_target(&mut self) -> Arc<Mutex<Option<[f64; 6]>>>;
    fn move_cartesian_quat_target(&mut self) -> Arc<Mutex<Option<na::Isometry3<f64>>>>;
    fn move_cartesian_homo_target(&mut self) -> Arc<Mutex<Option<[f64; 16]>>>;
    fn control_tau_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>>;
}

pub trait ArmRealtimeControl<const N: usize>: Arm<N> {
    fn move_with_closure<FM>(&mut self, closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, Duration) -> (MotionType<N>, bool) + Send + 'static;
    fn control_with_closure<FC>(&mut self, closure: FC) -> RobotResult<()>
    where
        FC: FnMut(ArmState<N>, Duration) -> (ControlType<N>, bool) + Send + 'static;
}

pub trait ArmRealtimeControlExt<const N: usize>: ArmRealtimeControl<N> {
    fn move_joint_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, Duration) -> ([f64; N], bool) + Send + Sync + 'static,
    {
        self.move_with_closure(move |state, duration| {
            let (joint, finished) = closure(state, duration);
            (MotionType::Joint(joint), finished)
        })
    }

    fn move_joint_vel_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, Duration) -> ([f64; N], bool) + Send + Sync + 'static,
    {
        self.move_with_closure(move |state, duration| {
            let (joint_vel, finished) = closure(state, duration);
            (MotionType::JointVel(joint_vel), finished)
        })
    }

    fn move_cartesian_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, Duration) -> (Pose, bool) + Send + Sync + 'static,
    {
        self.move_with_closure(move |state, duration| {
            let (pose, finished) = closure(state, duration);
            (MotionType::Cartesian(pose), finished)
        })
    }

    fn move_cartesian_vel_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, Duration) -> ([f64; 6], bool) + Send + Sync + 'static,
    {
        self.move_with_closure(move |state, duration| {
            let (vel, finished) = closure(state, duration);
            (MotionType::CartesianVel(vel), finished)
        })
    }

    fn move_cartesian_euler_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, Duration) -> ([f64; 3], [f64; 3], bool) + Send + Sync + 'static,
    {
        self.move_cartesian_with_closure(move |state, duration| {
            let (tran, rot, finished) = closure(state, duration);
            (Pose::Euler(tran, rot), finished)
        })
    }

    fn move_cartesian_quat_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, Duration) -> (na::Isometry3<f64>, bool) + Send + Sync + 'static,
    {
        self.move_cartesian_with_closure(move |state, duration| {
            let (quat, finished) = closure(state, duration);
            (Pose::Quat(quat), finished)
        })
    }

    fn move_cartesian_homo_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, Duration) -> ([f64; 16], bool) + Send + Sync + 'static,
    {
        self.move_cartesian_with_closure(move |state, duration| {
            let (homo, finished) = closure(state, duration);
            (Pose::Homo(homo), finished)
        })
    }
}

// pub struct ArmControlRhythm<I, O>;

impl<const N: usize> Default for ArmStateSample<N> {
    fn default() -> Self {
        ArmStateSample {
            joint: Some([0.; N]),
            joint_vel: Some([0.; N]),
            joint_acc: Some([0.; N]),
            torque: Some([0.; N]),
            pose_o_to_ee: Some(Pose::default()),
            pose_ee_to_k: Some(Pose::default()),
            cartesian_vel: Some([0.; 6]),
        }
    }
}

impl<const N: usize> Default for ArmState<N> {
    fn default() -> Self {
        ArmState {
            measured: ArmStateSample::default(),
            commanded: ArmStateSample::default(),
            desired: ArmStateSample::default(),
            load: Some(LoadState { m: 0., x: [0.; 3], i: [0.; 9] }),
        }
    }
}

impl<const N: usize> PartialEq<MotionType<N>> for ArmState<N> {
    fn eq(&self, other: &MotionType<N>) -> bool {
        if let (MotionType::Joint(joint_target), Some(joint_state)) = (other, self.measured.joint) {
            return joint_state == *joint_target;
        }
        if let (MotionType::JointVel(vel_target), Some(vel_state)) =
            (other, self.measured.joint_vel)
        {
            return vel_state == *vel_target;
        }
        if let (MotionType::Cartesian(pose_target), Some(pose_state)) =
            (other, self.measured.pose_o_to_ee)
        {
            return pose_state == *pose_target;
        }
        if let (MotionType::CartesianVel(vel_target), Some(vel_state)) =
            (other, self.measured.cartesian_vel)
        {
            return vel_state == *vel_target;
        }
        false
    }
}

impl<const N: usize> PartialEq<ControlType<N>> for ArmState<N> {
    fn eq(&self, other: &ControlType<N>) -> bool {
        if let (ControlType::Torque(torque_target), Some(force_state)) =
            (other, self.measured.torque)
        {
            return force_state == *torque_target;
        }
        false
    }
}

impl<const N: usize> Display for ArmState<N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            r#"arm_state:
    | q: {:?},
    | dq: {:?},
    | ddq: {:?},
    | torque: {:?},
    | pose_o_to_ee: {:?},"#,
            self.measured.joint,
            self.measured.joint_vel,
            self.measured.joint_acc,
            self.measured.torque,
            self.measured.pose_o_to_ee
        )
    }
}

#[cfg(feature = "to_py")]
mod to_py {
    use crate::PyPose;

    use super::*;
    use pyo3::{pyclass, pymethods};

    #[derive(Debug, Clone)]
    #[pyclass(name = "ArmState")]
    pub struct PyArmState {
        #[pyo3(get, set)]
        pub joint: Option<Vec<f64>>,
        #[pyo3(get, set)]
        pub joint_vel: Option<Vec<f64>>,
        #[pyo3(get, set)]
        pub joint_acc: Option<Vec<f64>>,
        #[pyo3(get, set)]
        pub torque: Option<Vec<f64>>,
        #[pyo3(get, set)]
        pub pose_o_to_ee: Option<PyPose>,
        #[pyo3(get, set)]
        pub pose_ee_to_k: Option<PyPose>,
        #[pyo3(get, set)]
        pub cartesian_vel: Option<[f64; 6]>,
        #[pyo3(get, set)]
        pub load: Option<LoadState>,
    }

    impl<const N: usize> From<ArmState<N>> for PyArmState {
        fn from(state: ArmState<N>) -> Self {
            PyArmState {
                joint: state.measured.joint.map(|j| j.to_vec()),
                joint_vel: state.measured.joint_vel.map(|j| j.to_vec()),
                joint_acc: state.measured.joint_acc.map(|j| j.to_vec()),
                torque: state.measured.torque.map(|t| t.to_vec()),
                pose_o_to_ee: state.measured.pose_o_to_ee.map(Into::into),
                pose_ee_to_k: state.measured.pose_ee_to_k.map(Into::into),
                cartesian_vel: state.measured.cartesian_vel,
                load: state.load,
            }
        }
    }

    impl<const N: usize> From<PyArmState> for ArmState<N> {
        fn from(value: PyArmState) -> Self {
            ArmState {
                measured: ArmStateSample {
                    joint: value.joint.map(|j| j.try_into().unwrap_or([0.; N])),
                    joint_vel: value.joint_vel.map(|j| j.try_into().unwrap_or([0.; N])),
                    joint_acc: value.joint_acc.map(|j| j.try_into().unwrap_or([0.; N])),
                    torque: value.torque.map(|t| t.try_into().unwrap_or([0.; N])),
                    pose_o_to_ee: value.pose_o_to_ee.map(Into::into),
                    pose_ee_to_k: value.pose_ee_to_k.map(Into::into),
                    cartesian_vel: value.cartesian_vel,
                },
                load: value.load,
                ..Default::default()
            }
        }
    }

    #[pymethods]
    impl PyArmState {
        fn __repr__(&self) -> String {
            format!(
                r#"ArmState:
    |> joint: {:?},
    |> joint_vel: {:?},
    |> joint_acc: {:?},
    |> torque: {:?},
    |> pose_o_to_ee: {:?},
    |> pose_ee_to_k: {:?},
    |> cartesian_vel: {:?},
    |> load: {:?}"#,
                self.joint,
                self.joint_vel,
                self.joint_acc,
                self.torque,
                self.pose_o_to_ee,
                self.pose_ee_to_k,
                self.cartesian_vel,
                self.load
            )
        }
    }
}

#[cfg(feature = "to_py")]
pub use to_py::*;

#[cfg(feature = "to_cxx")]
mod to_cxx {
    use crate::ArmState;

    pub struct CxxArmState {
        pub joint: Option<Vec<f64>>,
        pub joint_vel: Option<Vec<f64>>,
        pub joint_acc: Option<Vec<f64>>,
        pub tau: Option<Vec<f64>>,
        pub pose_o_to_ee: Option<crate::Pose>,
        pub pose_ee_to_k: Option<crate::Pose>,
        pub cartesian_vel: Option<Vec<f64>>,
        pub load: Option<crate::LoadState>,
    }

    impl<const N: usize> From<ArmState<N>> for CxxArmState {
        fn from(value: ArmState<N>) -> Self {
            CxxArmState {
                joint: value.measured.joint.map(|j| j.to_vec()),
                joint_vel: value.measured.joint_vel.map(|j| j.to_vec()),
                joint_acc: value.measured.joint_acc.map(|j| j.to_vec()),
                tau: value.measured.torque.map(|t| t.to_vec()),
                pose_o_to_ee: value.measured.pose_o_to_ee,
                pose_ee_to_k: value.measured.pose_ee_to_k,
                cartesian_vel: value.measured.cartesian_vel.map(|c| c.to_vec()),
                load: value.load,
            }
        }
    }

    #[cxx::bridge]
    mod cxx_bridge {
        extern "Rust" {
            type CxxArmState;
        }
    }
}

#[cfg(feature = "to_cxx")]
pub use to_cxx::*;
