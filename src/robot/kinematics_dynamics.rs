use nalgebra::{self as na, SVector};

use crate::robot::{dh::DhParam, joint::Joints, types::Pose};

/// A rigid-body transform in `SE(3)`.
pub type Iso3 = na::Isometry3<f64>; // SE(3)
/// A spatial velocity `[vx, vy, vz, wx, wy, wz]` (linear then angular).
pub type Twist = na::SVector<f64, 6>; // [vx, vy, vz, wx, wy, wz]
/// A spatial force `[fx, fy, fz, tx, ty, tz]` (force then torque).
pub type Wrench = na::SVector<f64, 6>;
/// A joint-space column vector of size `N`.
pub type JVec<const N: usize> = na::SVector<f64, N>;
/// A square `N x N` joint-space matrix (e.g. mass / inertia).
pub type JMat<const N: usize> = na::SMatrix<f64, N, N>;
/// A `6 x N` geometric Jacobian mapping joint rates to an end-effector twist.
pub type Jaco<const N: usize> = na::SMatrix<f64, 6, N>;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
/// A link index in `0..=N`, where `0` is the base and `N` is the end effector.
pub struct Link(pub usize);
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
/// A joint index in `1..=N`.
pub struct Joint(pub usize);

#[derive(Clone, Copy, Debug)]
/// The kinematic type of a joint.
pub enum JointType {
    /// Rotational joint (rotates about its axis).
    Revolute,
    /// Linear joint (translates along its axis).
    Prismatic,
}

/// Precomputed forward-kinematics quantities for one joint configuration.
///
/// Built once by [`build`](ArmKineCache::build) from the arm's DH parameters
/// and a joint vector, it caches every link transform so that link poses,
/// relative poses, the geometric [`jacobian`](ArmKineCache::jacobian) and the
/// end-effector [`twist`](ArmKineCache::ee_twist) can be queried without
/// recomputation. Requires `[(); N + 1]:` because it stores `N + 1` frames
/// (base + one per joint).
pub struct ArmKineCache<const N: usize>
where
    [(); N + 1]:,
{
    /// The joint velocities the cache was built with (rad/s).
    pub q_dot: [f64; N],
    /// Cumulative base-to-link transforms, index `0..=N` (`[0]` = identity base).
    pub t_prefix: [na::Isometry3<f64>; N + 1],
    /// World-frame origin of each link frame.
    pub origins: [na::Vector3<f64>; N + 1],
    /// World-frame `z` axis of each link frame.
    pub z_axes: [na::Vector3<f64>; N + 1],
    /// Per-joint revolute axis in the WORLD frame, evaluated at the
    /// pre-actuation point (between the constant link transform and the
    /// `Rz(θ+q)` actuation). Independent of DH/MDH convention.
    pub joint_z: [na::Vector3<f64>; N],
    /// Per-joint axis origin in the WORLD frame.
    pub joint_p: [na::Vector3<f64>; N],
}

impl<const N: usize> ArmKineCache<N>
where
    [(); N + 1]:,
{
    #[inline(always)]
    /// Build the cache from DH parameters, joint positions `q` and joint
    /// velocities `q_dot`.
    pub fn build(dh: &[DhParam; N], q: &[f64; N], q_dot: &[f64; N]) -> Self {
        let mut t_prefix = [na::Isometry3::identity(); N + 1];
        let mut origins = [na::Vector3::zeros(); N + 1];
        let mut z_axes = [na::Vector3::z(); N + 1];
        let mut joint_z = [na::Vector3::z(); N];
        let mut joint_p = [na::Vector3::zeros(); N];

        for i in 0..N {
            let t_pre = t_prefix[i] * dh[i].pre_actuation();
            joint_z[i] = t_pre.rotation * na::Vector3::z();
            joint_p[i] = t_pre.translation.vector;
            t_prefix[i + 1] = t_prefix[i] * dh[i].to_se3(q[i]);
            origins[i + 1] = t_prefix[i + 1].translation.vector;
            z_axes[i + 1] = t_prefix[i + 1].rotation * na::Vector3::z();
        }

        Self { q_dot: *q_dot, t_prefix, origins, z_axes, joint_z, joint_p }
    }

    #[inline(always)]
    /// The end-effector pose (transform of link `N`) in the base frame.
    pub fn end_effector_pose(&self) -> na::Isometry3<f64> {
        self.t_prefix[N]
    }

    #[inline(always)]
    /// The pose of a given [`Link`] in the base frame.
    pub fn link_pose(&self, link: Link) -> na::Isometry3<f64> {
        assert!(link.0 <= N, "link index must be less than or equal to N");
        self.t_prefix[link.0]
    }

    #[inline(always)]
    /// The transform of link `a` expressed in link `b`'s frame.
    pub fn relative_pose(&self, a: Link, b: Link) -> na::Isometry3<f64> {
        assert!(a.0 <= N && b.0 <= N, "Link index out of bounds");
        self.t_prefix[b.0].inv_mul(&self.t_prefix[a.0])
    }

    #[inline(always)]
    /// The base-frame poses of several links at once.
    pub fn link_poses(&self, indices: &[Link]) -> Vec<na::Isometry3<f64>> {
        indices.iter().map(|&i| self.link_pose(i)).collect()
    }

    #[inline(always)]
    /// The `6 x N` geometric Jacobian at the end effector.
    pub fn jacobian(&self) -> na::SMatrix<f64, 6, N> {
        let mut j = na::SMatrix::<f64, 6, N>::zeros();
        let p_n = self.origins[N];
        for i in 0..N {
            let z = self.joint_z[i];
            let p = self.joint_p[i];
            let jv = z.cross(&(p_n - p));
            j.fixed_view_mut::<3, 1>(0, i).copy_from(&jv);
            j.fixed_view_mut::<3, 1>(3, i).copy_from(&z);
        }
        j
    }

    #[inline(always)]
    /// The `6 x N` geometric Jacobian evaluated at an intermediate link
    /// (`link_index` in `0..=N`); columns for joints beyond it are zero.
    pub fn jacobian_at_link(&self, link_index: usize) -> na::SMatrix<f64, 6, N> {
        assert!(
            link_index <= N,
            "link_index must be less than or equal to N"
        );
        let mut j = na::SMatrix::<f64, 6, N>::zeros();
        let p_n = self.origins[link_index];
        for i in 0..link_index {
            let z = self.joint_z[i];
            let p = self.joint_p[i];
            let jv = z.cross(&(p_n - p));
            j.fixed_view_mut::<3, 1>(0, i).copy_from(&jv);
            j.fixed_view_mut::<3, 1>(3, i).copy_from(&z);
        }
        j
    }

    #[inline(always)]
    /// The end-effector [`Twist`] produced by the cached joint velocities.
    pub fn ee_twist(&self) -> Twist {
        self.jacobian() * SVector::<f64, N>::from_column_slice(&self.q_dot)
    }
}

/// Forward kinematics for an `N`-DoF arm with a fixed DH model.
///
/// An implementor declares its [`DH`](ArmForwardKinematics::DH) chain once; the
/// provided methods then build an [`ArmKineCache`] and compute the
/// end-effector [`Pose`]. This is the FK half of the kinematics API and the
/// super-trait of [`ArmInverseKinematics`].
pub trait ArmForwardKinematics<const N: usize>
where
    [(); N + 1]:,
{
    /// The arm's Denavit–Hartenberg parameters, one [`DhParam`] per joint.
    const DH: [DhParam; N];

    #[inline(always)]
    /// Build a full [`ArmKineCache`] for the given joint positions/velocities.
    fn kine_cache(q: &[f64; N], q_dot: &[f64; N]) -> ArmKineCache<N> {
        ArmKineCache::build(&Self::DH, q, q_dot)
    }

    #[inline(always)]
    /// Compute the end-effector [`Pose`] for joint positions `q`.
    fn fk_end_pose(q: &[f64; N]) -> Pose {
        let k = Self::kine_cache(q, &[0.0; N]);
        Pose::Quat(k.end_effector_pose())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// A recognised closed-form (analytic) inverse-kinematics family.
///
/// Used by [`ArmInverseKinematics::ANALYTIC_FAMILY`] to decide whether an
/// analytic solver is available before falling back to an iterative method.
pub enum AnalyticFamily {
    /// Planar 2-revolute arm.
    Planar2R,
    /// Planar 3-revolute arm.
    Planar3R,
    /// SCARA with an RRPR structure.
    ScaraRRPR,
    /// Spherical-wrist 6-revolute arm.
    Spherical6R,
    /// Generic SCARA.
    Scara,
    /// A user-provided custom analytic solver.
    Custom,
}

#[derive(Clone, Copy, Debug)]
/// Convergence and step-limiting criteria shared by the iterative IK methods.
pub struct CommonStop {
    /// Position error tolerance (m) for convergence.
    pub pos_tol: f64,
    /// Orientation error tolerance (rad) for convergence.
    pub rot_tol: f64,
    /// Maximum number of iterations before giving up.
    pub max_iters: usize,
    /// Per-step clamp on each `|Δq_i|` (rad).
    pub step_clip: f64,
}
impl Default for CommonStop {
    fn default() -> Self {
        Self { pos_tol: 1e-4, rot_tol: 1e-4, max_iters: 200, step_clip: 0.2 }
    }
}

#[derive(Clone, Debug)]
/// The inverse-kinematics solving strategy to use.
pub enum IKMethod {
    /// Try the analytic solver if [`ArmInverseKinematics::ANALYTIC_FAMILY`] is
    /// set, otherwise recurse into `fallback` (e.g. [`IKMethod::DLS`]).
    Analytic { fallback: Box<IKMethod> },
    /// Damped Least Squares (Levenberg–Marquardt style); robust near singularities.
    DLS { lambda: f64, stop: CommonStop },
    /// Jacobian transpose; simple and robust but slow to converge.
    JT { gain: f64, stop: CommonStop },
    /// Newton–Raphson via (pseudo)inverse; fast on well-conditioned problems.
    Newton { stop: CommonStop },
    /// Levenberg–Marquardt on the geometric residual with adaptive damping.
    LM { lambda0: f64, stop: CommonStop },
}

impl Default for IKMethod {
    fn default() -> Self {
        IKMethod::DLS { lambda: 1e-3, stop: CommonStop::default() }
    }
}

/// Inverse kinematics for an `N`-DoF arm.
///
/// Builds on [`ArmForwardKinematics`] and the joint limits of [`Joints<N>`].
/// Implementors may override [`ANALYTIC_FAMILY`](ArmInverseKinematics::ANALYTIC_FAMILY)
/// and [`ik_analytic_all`](ArmInverseKinematics::ik_analytic_all) to provide a
/// closed-form solver; otherwise the provided
/// [`ik_step`](ArmInverseKinematics::ik_step) drives the iterative
/// [`IKMethod`]s, always projecting results back into the joint limits.
pub trait ArmInverseKinematics<const N: usize>: ArmForwardKinematics<N> + Joints<N>
where
    [(); N + 1]:,
{
    /// The analytic family this arm belongs to, or `None` for iterative-only.
    const ANALYTIC_FAMILY: Option<AnalyticFamily> = None;

    /// All analytic IK solutions for `target`, or `None` if no analytic solver
    /// is implemented. Default returns `None`.
    fn ik_analytic_all(target: &Pose) -> Option<Vec<JVec<N>>> {
        let _ = target;
        None
    }

    /// The analytic solution closest to `q_seed`, if any exist.
    fn ik_analytic_best(q_seed: &JVec<N>, target: &Pose) -> Option<JVec<N>> {
        Self::ik_analytic_all(target).and_then(|mut sols| {
            if sols.is_empty() {
                return None;
            }
            sols.sort_by(|a, b| {
                ((a - *q_seed).norm())
                    .partial_cmp(&(b - *q_seed).norm())
                    .unwrap()
            });
            Some(sols[0])
        })
    }

    // 计算 task 残差 e 和（可选）J；默认几何位姿误差 + 几何雅可比
    /// Compute the task-space error twist `e` (and geometric Jacobian) between
    /// the current pose at `q` and `target`. Default uses a geometric pose
    /// error in the base frame plus the geometric Jacobian.
    fn task_error_and_jacobian(q: &JVec<N>, target: &Pose) -> (Twist, Jaco<N>) {
        let k = Self::kine_cache(q.as_slice().try_into().unwrap(), &[0.0; N]);
        let cur = k.end_effector_pose();
        let tgt = target.quat();
        let dp = tgt.translation.vector - cur.translation.vector;
        // rrel must be expressed in the BASE frame to match the geometric
        // Jacobian's angular rows; `cur^{-1} * tgt` is the EE-frame variant
        // and would mix coordinate systems with `dp`.
        let rrel = tgt.rotation * cur.rotation.inverse();
        let drot = rrel.scaled_axis();

        let e = Twist::from_row_slice(&[dp.x, dp.y, dp.z, drot.x, drot.y, drot.z]);
        (e, k.jacobian())
    }

    // ========== 单步迭代器（按方法）==========
    /// Perform a single IK update of `q` toward `target` using `method`,
    /// returning the next joint vector clamped into the arm's joint limits.
    fn ik_step(q: &JVec<N>, target: &Pose, method: &IKMethod) -> JVec<N>
    where
        na::Const<N>: na::DimMin<na::Const<N>, Output = na::Const<N>> + na::ToTypenum,
    {
        match method {
            IKMethod::Analytic { fallback } => {
                if Self::ANALYTIC_FAMILY.is_some()
                    && let Some(sol) = Self::ik_analytic_best(q, target)
                {
                    return sol;
                }
                Self::ik_step(q, target, fallback)
            }
            IKMethod::DLS { lambda, stop } => {
                let (e, j) = Self::task_error_and_jacobian(q, target);
                let jj_t = j * j.transpose();
                let pinv = j.transpose()
                    * (jj_t + na::SMatrix::<f64, 6, 6>::identity() * (*lambda) * (*lambda))
                        .try_inverse()
                        .unwrap();
                let mut dq = pinv * e;
                clip_step(&mut dq, stop.step_clip);
                project_to_limits(q + dq, Self::JOINT_MIN, Self::JOINT_MAX)
            }
            IKMethod::JT { gain, stop } => {
                let (e, j) = Self::task_error_and_jacobian(q, target);
                let mut dq = j.transpose() * e * *gain;
                clip_step(&mut dq, stop.step_clip);
                project_to_limits(q + dq, Self::JOINT_MIN, Self::JOINT_MAX)
            }
            IKMethod::Newton { stop } => {
                let (e, j) = Self::task_error_and_jacobian(q, target);
                // 伪逆牛顿：Δq = J^+ e
                // SMatrix::as_slice() is column-major, so we MUST use
                // from_column_slice; the previous from_row_slice produced J^T.
                let j_dyn = na::DMatrix::from_column_slice(6, N, j.as_slice());
                let pinv = j_dyn.pseudo_inverse(1e-9).unwrap();
                let pinv = na::SMatrix::<f64, N, 6>::from_column_slice(pinv.as_slice());
                let mut dq = pinv * e;
                clip_step(&mut dq, stop.step_clip);
                project_to_limits(q + dq, Self::JOINT_MIN, Self::JOINT_MAX)
            }
            IKMethod::LM { lambda0, stop } => {
                // 经典 LM：Δq = (JᵀJ + λ I)^{-1} Jᵀ e
                let (e, j) = Self::task_error_and_jacobian(q, target);
                let jt = j.transpose();
                let h = jt * j + na::SMatrix::<f64, N, N>::identity() * (*lambda0);
                let mut dq = h.lu().solve(&(jt * e)).unwrap_or(JVec::<N>::zeros());
                clip_step(&mut dq, stop.step_clip);
                project_to_limits(q + dq, Self::JOINT_MIN, Self::JOINT_MAX)
            }
        }
    }

    // ========== 顶层：迭代求解（位姿 IK）==========
    fn ik_solve(q0: &JVec<N>, target: &Pose, method: IKMethod) -> JVec<N>
    where
        na::Const<N>: na::DimMin<na::Const<N>, Output = na::Const<N>> + na::ToTypenum,
    {
        // 优先解析（当 method=Analytic 且支持时）
        if let IKMethod::Analytic { .. } = method
            && Self::ANALYTIC_FAMILY.is_some()
            && let Some(sol) = Self::ik_analytic_best(q0, target)
        {
            return sol;
        }
        // 迭代
        let stop = match &method {
            IKMethod::DLS { stop, .. }
            | IKMethod::JT { stop, .. }
            | IKMethod::Newton { stop }
            | IKMethod::LM { stop, .. } => *stop,
            IKMethod::Analytic { fallback } => match &**fallback {
                IKMethod::DLS { stop, .. }
                | IKMethod::JT { stop, .. }
                | IKMethod::Newton { stop }
                | IKMethod::LM { stop, .. } => *stop,
                _ => CommonStop::default(),
            },
        };

        let mut q = *q0;
        for _ in 0..stop.max_iters {
            let qn = Self::ik_step(&q, target, &method);
            if (qn - q).amax() < 1e-10 {
                break;
            }
            q = qn;
            // 简单收敛终止（位姿误差门限）
            let (e, _) = Self::task_error_and_jacobian(&q, target);
            let pos_ok = e.fixed_rows::<3>(0).norm() < stop.pos_tol;
            let rot_ok = e.fixed_rows::<3>(3).norm() < stop.rot_tol;
            if pos_ok && rot_ok {
                break;
            }
        }
        q
    }
}

#[inline]
fn project_to_limits<const N: usize>(mut q: JVec<N>, min: [f64; N], max: [f64; N]) -> JVec<N> {
    for i in 0..N {
        q[i] = q[i].clamp(min[i], max[i]);
    }
    q
}
#[inline]
fn clip_step<const N: usize>(dq: &mut JVec<N>, clip: f64) {
    for i in 0..N {
        dq[i] = dq[i].clamp(-clip, clip);
    }
}

pub trait ArmDynamics<const N: usize> {}
