use nalgebra::{self as na, SVector};

use crate::{ArmParam, DhParam, Pose};

pub type Iso3 = na::Isometry3<f64>; // SE(3)
pub type Twist = na::SVector<f64, 6>; // [vx, vy, vz, wx, wy, wz]
pub type Wrench = na::SVector<f64, 6>;
pub type JVec<const N: usize> = na::SVector<f64, N>;
pub type JMat<const N: usize> = na::SMatrix<f64, N, N>;
pub type Jaco<const N: usize> = na::SMatrix<f64, 6, N>;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct Link(pub usize); // 0..=N，0 表示基座，N 末端
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct Joint(pub usize); // 1..=N

#[derive(Clone, Copy, Debug)]
pub enum JointType {
    Revolute,
    Prismatic,
}

pub struct ArmKineCache<const N: usize>
where
    [(); N + 1]:,
{
    pub q_dot: [f64; N],
    pub t_prefix: [na::Isometry3<f64>; N + 1],
    pub origins: [na::Vector3<f64>; N + 1],
    pub z_axes: [na::Vector3<f64>; N + 1],
}

impl<const N: usize> ArmKineCache<N>
where
    [(); N + 1]:,
{
    #[inline(always)]
    pub fn build(dh: &[DhParam; N], q: &[f64; N], q_dot: &[f64; N]) -> Self {
        let mut t_prefix = [na::Isometry3::identity(); N + 1];
        let mut origins = [na::Vector3::zeros(); N + 1];
        let mut z_axes = [na::Vector3::z(); N + 1];

        for i in 0..N {
            t_prefix[i + 1] = t_prefix[i] * dh[i].to_se3(q[i]);
            origins[i + 1] = t_prefix[i + 1].translation.vector;
            z_axes[i + 1] = t_prefix[i + 1].rotation * na::Vector3::z();
        }

        Self { q_dot: *q_dot, t_prefix, origins, z_axes }
    }

    #[inline(always)]
    pub fn end_effector_pose(&self) -> na::Isometry3<f64> {
        self.t_prefix[N]
    }

    #[inline(always)]
    pub fn link_pose(&self, link: Link) -> na::Isometry3<f64> {
        assert!(link.0 <= N, "link index must be less than or equal to N");
        self.t_prefix[link.0]
    }

    #[inline(always)]
    pub fn relative_pose(&self, a: Link, b: Link) -> na::Isometry3<f64> {
        assert!(a.0 <= N && b.0 <= N, "Link index out of bounds");
        self.t_prefix[b.0].inv_mul(&self.t_prefix[a.0])
    }

    #[inline(always)]
    pub fn link_poses(&self, indices: &[Link]) -> Vec<na::Isometry3<f64>> {
        indices.iter().map(|&i| self.link_pose(i)).collect()
    }

    #[inline(always)]
    pub fn jacobian(&self) -> na::SMatrix<f64, 6, N> {
        let mut j = na::SMatrix::<f64, 6, N>::zeros();
        let p_n = self.origins[N];
        for i in 0..N {
            let z = self.z_axes[i]; // z_{i-1}
            let p = self.origins[i]; // p_{i-1}
            let jv = z.cross(&(p_n - p));
            j.fixed_view_mut::<3, 1>(0, i).copy_from(&jv);
            j.fixed_view_mut::<3, 1>(3, i).copy_from(&z);
        }
        j
    }

    #[inline(always)]
    pub fn jacobian_at_link(&self, link_index: usize) -> na::SMatrix<f64, 6, N> {
        assert!(
            link_index <= N,
            "link_index must be less than or equal to N"
        );
        let mut j = na::SMatrix::<f64, 6, N>::zeros();
        let p_n = self.origins[link_index];
        for i in 0..link_index {
            let z = self.z_axes[i]; // z_{i-1}
            let p = self.origins[i]; // p_{i-1}
            let jv = z.cross(&(p_n - p));
            j.fixed_view_mut::<3, 1>(0, i).copy_from(&jv);
            j.fixed_view_mut::<3, 1>(3, i).copy_from(&z);
        }
        j
    }

    #[inline(always)]
    pub fn ee_twist(&self) -> Twist {
        self.jacobian() * SVector::<f64, N>::from_column_slice(&self.q_dot)
    }
}

pub trait ArmForwardKinematics<const N: usize>
where
    [(); N + 1]:,
{
    const DH: [DhParam; N];

    #[inline(always)]
    fn kine_cache(q: &[f64; N], q_dot: &[f64; N]) -> ArmKineCache<N> {
        ArmKineCache::build(&Self::DH, q, q_dot)
    }

    #[inline(always)]
    fn fk_end_pose(q: &[f64; N]) -> Pose {
        let k = Self::kine_cache(q, &[0.0; N]);
        Pose::Quat(k.end_effector_pose())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AnalyticFamily {
    Planar2R,
    Planar3R,
    ScaraRRPR,
    Spherical6R,
    Scara,
    Custom, // 你有自定义解析实现
}

#[derive(Clone, Copy, Debug)]
pub struct CommonStop {
    pub pos_tol: f64,
    pub rot_tol: f64,
    pub max_iters: usize,
    pub step_clip: f64, // 每步 |Δq_i| 限制
}
impl Default for CommonStop {
    fn default() -> Self {
        Self { pos_tol: 1e-4, rot_tol: 1e-4, max_iters: 200, step_clip: 0.2 }
    }
}

#[derive(Clone, Debug)]
pub enum IKMethod {
    /// 解析（如果 RigidArm::ANALYTIC_FAMILY != None 则尝试解析；否则回退到 fallback）
    Analytic { fallback: Box<IKMethod> }, // fallback 例如 DLS
    /// Damped Least Squares （Levenberg-Marquardt 样式）
    DLS { lambda: f64, stop: CommonStop },
    /// Jacobian Transpose（简单鲁棒但收敛慢）
    JT { gain: f64, stop: CommonStop },
    /// Newton-Raphson（J^{-1} 或 伪逆，适合方阵/条件好）
    Newton { stop: CommonStop },
    /// Levenberg–Marquardt（对几何残差，阻尼可自适应）
    LM { lambda0: f64, stop: CommonStop },
}

pub trait ArmInverseKinematics<const N: usize>: ArmForwardKinematics<N> + ArmParam<N>
where
    [(); N + 1]:,
{
    const ANALYTIC_FAMILY: Option<AnalyticFamily> = None;

    fn ik_analytic_all(target: &Pose) -> Option<Vec<JVec<N>>> {
        let _ = target;
        None
    }

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
    fn task_error_and_jacobian(q: &JVec<N>, target: &Pose) -> (Twist, Jaco<N>) {
        let k = Self::kine_cache(q.as_slice().try_into().unwrap(), &[0.0; N]);
        let cur = k.end_effector_pose();
        let tgt = target.quat();
        let dp = tgt.translation.vector - cur.translation.vector;
        let rrel = cur.rotation.inverse() * tgt.rotation;
        let drot = rrel
            .axis()
            .map(|ax| ax.into_inner() * rrel.angle())
            .unwrap_or(na::Vector3::zeros());

        let e = Twist::from_row_slice(&[dp.x, dp.y, dp.z, drot.x, drot.y, drot.z]);
        (e, k.jacobian())
    }

    // ========== 单步迭代器（按方法）==========
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
                let j_dyn = na::DMatrix::from_row_slice(6, N, j.as_slice());
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
