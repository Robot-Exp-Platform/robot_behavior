//! Inverse kinematics for a URDF chain.

use nalgebra as na;
use roplat_behavior::Pose;
use roplat_description::{Chain, DescriptionError};

use crate::{ArmKineCache, CommonStop, DhParam};

/// The result of [`chain_ik`]: the last joint vector and how close it got.
#[derive(Clone, Debug, PartialEq)]
pub struct ChainIkSolution {
    /// Positions of the chain's movable joints, within their limits.
    pub q: Vec<f64>,
    /// Both errors are within the tolerances of the [`CommonStop`].
    pub converged: bool,
    pub iterations: usize,
    /// Tip position error (m) at `q`.
    pub position_error: f64,
    /// Tip orientation error (rad) at `q`.
    pub rotation_error: f64,
}

/// Damped least squares on the chain's geometric Jacobian, for any mix of
/// revolute, continuous and prismatic joints: each step is
/// `Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e`, clipped to `stop.step_clip` per joint and
/// projected into the URDF position limits.
///
/// The error `e` is the tip's position and orientation error in the base
/// frame, as in [`ArmInverseKinematics`](crate::ArmInverseKinematics). An
/// unreachable target is not an error: the solution says it did not converge.
pub fn chain_ik(
    chain: &Chain,
    q0: &[f64],
    target: &Pose,
    lambda: f64,
    stop: CommonStop,
) -> Result<ChainIkSolution, DescriptionError> {
    let dof = chain.dof();
    if q0.len() != dof {
        return Err(DescriptionError::DofMismatch { expected: dof, got: q0.len() });
    }
    let (lower, upper): (Vec<f64>, Vec<f64>) = chain
        .movable()
        .map(|j| {
            j.limit
                .map_or((f64::NEG_INFINITY, f64::INFINITY), |l| (l.lower, l.upper))
        })
        .unzip();
    let project = |q: &mut na::DVector<f64>| {
        for i in 0..dof {
            q[i] = q[i].clamp(lower[i], upper[i]);
        }
    };
    let target = target.quat();
    let error = |q: &na::DVector<f64>| -> Result<na::Vector6<f64>, DescriptionError> {
        let tip = chain.forward_kinematics_iso(q.as_slice())?;
        let dp = target.translation.vector - tip.translation.vector;
        let dr = (target.rotation * tip.rotation.inverse()).scaled_axis();
        Ok(na::Vector6::new(dp.x, dp.y, dp.z, dr.x, dr.y, dr.z))
    };
    let done = |e: &na::Vector6<f64>| {
        e.fixed_rows::<3>(0).norm() < stop.pos_tol && e.fixed_rows::<3>(3).norm() < stop.rot_tol
    };

    let mut q = na::DVector::from_column_slice(q0);
    project(&mut q);
    let mut e = error(&q)?;
    let mut iterations = 0;
    while !done(&e) && iterations < stop.max_iters {
        let j = chain.jacobian(q.as_slice())?;
        let damped = &j * j.transpose() + na::Matrix6::identity() * (lambda * lambda);
        let Some(inverse) = damped.try_inverse() else {
            break;
        };
        let mut dq = j.transpose() * (inverse * e);
        dq.apply(|d| *d = d.clamp(-stop.step_clip, stop.step_clip));
        let mut next = &q + dq;
        project(&mut next);
        iterations += 1;
        if (&next - &q).amax() < 1e-12 {
            break;
        }
        q = next;
        e = error(&q)?;
    }
    Ok(ChainIkSolution {
        converged: done(&e),
        position_error: e.fixed_rows::<3>(0).norm(),
        rotation_error: e.fixed_rows::<3>(3).norm(),
        q: q.as_slice().to_vec(),
        iterations,
    })
}

/// Settings for [`chain_ik_solutions`].
#[derive(Clone, Copy, Debug)]
pub struct IkSearch {
    /// Random starts tried after the reference.
    pub restarts: usize,
    /// Stop once this many distinct solutions are found.
    pub max_solutions: usize,
    /// Two solutions whose joints all differ by less than this are one.
    pub distinct: f64,
    /// Damping of each [`chain_ik`] run; heavier than for a nearby seed,
    /// which converges from more random starts.
    pub lambda: f64,
    pub stop: CommonStop,
    /// Seed of the start sampler: a seed always gives the same solutions.
    pub seed: u64,
}

impl Default for IkSearch {
    fn default() -> Self {
        Self {
            restarts: 64,
            max_solutions: 16,
            distinct: 1e-3,
            lambda: 0.1,
            stop: CommonStop { pos_tol: 1e-6, rot_tol: 1e-6, max_iters: 200, step_clip: 0.2 },
            seed: 1,
        }
    }
}

/// Distinct joint vectors reaching `target` that lie within the chain's URDF
/// limits and pass `is_valid` (e.g. a collision check), nearest to
/// `reference` first by the largest joint difference.
///
/// [`chain_ik`] runs from `reference`, then from up to `search.restarts`
/// random starts within the limits (`reference ± π` for an unbounded
/// joint); only converged runs count. An empty result means none was found
/// within the budget, not that none exists.
pub fn chain_ik_solutions(
    chain: &Chain,
    target: &Pose,
    reference: &[f64],
    mut is_valid: impl FnMut(&[f64]) -> bool,
    search: IkSearch,
) -> Result<Vec<Vec<f64>>, DescriptionError> {
    let dof = chain.dof();
    if reference.len() != dof {
        return Err(DescriptionError::DofMismatch { expected: dof, got: reference.len() });
    }
    let bounds: Vec<(f64, f64)> = chain
        .movable()
        .zip(reference)
        .map(|(j, &r)| {
            j.limit
                .filter(|l| l.lower.is_finite() && l.upper.is_finite() && l.lower < l.upper)
                .map_or((r - std::f64::consts::PI, r + std::f64::consts::PI), |l| {
                    (l.lower, l.upper)
                })
        })
        .collect();
    let mut state = search.seed.max(1);
    let mut unit = move || {
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        (state.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1u64 << 53) as f64
    };
    let distance = |a: &[f64], b: &[f64]| {
        a.iter()
            .zip(b)
            .map(|(x, y)| (x - y).abs())
            .fold(0.0, f64::max)
    };

    let mut found: Vec<Vec<f64>> = Vec::new();
    for attempt in 0..=search.restarts {
        if found.len() >= search.max_solutions {
            break;
        }
        let start: Vec<f64> = if attempt == 0 {
            reference.to_vec()
        } else {
            bounds
                .iter()
                .map(|&(lo, hi)| lo + unit() * (hi - lo))
                .collect()
        };
        let solution = chain_ik(chain, &start, target, search.lambda, search.stop)?;
        // chain_ik keeps every iterate within the URDF limits.
        if !solution.converged
            || found
                .iter()
                .any(|q| distance(q, &solution.q) < search.distinct)
            || !is_valid(&solution.q)
        {
            continue;
        }
        found.push(solution.q);
    }
    found.sort_by(|a, b| distance(a, reference).total_cmp(&distance(b, reference)));
    Ok(found)
}

/// How far a DH table and a URDF chain are from describing the same arm.
///
/// Two models of one arm may place the end effector differently, so they
/// agree when `chain(q) = dh(q) · E` for one constant tool transform `E`.
/// This evaluates both at each of `samples`, takes `E` from the first, and
/// returns the largest deviation from it, as metres of translation plus
/// radians of rotation; near zero means the same kinematics. A different
/// joint direction or zero posture gives a deviation of order one. Both
/// models must share the base frame.
pub fn dh_chain_discrepancy<const N: usize>(
    dh: &[DhParam; N],
    chain: &Chain,
    samples: &[[f64; N]],
) -> Result<f64, DescriptionError> {
    let tool = |q: &[f64; N]| -> Result<na::Isometry3<f64>, DescriptionError> {
        let dh = ArmKineCache::build(dh, q, &[0.0; N]).end_effector_pose();
        Ok(dh.inv_mul(&chain.forward_kinematics_iso(q)?))
    };
    let Some(first) = samples.first() else {
        return Ok(0.0);
    };
    let reference = tool(first)?;
    let mut worst: f64 = 0.0;
    for q in &samples[1..] {
        let e = tool(q)?;
        let deviation = (e.translation.vector - reference.translation.vector).norm()
            + e.rotation.angle_to(&reference.rotation);
        worst = worst.max(deviation);
    }
    Ok(worst)
}
