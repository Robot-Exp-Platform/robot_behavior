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
