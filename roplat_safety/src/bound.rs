//! Element-wise bounds shared by the channel checks. Every comparison is
//! written so that NaN, in a value or in a limit, refuses.

use crate::{Axis, Quantity, SafetyViolation};

pub(crate) fn within(value: f64, max: f64) -> bool {
    value <= max
}

/// Whether any of these magnitude limits is finite, so a check needs data.
pub(crate) fn bounded(max: &[f64]) -> bool {
    max.iter().any(|&max| max != f64::INFINITY)
}

pub(crate) fn finite<const N: usize>(
    quantity: Quantity,
    values: &[f64; N],
) -> Result<(), SafetyViolation> {
    match values.iter().position(|value| !value.is_finite()) {
        Some(joint) => Err(SafetyViolation::NonFinite { quantity, axis: Axis::Joint(joint) }),
        None => Ok(()),
    }
}

pub(crate) fn range<const N: usize>(
    quantity: Quantity,
    values: &[f64; N],
    min: &[f64; N],
    max: &[f64; N],
) -> Result<(), SafetyViolation> {
    for (joint, ((&value, &min), &max)) in values.iter().zip(min).zip(max).enumerate() {
        let inside = min <= value && value <= max;
        if !inside {
            return Err(SafetyViolation::OutOfRange {
                quantity,
                axis: Axis::Joint(joint),
                value,
                min,
                max,
            });
        }
    }
    Ok(())
}

pub(crate) fn magnitude<const N: usize>(
    quantity: Quantity,
    values: &[f64; N],
    max: &[f64; N],
) -> Result<(), SafetyViolation> {
    for (joint, (&value, &max)) in values.iter().zip(max).enumerate() {
        if !within(value.abs(), max) {
            return Err(SafetyViolation::Exceeded {
                quantity,
                axis: Axis::Joint(joint),
                value,
                max,
            });
        }
    }
    Ok(())
}

/// `|next - previous| <= max * dt`, reported as a rate. Infinite limits are
/// not checked; over a zero `dt` only an unchanged value passes.
pub(crate) fn rate<const N: usize>(
    quantity: Quantity,
    previous: &[f64; N],
    next: &[f64; N],
    max: &[f64; N],
    dt: f64,
) -> Result<(), SafetyViolation> {
    for (joint, ((&previous, &next), &max)) in previous.iter().zip(next).zip(max).enumerate() {
        let change = (next - previous).abs();
        if max != f64::INFINITY && !within(change, max * dt) {
            return Err(SafetyViolation::Exceeded {
                quantity,
                axis: Axis::Joint(joint),
                value: change / dt,
                max,
            });
        }
    }
    Ok(())
}

pub(crate) fn norm_rate(
    quantity: Quantity,
    axis: Axis,
    change: f64,
    max: f64,
    dt: f64,
) -> Result<(), SafetyViolation> {
    if max != f64::INFINITY && !within(change, max * dt) {
        return Err(SafetyViolation::Exceeded { quantity, axis, value: change / dt, max });
    }
    Ok(())
}

pub(crate) fn norm(
    quantity: Quantity,
    axis: Axis,
    value: f64,
    max: f64,
) -> Result<(), SafetyViolation> {
    if !within(value, max) {
        return Err(SafetyViolation::Exceeded { quantity, axis, value, max });
    }
    Ok(())
}

/// A measured reference. Only checks with a finite limit need it; the others
/// never read the placeholder.
pub(crate) fn measured<const N: usize>(
    quantity: Quantity,
    value: Option<[f64; N]>,
    needed: bool,
) -> Result<[f64; N], SafetyViolation> {
    match value {
        Some(value) if value.iter().all(|v| v.is_finite()) => Ok(value),
        _ if !needed => Ok([0.; N]),
        _ => Err(SafetyViolation::Unobserved { quantity }),
    }
}
